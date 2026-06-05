#!/usr/bin/env python3
"""
central_server_gnn.py  —  GNN-based map fusion module
======================================================
Two execution paths:

  1. Trained PyTorch model (preferred):
     Loads /ros2_ws/results/gnn/gnn_fusion.pt if available.
     Uses CNN-based GNNMapFusion from train_gnn.py for per-robot weights.

  2. NumPy GAT fallback:
     2-layer Graph Attention Network, pure NumPy, no ML framework needed.
     Weights initialised analytically with optional online self-supervised
     update (finite-difference gradient on W_out only — 8 parameters).

Public API used by central_server_unified.py:
  load_gnn_model(n_robots)          -> OccupancyGridGNN instance
  update_gnn(model, maps, res, pc)  -> updates W_out weights in-place
  fuse_with_gnn(model, maps, res)   -> OccupancyGrid | None
  fuse_with_gnn_trained(maps, res)  -> OccupancyGrid | None

Changes from v1:
  FIX1: _node_features division by zero when all point counts = 0
  FIX2: _pytorch_gnn_weights file handle leak (open without close)
  FIX3: torch.load weights_only=False to suppress FutureWarning flood
  CLEAN: _weighted_fuse extracted to remove code duplication between
         OccupancyGridGNN.fuse() and fuse_with_gnn()
"""

import numpy as np
import os as _os
from nav_msgs.msg import OccupancyGrid

# -- Constants ----------------------------------------------------------------
FEAT_DIM          = 5
HIDDEN_DIM        = 8
EDGE_DIM          = 2
ATT_DIM           = HIDDEN_DIM * 2 + EDGE_DIM
MIN_OVERLAP_CELLS = 10
P_OCC_THRESH      = 0.65
P_FREE_THRESH     = 0.35
LEAKY_ALPHA       = 0.2

# -- Activations --------------------------------------------------------------
def _relu(x):                    return np.maximum(0.0, x)
def _leaky_relu(x, alpha=LEAKY_ALPHA): return np.where(x >= 0, x, alpha * x)
def _sigmoid(x):                 return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))
def _softmax(x):                 e = np.exp(x - x.max()); return e / e.sum()


# -- Weight initialisation ----------------------------------------------------

def _init_weights(n_robots, seed=42):
    """
    Analytically-primed random weights for the 2-layer GAT.
    Priors encode:
      W_self_0[:,0] += 0.3   upweight occupied fraction
      W_self_0[:,2] -= 0.2   downweight unknown fraction
      a0[2*HIDDEN+1] += 0.5  boost overlap edge attention
    """
    rng = np.random.default_rng(seed)
    def r(*s): return rng.standard_normal(s).astype(np.float32) * 0.05

    W0 = r(HIDDEN_DIM, FEAT_DIM);  N0 = r(HIDDEN_DIM, FEAT_DIM);  a0 = r(ATT_DIM)
    W0[:, 0] += 0.3;  W0[:, 2] -= 0.2;  W0[:, 4] += 0.2
    N0[:, 0] += 0.2;  N0[:, 4] += 0.2;  a0[2 * HIDDEN_DIM + 1] += 0.5
    W1 = r(HIDDEN_DIM, HIDDEN_DIM)
    N1 = r(HIDDEN_DIM, HIDDEN_DIM)
    a1 = r(HIDDEN_DIM * 2 + EDGE_DIM)
    Wo = r(1, HIDDEN_DIM);  Wo[0, :] += 0.1

    return {
        "W_self_0": W0, "W_neigh_0": N0, "att_0": a0,
        "W_self_1": W1, "W_neigh_1": N1, "att_1": a1,
        "W_out":    Wo,
    }


# -- Grid helpers -------------------------------------------------------------

def _grid_to_prob(d):
    """OccupancyGrid int8 -> probability float32: 100->1.0, 0->0.0, -1->0.5"""
    return np.where(d == 100, 1.0, np.where(d == 0, 0.0, 0.5)).astype(np.float32)


def _world_bounds(maps, resolution):
    mnx = mny =  float("inf")
    mxx = mxy = float("-inf")
    for m in maps.values():
        ox = m.info.origin.position.x;  oy = m.info.origin.position.y
        mnx = min(mnx, ox);  mny = min(mny, oy)
        mxx = max(mxx, ox + m.info.width  * resolution)
        mxy = max(mxy, oy + m.info.height * resolution)
    return mnx, mny, mxx, mxy


def _map_to_full_grid(m, mnx, mny, W, H, res):
    """Project a per-robot OccupancyGrid into the world-frame canvas."""
    g  = np.full((H, W), 0.5, dtype=np.float32)
    xo = int(round((m.info.origin.position.x - mnx) / res))
    yo = int(round((m.info.origin.position.y - mny) / res))
    d  = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    p  = _grid_to_prob(d)
    re = min(yo + m.info.height, H)
    ce = min(xo + m.info.width,  W)
    if re > yo and ce > xo and yo >= 0 and xo >= 0:
        g[yo:re, xo:ce] = p[:re - yo, :ce - xo]
    return g


def _sharpness(g):
    """
    Mean run-length of occupied cells (lower = sharper/thinner walls).
    Padding ensures every start transition has a matching end transition
    so np.where(starts) and np.where(ends) always return equal-length arrays.
    """
    occ = (g == 1.0)
    p   = np.pad(occ, ((0, 0), (1, 1)), constant_values=False)
    s   = ~p[:, :-1] &  p[:, 1:]
    e   =  p[:, :-1] & ~p[:, 1:]
    rl  = np.where(e)[1] - np.where(s)[1]
    return float(rl.mean()) if len(rl) > 0 else 1.0


# -- Node / edge features -----------------------------------------------------

def _node_features(grids, names, pc=None):
    """
    5-dim feature vector per robot:
      [occ_frac, free_frac, unknown_frac, point_density, sharpness]

    FIX1: max(max(pc.values()), 1) prevents division by zero when all
    robots have 0 accumulated points at the start of a run.
    """
    n  = len(names)
    F  = np.zeros((n, FEAT_DIM), dtype=np.float32)
    mp = max(max(pc.values()), 1) if pc else 1   # FIX1
    for i, name in enumerate(names):
        g  = grids[name]
        km = g != 0.5
        nk = km.sum()
        if nk == 0:
            F[i] = [0, 0, 1, 0, 0]
            continue
        oc = float((g == 1.0).sum()) / nk
        fr = float((g == 0.0).sum()) / nk
        dn = (pc[name] / mp) if pc else 0.5
        sh = min(_sharpness(g) / 10.0, 1.0)
        F[i] = [oc, fr, 1 - oc - fr, float(dn), sh]
    return F


def _edge_features(grids, names):
    """
    2-dim edge feature for each overlapping robot pair:
      [overlap_ratio, agreement_ratio]
    Only pairs with >= MIN_OVERLAP_CELLS known cells in common get an edge.
    """
    n     = len(names)
    edges = {}
    for i in range(n):
        for j in range(i + 1, n):
            gi = grids[names[i]];  gj = grids[names[j]]
            ki = gi != 0.5;        kj = gj != 0.5
            both   = ki & kj
            either = ki | kj
            nb = int(both.sum())
            if nb < MIN_OVERLAP_CELLS:
                continue
            ov = nb / max(int(either.sum()), 1)
            ag = float((both & (np.abs(gi - gj) < 0.5)).sum()) / nb
            ef = np.array([ov, ag], dtype=np.float32)
            edges[(i, j)] = ef
            edges[(j, i)] = ef
    return edges


# -- GNN forward pass ---------------------------------------------------------

def _gnn_forward(F, edges, weights):
    """
    2-layer Graph Attention Network forward pass.
    Returns per-robot confidence weights that sum to 1.
    """
    n = F.shape[0]
    H = F.copy()

    for layer in range(2):
        Ws = weights[f"W_self_{layer}"]
        Wn = weights[f"W_neigh_{layer}"]
        av = weights[f"att_{layer}"]
        Hn = np.zeros((n, HIDDEN_DIM), dtype=np.float32)

        for i in range(n):
            sm = Ws @ H[i]
            ni = [j for (ii, j) in edges if ii == i]
            if not ni:
                Hn[i] = _relu(sm)
                continue
            ats = [];  nms = []
            for j in ni:
                nm = Wn @ H[j]
                ats.append(_leaky_relu(av @ np.concatenate([sm, nm, edges[(i, j)]])))
                nms.append(nm)
            aw  = _softmax(np.array(ats, dtype=np.float32))
            agg = sum(w * m for w, m in zip(aw, nms))
            Hn[i] = _relu(sm + agg)
        H = Hn

    c = _sigmoid((weights["W_out"] @ H.T).flatten())
    t = c.sum()
    return (c / t).astype(np.float32) if t > 1e-8 else np.ones(n, dtype=np.float32) / n


# -- Shared weighted fusion ---------------------------------------------------

def _weighted_fuse(fg, names, rw, mnx, mny, W, H, resolution):
    """
    Weighted average of per-robot probability grids.
    Unknown cells (0.5) are excluded from the weighted sum.
    Thresholds result to OccupancyGrid int8 values.

    Used by both OccupancyGridGNN.fuse() and fuse_with_gnn() to avoid
    code duplication.
    """
    fp = np.zeros((H, W), dtype=np.float32)
    tw = np.zeros((H, W), dtype=np.float32)
    for i, name in enumerate(names):
        g  = fg[name]
        kn = (g != 0.5).astype(np.float32)
        fp += rw[i] * g  * kn
        tw += rw[i] * kn
    ob      = tw > 1e-8
    fp[ob] /= tw[ob]
    fp[~ob] = 0.5

    mg = np.full((H, W), -1, dtype=np.int8)
    mg[fp >  P_OCC_THRESH]  = 100
    mg[fp <  P_FREE_THRESH] = 0

    out = OccupancyGrid()
    out.header.frame_id            = "world"
    out.info.resolution            = resolution
    out.info.width                 = W
    out.info.height                = H
    out.info.origin.position.x    = float(mnx)
    out.info.origin.position.y    = float(mny)
    out.info.origin.orientation.w = 1.0
    out.data = mg.flatten().tolist()
    return out


# -- OccupancyGridGNN class ---------------------------------------------------

class OccupancyGridGNN:
    """
    Pure-NumPy GNN for occupancy grid fusion.

    fuse()           weighted map fusion using GNN per-robot confidence weights
    update_weights() online finite-difference update of W_out (8 params only)

    Online learning note: only W_out (output head, shape 1x8) is updated.
    Attention layers are frozen. Framed in thesis as "online adaptation of
    output confidence weights via finite-difference gradient descent."
    """

    def __init__(self, n_robots=3, seed=42, learning_rate=0.01):
        self.n_robots      = n_robots
        self.lr            = learning_rate
        self.weights       = _init_weights(n_robots, seed)
        self._update_count = 0

    def fuse(self, maps, resolution, point_counts=None):
        if not maps:
            return None
        names = sorted(maps.keys())
        mnx, mny, mxx, mxy = _world_bounds(maps, resolution)
        W = int(np.ceil((mxx - mnx) / resolution))
        H = int(np.ceil((mxy - mny) / resolution))
        if W <= 0 or H <= 0:
            return None
        fg    = {n: _map_to_full_grid(maps[n], mnx, mny, W, H, resolution) for n in names}
        pc    = point_counts or {n: 1 for n in names}
        F     = _node_features(fg, names, pc)
        edges = _edge_features(fg, names)
        rw    = _gnn_forward(F, edges, self.weights)
        return _weighted_fuse(fg, names, rw, mnx, mny, W, H, resolution)

    def update_weights(self, maps, resolution, point_counts=None):
        names = sorted(maps.keys())
        n     = len(names)
        mnx, mny, mxx, mxy = _world_bounds(maps, resolution)
        W = int(np.ceil((mxx - mnx) / resolution))
        H = int(np.ceil((mxy - mny) / resolution))
        if W <= 0 or H <= 0:
            return
        fg    = {nm: _map_to_full_grid(maps[nm], mnx, mny, W, H, resolution) for nm in names}
        edges = _edge_features(fg, names)
        if not edges:
            return
        pc = point_counts or {nm: 1 for nm in names}
        F  = _node_features(fg, names, pc)

        oc = None
        for (i, j) in [(e[0], e[1]) for e in edges if e[0] < e[1]]:
            mask = (fg[names[i]] != 0.5) & (fg[names[j]] != 0.5)
            oc   = mask if oc is None else (oc | mask)
        if oc is None or oc.sum() < MIN_OVERLAP_CELLS:
            return

        def loss(w):
            rw = _gnn_forward(F, edges, w)
            fs = np.zeros((H, W), dtype=np.float32)
            tw = np.zeros((H, W), dtype=np.float32)
            for i, nm in enumerate(names):
                g  = fg[nm]
                kn = (g != 0.5).astype(np.float32)
                fs += rw[i] * g * kn
                tw += rw[i] * kn
            ob = tw > 1e-8
            fs[ob]  /= tw[ob]
            fs[~ob]  = 0.5
            l = 0.0
            for nm in names:
                g  = fg[nm]
                kn = (g != 0.5) & oc
                if kn.sum() == 0:
                    continue
                l += float(np.mean((fs[kn] - g[kn]) ** 2))
            return l / max(n, 1)

        eps  = 1e-3
        bl   = loss(self.weights)
        Wo   = self.weights["W_out"]
        grad = np.zeros_like(Wo)
        for idx in np.ndindex(*Wo.shape):
            Wo[idx]   += eps
            grad[idx]  = (loss(self.weights) - bl) / eps
            Wo[idx]   -= eps
        self.weights["W_out"] = Wo - self.lr * grad
        self._update_count   += 1


# -- Singleton loader ---------------------------------------------------------

_DEFAULT_GNN = None

def load_gnn_model(n_robots=3):
    global _DEFAULT_GNN
    if _DEFAULT_GNN is None:
        _DEFAULT_GNN = OccupancyGridGNN(n_robots=n_robots)
    return _DEFAULT_GNN


def update_gnn(model, maps, resolution, point_counts=None):
    model.update_weights(maps, resolution, point_counts=point_counts)


# -- Trained PyTorch model ----------------------------------------------------

_TRAINED_GNN_MODEL  = None
_TRAINED_MODEL_PATH = "/ros2_ws/results/gnn/gnn_fusion.pt"
_GT_META_PATH       = "/ros2_ws/results/gnn/ground_truth_meta.json"
_TRAINED_GNN_LOADED = False


def _load_pytorch_gnn():
    global _TRAINED_GNN_MODEL, _TRAINED_GNN_LOADED
    if _TRAINED_GNN_LOADED:
        return _TRAINED_GNN_MODEL
    _TRAINED_GNN_LOADED = True

    if not _os.path.exists(_TRAINED_MODEL_PATH):
        print(f"[GNN] No trained model at {_TRAINED_MODEL_PATH}")
        return None
    if not _os.path.exists(_GT_META_PATH):
        print("[GNN] No ground truth meta — cannot use trained model")
        return None
    try:
        import torch, sys
        sys.path.insert(0, "/ros2_ws/src/multi_robot_slam/scripts")
        from train_gnn import GNNMapFusion
        # FIX3: weights_only=False suppresses FutureWarning in PyTorch 2.x
        ckpt  = torch.load(_TRAINED_MODEL_PATH, map_location="cpu", weights_only=False)
        model = GNNMapFusion()
        model.load_state_dict(ckpt["model_state"])
        model.eval()
        _TRAINED_GNN_MODEL = model
        print(f"[GNN] Loaded trained model — "
              f"val_MSE={ckpt['val_mse']:.6f}  epoch={ckpt['epoch']}")
        return model
    except Exception as e:
        print(f"[GNN] Failed to load trained model: {e}")
        return None


def _pytorch_gnn_weights(maps, resolution):
    """
    Run trained PyTorch model to get per-robot fusion weights.
    Returns {robot_name: float} or None if model unavailable/inference fails.
    """
    import json
    model = _load_pytorch_gnn()
    if model is None:
        return None
    try:
        import torch, sys
        sys.path.insert(0, "/ros2_ws/src/multi_robot_slam/scripts")
        from train_gnn import GRID_SIZE, project_to_world_grid

        # FIX2: context manager prevents file handle leak
        with open(_GT_META_PATH) as f:
            gt_meta = json.load(f)

        names     = sorted(maps.keys())
        projected = []
        for name in names:
            m  = maps[name]
            mm = {
                "origin_x":   m.info.origin.position.x,
                "origin_y":   m.info.origin.position.y,
                "resolution": resolution,
            }
            data = np.array(m.data, dtype=np.int8).reshape(
                m.info.height, m.info.width)
            projected.append(project_to_world_grid(data, mm, gt_meta, GRID_SIZE))

        X   = np.stack(projected)[:, np.newaxis, :, :]
        X_t = torch.from_numpy(X.astype(np.float32)).unsqueeze(0)
        with torch.no_grad():
            w = model(X_t).squeeze(0).numpy()
        return {name: float(w[i]) for i, name in enumerate(names)}

    except Exception as e:
        print(f"[GNN] Inference error: {e}")
        return None


# -- Public fusion functions --------------------------------------------------

def fuse_with_gnn(model, maps, resolution, point_counts=None):
    """
    Fuse maps using GNN-computed per-robot weights.
    Tries trained PyTorch model first; falls back to numpy GAT weights.
    model: OccupancyGridGNN instance (used as fallback weight source only)
    """
    if not maps:
        return None
    names = sorted(maps.keys())
    mnx, mny, mxx, mxy = _world_bounds(maps, resolution)
    W = int(np.ceil((mxx - mnx) / resolution))
    H = int(np.ceil((mxy - mny) / resolution))
    if W <= 0 or H <= 0:
        return None
    fg = {name: _map_to_full_grid(maps[name], mnx, mny, W, H, resolution)
          for name in names}

    trained_w = _pytorch_gnn_weights(maps, resolution)
    if trained_w is not None:
        rw = np.array([trained_w[name] for name in names], dtype=np.float32)
    else:
        pc    = point_counts or {name: 1 for name in names}
        F     = _node_features(fg, names, pc)
        edges = _edge_features(fg, names)
        rw    = _gnn_forward(F, edges, model.weights)

    return _weighted_fuse(fg, names, rw, mnx, mny, W, H, resolution)


def fuse_with_gnn_trained(maps, resolution, point_counts=None, fallback_model=None):
    """
    Entry point called by central_server_unified.gnn_merge().
    Uses trained .pt model if available, else numpy GAT fallback.
    """
    if fallback_model is None:
        fallback_model = load_gnn_model()
    return fuse_with_gnn(fallback_model, maps, resolution,
                         point_counts=point_counts)