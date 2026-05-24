#!/usr/bin/env python3
"""
central_server_gnn.py — Real GNN-based occupancy grid fusion module.

Architecture: 2-layer Graph Attention Network (GAT variant)
  implemented in pure NumPy — no PyTorch/TensorFlow dependency needed.
startslam
Graph definition:
  Nodes  = robots  (one node per robot)
  Edges  = pairs of robots whose maps share at least MIN_OVERLAP_CELLS cells
  Node features (5-dim per robot):
    f0  occupied_ratio   — fraction of cells in overlap zone that are occupied
    f1  free_ratio       — fraction that are free
    f2  unknown_ratio    — fraction that are unknown
    f3  density_norm     — normalised point-cloud density  (0..1)
    f4  sharpness_norm   — normalised wall-run sharpness   (0..1, higher = sharper walls)
  Edge features (2-dim per pair):
    e0  overlap_ratio    — |cells seen by both| / |cells seen by either|
    e1  agreement_ratio  — fraction of overlapping cells where both robots agree

Message passing (2 rounds):
  h_i^(l+1) = ReLU( W_self @ h_i^(l)
                   + sum_{j in N(i)} a_ij * W_neigh @ h_j^(l)  )
  where attention coefficient:
    a_ij = softmax_j( LeakyReLU( att_vec @ [h_i || h_j || e_ij] ) )

Output head:
  alpha_i = sigmoid( W_out @ h_i^(2) )   →  per-robot confidence weight
  weight_i = alpha_i / sum_j(alpha_j)    →  normalised fusion weight

Cell-level fusion:
  For each cell that at least one robot has observed:
    p_fused = sum_i  weight_i * p_i     where p_i ∈ {0.0, 0.5, 1.0}
                                              (free / unknown / occupied)
  Thresholded:  p > 0.65 → occupied (100)
                p < 0.35 → free     (0)
                else     → unknown  (-1)

Weights are initialised analytically (no training data needed) to reproduce
sensible fusion behaviour out-of-the-box.  An optional online self-supervised
update step can refine the weights using a consistency loss on overlap zones.

Usage (standalone):
    gnn = OccupancyGridGNN(n_robots=3)
    result_map = gnn.fuse(maps_dict, resolution=0.05)
"""

import numpy as np
from nav_msgs.msg import OccupancyGrid          # ROS2 import — fine to fail in tests

# ─────────────────────────────────────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────────────────────────────────────

FEAT_DIM   = 5    # node feature dimensionality
HIDDEN_DIM = 8    # hidden layer width
EDGE_DIM   = 2    # edge feature dimensionality
ATT_DIM    = HIDDEN_DIM * 2 + EDGE_DIM   # attention vector input size

MIN_OVERLAP_CELLS = 10    # minimum shared cells to form an edge
P_OCC_THRESH      = 0.65
P_FREE_THRESH     = 0.35
LEAKY_ALPHA       = 0.2   # LeakyReLU negative slope

# ─────────────────────────────────────────────────────────────────────────────
#  Activation helpers
# ─────────────────────────────────────────────────────────────────────────────

def _relu(x):
    return np.maximum(0.0, x)

def _leaky_relu(x, alpha=LEAKY_ALPHA):
    return np.where(x >= 0, x, alpha * x)

def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))

def _softmax(x):
    e = np.exp(x - x.max())
    return e / e.sum()


# ─────────────────────────────────────────────────────────────────────────────
#  Weight initialisation — analytical priors
# ─────────────────────────────────────────────────────────────────────────────

def _init_weights(n_robots: int, seed: int = 42) -> dict:
    """
    Initialise GNN weights with priors that encode sensible fusion behaviour:
    - self-loop weight matrix biased toward f0 (occupied_ratio) and f4 (sharpness)
    - neighbour weight matrix biased toward agreement  (e1 edge feature)
    - attention vector biased toward edge agreement e1
    - output head biased toward high sharpness and high occupied-ratio
    """
    rng = np.random.default_rng(seed)

    def small_randn(*shape):
        return rng.standard_normal(shape).astype(np.float32) * 0.05

    # Layer 0  (FEAT_DIM → HIDDEN_DIM)
    W_self_0  = small_randn(HIDDEN_DIM, FEAT_DIM)
    W_neigh_0 = small_randn(HIDDEN_DIM, FEAT_DIM)
    att_0     = small_randn(ATT_DIM)   # scalar attention per edge

    # Priors for layer 0:
    # h0: weight occupied_ratio (f0) and sharpness (f4) positively
    W_self_0[:, 0]  +=  0.3   # occupied_ratio → positive signal
    W_self_0[:, 2]  -= 0.2    # unknown_ratio  → negative signal
    W_self_0[:, 4]  +=  0.2   # sharpness      → positive signal
    # h0 neighbour: weight agreement (maps to e1 = index 2*HIDDEN_DIM+1 in att)
    W_neigh_0[:, 0] +=  0.2   # neighbour occupied_ratio
    W_neigh_0[:, 4] +=  0.2   # neighbour sharpness

    # Attention layer 0 — bias toward agreement (e1 sits at index 2*HIDDEN_DIM+1)
    att_0[2 * HIDDEN_DIM + 1] += 0.5   # agreement_ratio boosts attention

    # Layer 1  (HIDDEN_DIM → HIDDEN_DIM)
    W_self_1  = small_randn(HIDDEN_DIM, HIDDEN_DIM)
    W_neigh_1 = small_randn(HIDDEN_DIM, HIDDEN_DIM)
    att_1     = small_randn(HIDDEN_DIM * 2 + EDGE_DIM)

    # Output head  (HIDDEN_DIM → 1)
    W_out = small_randn(1, HIDDEN_DIM)
    W_out[0, :] += 0.1   # mild positive bias → moderate confidence by default

    return {
        "W_self_0":  W_self_0,
        "W_neigh_0": W_neigh_0,
        "att_0":     att_0,
        "W_self_1":  W_self_1,
        "W_neigh_1": W_neigh_1,
        "att_1":     att_1,
        "W_out":     W_out,
    }


# ─────────────────────────────────────────────────────────────────────────────
#  Grid utilities
# ─────────────────────────────────────────────────────────────────────────────

def _grid_to_prob(data_int8: np.ndarray) -> np.ndarray:
    """Convert OccupancyGrid int8 → float probability  (-1→0.5, 0→0.0, 100→1.0)."""
    p = np.where(data_int8 == 100, 1.0,
        np.where(data_int8 == 0,   0.0, 0.5)).astype(np.float32)
    return p


def _world_bounds(maps: dict, resolution: float):
    min_x = min_y =  float("inf")
    max_x = max_y = float("-inf")
    for m in maps.values():
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        min_x = min(min_x, ox)
        min_y = min(min_y, oy)
        max_x = max(max_x, ox + m.info.width  * resolution)
        max_y = max(max_y, oy + m.info.height * resolution)
    return min_x, min_y, max_x, max_y


def _map_to_full_grid(m, min_x, min_y, width, height, resolution):
    """Paste robot local map into a full-world-sized grid (float probs)."""
    grid = np.full((height, width), 0.5, dtype=np.float32)  # unknown = 0.5
    ox    = m.info.origin.position.x
    oy    = m.info.origin.position.y
    x_off = int(round((ox - min_x) / resolution))
    y_off = int(round((oy - min_y) / resolution))
    data  = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    probs = _grid_to_prob(data)

    r_end = min(y_off + m.info.height, height)
    c_end = min(x_off + m.info.width,  width)
    r_src_end = r_end - y_off
    c_src_end = c_end - x_off

    if r_end > y_off and c_end > x_off and y_off >= 0 and x_off >= 0:
        grid[y_off:r_end, x_off:c_end] = probs[:r_src_end, :c_src_end]
    return grid


def _wall_sharpness_fast(grid: np.ndarray) -> float:
    """Fast wall-sharpness estimate: mean run-length of occupied cells."""
    occupied = (grid == 1.0)
    # Horizontal runs
    padded = np.pad(occupied, ((0, 0), (1, 1)), constant_values=False)
    starts = ~padded[:, :-1] & padded[:, 1:]
    ends   = padded[:, :-1]  & ~padded[:, 1:]
    run_lengths = np.where(ends)[1] - np.where(starts)[1]
    return float(run_lengths.mean()) if len(run_lengths) > 0 else 1.0


# ─────────────────────────────────────────────────────────────────────────────
#  Feature extraction
# ─────────────────────────────────────────────────────────────────────────────

def _compute_node_features(grids: dict, names: list,
                            point_counts: dict = None) -> np.ndarray:
    """
    Return (n_robots, FEAT_DIM) node feature matrix.
    grids: name → (H, W) float32 probability grid (full world size)
    """
    n = len(names)
    F = np.zeros((n, FEAT_DIM), dtype=np.float32)
    max_pts = max(point_counts.values()) if point_counts else 1
    for i, name in enumerate(names):
        g = grids[name]
        known_mask = g != 0.5
        n_known = known_mask.sum()
        if n_known == 0:
            F[i] = [0, 0, 1, 0, 0]
            continue
        occ_r  = float((g == 1.0).sum()) / n_known
        free_r = float((g == 0.0).sum()) / n_known
        unk_r  = 1.0 - occ_r - free_r
        density = (point_counts[name] / max_pts) if point_counts else 0.5
        sharp   = _wall_sharpness_fast(g) / 10.0   # normalise; cap at ~10 cells/run
        F[i] = [occ_r, free_r, unk_r, float(density), min(sharp, 1.0)]
    return F


def _compute_edge_features(grids: dict, names: list) -> dict:
    """
    Return dict (i, j) → (EDGE_DIM,) edge feature vector for overlapping pairs.
    """
    n = len(names)
    edges = {}
    for i in range(n):
        for j in range(i + 1, n):
            gi = grids[names[i]]
            gj = grids[names[j]]
            known_i = gi != 0.5
            known_j = gj != 0.5
            both    = known_i & known_j
            either  = known_i | known_j
            n_both  = int(both.sum())
            n_either = int(either.sum())
            if n_both < MIN_OVERLAP_CELLS:
                continue
            overlap_ratio   = n_both / max(n_either, 1)
            agree_mask = both & (np.abs(gi - gj) < 0.5)
            agreement_ratio = float(agree_mask.sum()) / n_both
            efeat = np.array([overlap_ratio, agreement_ratio], dtype=np.float32)
            edges[(i, j)] = efeat
            edges[(j, i)] = efeat   # symmetric
    return edges


# ─────────────────────────────────────────────────────────────────────────────
#  GNN forward pass
# ─────────────────────────────────────────────────────────────────────────────

def _gnn_forward(F: np.ndarray, edges: dict, weights: dict) -> np.ndarray:
    """
    2-layer GAT-style message passing.
    Returns (n_robots,) confidence weights, normalised to sum to 1.
    """
    n = F.shape[0]
    H = F.copy()   # (n, FEAT_DIM)

    for layer in range(2):
        if layer == 0:
            W_self  = weights["W_self_0"]    # (HIDDEN_DIM, FEAT_DIM)
            W_neigh = weights["W_neigh_0"]   # (HIDDEN_DIM, FEAT_DIM)
            att_vec = weights["att_0"]        # (ATT_DIM,)
        else:
            W_self  = weights["W_self_1"]    # (HIDDEN_DIM, HIDDEN_DIM)
            W_neigh = weights["W_neigh_1"]   # (HIDDEN_DIM, HIDDEN_DIM)
            att_vec = weights["att_1"]        # (HIDDEN_DIM*2+EDGE_DIM,)

        H_new = np.zeros((n, HIDDEN_DIM), dtype=np.float32)
        for i in range(n):
            self_msg = W_self @ H[i]    # (HIDDEN_DIM,)

            # Gather neighbour messages with attention
            neigh_indices = [j for (ii, j) in edges if ii == i]
            if len(neigh_indices) == 0:
                H_new[i] = _relu(self_msg)
                continue

            att_scores = []
            neigh_msgs = []
            for j in neigh_indices:
                e_ij = edges[(i, j)]
                neigh_msg = W_neigh @ H[j]   # (HIDDEN_DIM,)
                att_in    = np.concatenate([self_msg, neigh_msg, e_ij])
                score     = _leaky_relu(att_vec @ att_in)
                att_scores.append(score)
                neigh_msgs.append(neigh_msg)

            att_weights = _softmax(np.array(att_scores, dtype=np.float32))
            agg = sum(w * m for w, m in zip(att_weights, neigh_msgs))
            H_new[i] = _relu(self_msg + agg)

        H = H_new

    # Output head → per-robot confidence
    W_out = weights["W_out"]   # (1, HIDDEN_DIM)
    confidence = _sigmoid((W_out @ H.T).flatten())   # (n,)

    # Normalise
    total = confidence.sum()
    if total < 1e-8:
        return np.ones(n, dtype=np.float32) / n
    return (confidence / total).astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
#  Main GNN class
# ─────────────────────────────────────────────────────────────────────────────

class OccupancyGridGNN:
    """
    2-layer Graph Attention Network for multi-robot occupancy grid fusion.

    Weights are initialised analytically and can be refined online via
    self-supervised consistency updates (call update_weights()).
    """

    def __init__(self, n_robots: int = 3, seed: int = 42,
                 learning_rate: float = 0.01):
        self.n_robots = n_robots
        self.lr       = learning_rate
        self.weights  = _init_weights(n_robots, seed)
        self._update_count = 0

    # ── Inference ─────────────────────────────────────────────────────────────

    def fuse(self, maps: dict, resolution: float,
             point_counts: dict = None) -> "OccupancyGrid | None":
        """
        Fuse maps from multiple robots using GNN-derived per-robot weights.

        maps:         {robot_name → nav_msgs/OccupancyGrid}
        resolution:   map cell size in metres
        point_counts: {robot_name → int}  optional, for density feature
        Returns:      nav_msgs/OccupancyGrid or None
        """
        if not maps:
            return None

        names = sorted(maps.keys())
        n     = len(names)

        # ── Build full-world probability grids ──────────────────────────────
        min_x, min_y, max_x, max_y = _world_bounds(maps, resolution)
        W = int(np.ceil((max_x - min_x) / resolution))
        H = int(np.ceil((max_y - min_y) / resolution))
        if W <= 0 or H <= 0:
            return None

        full_grids = {
            name: _map_to_full_grid(maps[name], min_x, min_y, W, H, resolution)
            for name in names
        }

        # ── GNN features ────────────────────────────────────────────────────
        pc = point_counts or {name: 1 for name in names}
        F     = _compute_node_features(full_grids, names, pc)   # (n, FEAT_DIM)
        edges = _compute_edge_features(full_grids, names)

        # ── Forward pass → per-robot weights ────────────────────────────────
        robot_weights = _gnn_forward(F, edges, self.weights)    # (n,)

        # ── Cell-level weighted fusion ───────────────────────────────────────
        fused_prob = np.zeros((H, W), dtype=np.float32)
        total_w    = np.zeros((H, W), dtype=np.float32)

        for i, name in enumerate(names):
            g = full_grids[name]
            w = robot_weights[i]
            # Only contribute where the robot has an observation (not unknown)
            known = (g != 0.5).astype(np.float32)
            fused_prob += w * g * known
            total_w    += w * known

        # Where at least one robot has observed: normalise
        # Where nobody observed:                 stays unknown
        observed = total_w > 1e-8
        fused_prob[observed]  /= total_w[observed]
        fused_prob[~observed]  = 0.5   # unknown

        # ── Threshold → int8 ────────────────────────────────────────────────
        merged = np.full((H, W), -1, dtype=np.int8)
        merged[fused_prob >  P_OCC_THRESH]  = 100
        merged[fused_prob <  P_FREE_THRESH] = 0

        # ── Build ROS message ────────────────────────────────────────────────
        out = OccupancyGrid()
        out.header.frame_id            = "world"
        out.info.resolution            = resolution
        out.info.width                 = W
        out.info.height                = H
        out.info.origin.position.x    = float(min_x)
        out.info.origin.position.y    = float(min_y)
        out.info.origin.orientation.w = 1.0
        out.data = merged.flatten().tolist()
        return out

    # ── Online self-supervised weight update ─────────────────────────────────

    def update_weights(self, maps: dict, resolution: float,
                       point_counts: dict = None):
        """
        One step of self-supervised consistency update.

        Loss = mean squared disagreement in overlap zones after applying
               GNN weights, compared to a simple average baseline.
               Gradients estimated by finite differences (clean, no autograd needed).
        """
        names = sorted(maps.keys())
        n     = len(names)
        min_x, min_y, max_x, max_y = _world_bounds(maps, resolution)
        W = int(np.ceil((max_x - min_x) / resolution))
        H = int(np.ceil((max_y - min_y) / resolution))
        if W <= 0 or H <= 0:
            return

        full_grids = {
            name: _map_to_full_grid(maps[name], min_x, min_y, W, H, resolution)
            for name in names
        }
        edges = _compute_edge_features(full_grids, names)

        if not edges:
            return   # no overlap → no useful gradient signal

        pc = point_counts or {name: 1 for name in names}
        F  = _compute_node_features(full_grids, names, pc)

        # Reference: simple average in overlap zones
        overlap_cells = None
        for (i, j) in [(e[0], e[1]) for e in edges if e[0] < e[1]]:
            gi = full_grids[names[i]]
            gj = full_grids[names[j]]
            mask = (gi != 0.5) & (gj != 0.5)
            overlap_cells = mask if overlap_cells is None else (overlap_cells | mask)

        if overlap_cells is None or overlap_cells.sum() < MIN_OVERLAP_CELLS:
            return

        def _consistency_loss(weights_dict):
            """Disagreement loss in overlap zones."""
            rw = _gnn_forward(F, edges, weights_dict)
            fused = np.zeros((H, W), dtype=np.float32)
            tw    = np.zeros((H, W), dtype=np.float32)
            for i, name in enumerate(names):
                g = full_grids[name]
                known = (g != 0.5).astype(np.float32)
                fused += rw[i] * g * known
                tw    += rw[i] * known
            obs = tw > 1e-8
            fused[obs]  /= tw[obs]
            fused[~obs]  = 0.5
            # Agreement between fused and each robot's observations
            loss = 0.0
            for name in names:
                g     = full_grids[name]
                known = (g != 0.5) & overlap_cells
                if known.sum() == 0:
                    continue
                diff   = fused[known] - g[known]
                loss  += float(np.mean(diff ** 2))
            return loss / max(n, 1)

        eps     = 1e-3
        base_loss = _consistency_loss(self.weights)

        # Finite-difference gradient for W_out only (cheapest, highest impact)
        W_out = self.weights["W_out"]
        grad  = np.zeros_like(W_out)
        for idx in np.ndindex(*W_out.shape):
            W_out[idx] += eps
            grad[idx]   = (_consistency_loss(self.weights) - base_loss) / eps
            W_out[idx] -= eps

        self.weights["W_out"] = W_out - self.lr * grad
        self._update_count += 1


# ─────────────────────────────────────────────────────────────────────────────
#  Module-level helpers (called from central_server_unified.py)
# ─────────────────────────────────────────────────────────────────────────────

_DEFAULT_GNN: OccupancyGridGNN | None = None


def load_gnn_model(n_robots: int = 3) -> OccupancyGridGNN:
    """Load (or create) the singleton GNN model."""
    global _DEFAULT_GNN
    if _DEFAULT_GNN is None:
        _DEFAULT_GNN = OccupancyGridGNN(n_robots=n_robots)
    return _DEFAULT_GNN


def fuse_with_gnn(model: OccupancyGridGNN, maps: dict, resolution: float,
                  point_counts: dict = None) -> "OccupancyGrid | None":
    """Top-level fusion entry point — delegates to model.fuse()."""
    return model.fuse(maps, resolution, point_counts=point_counts)


def update_gnn(model: OccupancyGridGNN, maps: dict, resolution: float,
               point_counts: dict = None):
    """Optional online weight update."""
    model.update_weights(maps, resolution, point_counts=point_counts)