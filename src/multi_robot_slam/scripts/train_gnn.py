#train_gnn.py
#!/usr/bin/env python3
"""
train_gnn.py — Stage 3a Week 4: GNN map fusion (v4)

Key insight from v1-v3 failures:
  Predicting a full 64x64 binary occupancy map from 12 training samples
  is impossible — 566k parameters on 10 examples always collapses.

Correct formulation for limited data:
  1. Pre-compute OPTIMAL fusion weights per sample via least-squares.
     optimal_w = argmin ||w1*m1 + w2*m2 + w3*m3 - GT||^2  s.t. sum=1
  2. Train GNN to PREDICT those optimal weights (3 scalars per sample).
  3. Loss: MSELoss(predicted_weights, optimal_weights)  — simple regression.

This works with 12 samples because:
  - Output is 3 scalars, not 4096 binary pixels
  - Tiny model (23k parameters) avoids overfitting
  - GT supervision is strong (optimal weights are unique per sample)

Thesis interpretation:
  The GNN learns which robot's map is most reliable for a given
  scene configuration — reflected in which robot gets highest weight.
  Weight analysis answers: does the GNN detect robot quality differences?
"""

import argparse, os, json, glob
import numpy as np
from datetime import datetime
from scipy.optimize import minimize

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

GRID_SIZE  = 64
N_ROBOTS   = 3
ROBOTS     = ["robot1", "robot2", "robot3"]
EMBED_DIM  = 64    # smaller — 12 samples needs tiny model
GNN_LEVELS = 2
N_HEADS    = 2


# ── Spatial projection (world-frame alignment) ────────────────────────────────

def project_to_world_grid(grid_int8, map_meta, gt_meta, size=GRID_SIZE):
    """Place robot observations into correct world-frame grid cells."""
    wmin_x  = gt_meta["world_min_x"]
    wmin_y  = gt_meta["world_min_y"]
    world_w = gt_meta["world_max_x"] - wmin_x
    world_h = gt_meta["world_max_y"] - wmin_y
    out = np.full((size, size), 0.5, dtype=np.float32)
    ox, oy, res = map_meta["origin_x"], map_meta["origin_y"], map_meta["resolution"]
    H, W = grid_int8.shape
    cols, rows = np.arange(W), np.arange(H)
    cc, rr = np.meshgrid(cols, rows)
    wx = ox + (cc + 0.5) * res
    wy = oy + (rr + 0.5) * res
    gx = ((wx - wmin_x) / world_w * size).astype(int)
    gy = ((wy - wmin_y) / world_h * size).astype(int)
    valid = (gx >= 0) & (gx < size) & (gy >= 0) & (gy < size)
    norm = np.full_like(grid_int8, 0.5, dtype=np.float32)
    norm[grid_int8 == 0]   = 0.0
    norm[grid_int8 == 100] = 1.0
    out[gy[valid], gx[valid]] = norm[valid]
    return out

def resize_gt(gt_int8, size=GRID_SIZE):
    g = (gt_int8 == 100).astype(np.float32)
    t = torch.from_numpy(g).unsqueeze(0).unsqueeze(0)
    return F.interpolate(t, size=(size,size), mode="nearest").squeeze().numpy()


# ── Optimal weight computation ────────────────────────────────────────────────

def compute_optimal_weights(maps, gt):
    """
    Find weights w = [w1,w2,w3] (sum=1, wi>=0) that minimise
    ||w1*m1 + w2*m2 + w3*m3 - gt||^2 (Frobenius norm).

    This is a constrained least-squares problem solved analytically
    or via scipy minimize. Returns np.array shape (3,) float32.
    """
    # Flatten maps and GT to 1D
    M  = np.stack([m.flatten() for m in maps], axis=1)   # (N_cells, 3)
    gt_flat = gt.flatten()                                 # (N_cells,)

    # Closed-form: solve M^T M w = M^T gt  subject to sum(w)=1, w>=0
    def loss(w):
        return np.sum((M @ w - gt_flat)**2)

    def jac(w):
        return 2 * M.T @ (M @ w - gt_flat)

    constraints = {"type": "eq", "fun": lambda w: np.sum(w) - 1.0}
    bounds = [(0.0, 1.0)] * N_ROBOTS
    w0 = np.ones(N_ROBOTS) / N_ROBOTS

    result = minimize(loss, w0, jac=jac,
                      method="SLSQP",
                      bounds=bounds,
                      constraints=constraints,
                      options={"ftol": 1e-9, "maxiter": 200})
    w = np.clip(result.x, 0, 1).astype(np.float32)
    w /= w.sum()   # re-normalise after clip
    return w


# ── Dataset ───────────────────────────────────────────────────────────────────

class MapFusionDataset(Dataset):
    """
    Each item:
      X       — (N_ROBOTS, 1, 64, 64) projected world-frame maps
      w_opt   — (N_ROBOTS,) optimal fusion weights for this sample
    """
    def __init__(self, sample_dirs, gt_grid, gt_meta, augment=False):
        self.items   = []
        self.augment = augment
        gt_rs = resize_gt(gt_grid)

        print("  Pre-computing optimal weights...")
        for d in sample_dirs:
            meta = json.load(open(os.path.join(d, "meta.json")))
            maps = []
            for robot in ROBOTS:
                raw  = np.load(os.path.join(d, f"{robot}_map.npy"))
                proj = project_to_world_grid(raw, meta["map_info"][robot],
                                             gt_meta, GRID_SIZE)
                maps.append(proj)
            w_opt = compute_optimal_weights(maps, gt_rs)
            self.items.append((maps, w_opt))
            name = os.path.basename(d)
            print(f"    {name}: optimal_w = "
                  f"[{w_opt[0]:.3f}, {w_opt[1]:.3f}, {w_opt[2]:.3f}]")

    def __len__(self):
        return len(self.items)

    def __getitem__(self, idx):
        maps, w_opt = self.items[idx]
        X = np.stack(maps)[:, np.newaxis, :, :]   # (3,1,64,64)
        if self.augment:
            if np.random.rand() > 0.5:
                X = X[:,:,:,::-1].copy()
            if np.random.rand() > 0.5:
                X = X[:,:,::-1,:].copy()
        return (torch.from_numpy(X.copy()).float(),
                torch.from_numpy(w_opt).float())


# ── Tiny encoder (small model for small dataset) ──────────────────────────────

class MapEncoder(nn.Module):
    """
    Lightweight encoder: 64×64 → 64-dim.
    ~23k parameters total vs 566k in v1-v3.
    Small model is essential with only 12 training samples.
    """
    def __init__(self, embed_dim=EMBED_DIM):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 8, 3, padding=1), nn.ReLU(), nn.MaxPool2d(4),  # 16×16
            nn.Conv2d(8,16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(4),  # 4×4
            nn.Flatten(),
            nn.Linear(16*4*4, embed_dim), nn.ReLU(),
        )
    def forward(self, x):
        return self.net(x)


# ── Cross-attention message passing ───────────────────────────────────────────

class CrossAttentionMP(nn.Module):
    def __init__(self, embed_dim=EMBED_DIM, n_heads=N_HEADS):
        super().__init__()
        self.heads = nn.ModuleList([
            nn.Linear(embed_dim*2, 1) for _ in range(n_heads)])
        self.leaky = nn.LeakyReLU(0.2)

    def forward(self, H):
        B, N, D = H.shape
        H_new = torch.zeros_like(H)
        for i in range(N):
            nbrs = [j for j in range(N) if j != i]
            scores = []
            for j in nbrs:
                pair = torch.cat([H[:,j,:], H[:,i,:]], dim=-1)
                s = torch.stack([self.leaky(h(pair)) for h in self.heads]).mean(0)
                scores.append(s)
            scores   = torch.stack(scores, dim=1)
            weights  = torch.softmax(scores, dim=1)
            nbr_feat = torch.stack([H[:,j,:] for j in nbrs], dim=1)
            H_new[:,i,:] = (weights * nbr_feat).sum(dim=1)
        return H_new


# ── GNN model ─────────────────────────────────────────────────────────────────

class GNNMapFusion(nn.Module):
    """
    Encode → message passing → decode 3 fusion weights.
    Output: predicted optimal weights (softmax, sum=1).
    """
    def __init__(self):
        super().__init__()
        self.encoder = MapEncoder(EMBED_DIM)
        self.gnn     = nn.ModuleList([CrossAttentionMP() for _ in range(GNN_LEVELS)])
        self.decoder = nn.Sequential(
            nn.Linear(EMBED_DIM*2, 32), nn.ReLU(),
            nn.Linear(32, 1))

    def forward(self, X):
        B, N, C, H, W = X.shape
        H0 = self.encoder(X.view(B*N,C,H,W)).view(B,N,-1)
        Hc = H0
        for layer in self.gnn:
            Hc = layer(Hc)
        w       = self.decoder(torch.cat([H0,Hc],dim=-1)).squeeze(-1)
        weights = torch.softmax(w, dim=1)
        return weights


# ── Training ──────────────────────────────────────────────────────────────────

def train(args):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    gt_grid = np.load(args.gt)
    gt_meta = json.load(open(args.gt_meta))
    print(f"GT: {gt_grid.shape}")

    sample_dirs = sorted(glob.glob(os.path.join(args.samples, "sample_*")))
    valid_dirs  = []
    for d in sample_dirs:
        meta = json.load(open(os.path.join(d, "meta.json")))
        if "map_info" not in meta: continue
        if not all(r in meta["map_info"] for r in ROBOTS): continue
        if min(meta["occupied_cells"].values()) < 200: continue
        valid_dirs.append(d)

    print(f"Valid samples: {len(valid_dirs)}")
    if len(valid_dirs) < 4:
        print("Need at least 4 samples."); return

    n_val = max(1, int(len(valid_dirs) * 0.2))
    np.random.shuffle(valid_dirs)
    train_dirs = valid_dirs[n_val:]
    val_dirs   = valid_dirs[:n_val]
    print(f"Train: {len(train_dirs)}  Val: {len(val_dirs)}")

    print("\nBuilding training set:")
    train_ds = MapFusionDataset(train_dirs, gt_grid, gt_meta, augment=True)
    print("Building val set:")
    val_ds   = MapFusionDataset(val_dirs,   gt_grid, gt_meta, augment=False)

    train_dl = DataLoader(train_ds, batch_size=args.batch_size,
                          shuffle=True,  num_workers=0)
    val_dl   = DataLoader(val_ds,   batch_size=args.batch_size,
                          shuffle=False, num_workers=0)

    model    = GNNMapFusion().to(device)
    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"\nParameters: {n_params:,}")

    criterion = nn.MSELoss()
    optimiser = torch.optim.Adam(model.parameters(), lr=args.lr,
                                 weight_decay=1e-3)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimiser, patience=8, factor=0.5, min_lr=1e-6)

    os.makedirs(args.out, exist_ok=True)
    best_val = float("inf")
    history  = []

    print(f"\n{'Ep':<6}{'Train MSE':<14}{'Val MSE':<14}{'Pred weights (val mean)'}")
    print("-" * 70)

    for epoch in range(1, args.epochs+1):
        model.train()
        tloss = 0.0
        for X, w_opt in train_dl:
            X, w_opt = X.to(device), w_opt.to(device)
            optimiser.zero_grad()
            w_pred = model(X)
            loss   = criterion(w_pred, w_opt)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimiser.step()
            tloss += loss.item()
        tloss /= len(train_dl)

        model.eval()
        vloss  = 0.0
        all_w  = []
        all_wo = []
        with torch.no_grad():
            for X, w_opt in val_dl:
                X, w_opt = X.to(device), w_opt.to(device)
                w_pred  = model(X)
                vloss  += criterion(w_pred, w_opt).item()
                all_w.append(w_pred.cpu().numpy())
                all_wo.append(w_opt.cpu().numpy())
        vloss /= len(val_dl)
        scheduler.step(vloss)
        lr = optimiser.param_groups[0]["lr"]

        mw  = np.concatenate(all_w).mean(0)
        mwo = np.concatenate(all_wo).mean(0)
        w_str = f"pred=[{mw[0]:.2f},{mw[1]:.2f},{mw[2]:.2f}]  " \
                f"opt=[{mwo[0]:.2f},{mwo[1]:.2f},{mwo[2]:.2f}]"

        history.append({"epoch":epoch,"train_mse":tloss,"val_mse":vloss,
                        "pred_weights":mw.tolist(),"opt_weights":mwo.tolist()})
        print(f"{epoch:<6}{tloss:<14.6f}{vloss:<14.6f}{w_str}   lr={lr:.1e}")

        if vloss < best_val:
            best_val = vloss
            torch.save({"epoch":epoch,"model_state":model.state_dict(),
                        "val_mse":vloss,"args":vars(args)},
                       os.path.join(args.out,"gnn_fusion.pt"))
            print(f"       ✓ best model saved")

    json.dump(history, open(os.path.join(args.out,"training_history.json"),"w"),
              indent=2)

    print(f"\nDone. Best val_MSE={best_val:.6f}")
    print(f"Model: {os.path.join(args.out,'gnn_fusion.pt')}")

    # Final analysis
    print("\n" + "="*60)
    print("RESULT SUMMARY")
    print("="*60)
    last = history[-1]
    mw   = np.array(last["pred_weights"])
    mwo  = np.array(last["opt_weights"])
    print(f"\nOptimal weights (ground truth targets):")
    for i,r in enumerate(ROBOTS):
        bar = "█" * int(mwo[i]*30)
        print(f"  {r}: {mwo[i]:.3f}  {bar}")
    print(f"\nPredicted weights (GNN output):")
    for i,r in enumerate(ROBOTS):
        bar = "█" * int(mw[i]*30)
        print(f"  {r}: {mw[i]:.3f}  {bar}")
    print(f"\nMSE between predicted and optimal: {best_val:.6f}")
    print(f"(MSE=0.000 = perfect prediction, MSE=0.111 = random guess)")
    print(f"\nThesis interpretation:")
    dominant = ROBOTS[np.argmax(mwo)]
    print(f"  Optimal fusion gives highest weight to {dominant}")
    print(f"  This robot's observations were most consistent with GT")
    print(f"  GNN {'correctly identified' if np.argmax(mw)==np.argmax(mwo) else 'failed to identify'} "
          f"the dominant robot")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--samples",    default="/ros2_ws/results/gnn/samples")
    p.add_argument("--gt",         default="/ros2_ws/results/gnn/ground_truth.npy")
    p.add_argument("--gt_meta",    default="/ros2_ws/results/gnn/ground_truth_meta.json")
    p.add_argument("--out",        default="/ros2_ws/results/gnn")
    p.add_argument("--epochs",     type=int,   default=100)
    p.add_argument("--batch_size", type=int,   default=2)
    p.add_argument("--lr",         type=float, default=5e-4)
    p.add_argument("--seed",       type=int,   default=42)
    args = p.parse_args()
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    print("="*60)
    print("GNN Map Fusion — Training v4 (optimal weight regression)")
    print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)
    train(args)

if __name__ == "__main__":
    main()
