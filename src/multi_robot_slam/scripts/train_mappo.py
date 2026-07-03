"""
train_mappo.py
==============
IPPO / MAPPO training for multi-robot exploration coordination.

Algorithm
---------
IPPO  (--algo ippo): each robot uses local obs for both actor AND critic.
MAPPO (--algo mappo): actor uses local obs (deployable), critic uses
                      concatenated global state of ALL robots (CTDE).
Both use PPO with parameter sharing across robots (homogeneous agents).

Network
-------
  Actor  : CNN(64×64 map) → 128  +  MLP(self_pos, other_pos, frontiers)
         → Tanh(128) → logits(7)
  Critic : [MAPPO] CNN×N + MLP×N → Tanh(256) → scalar value
           [IPPO]  same as actor but separate head → scalar value

Training loop
-------------
  Collect ROLLOUT_STEPS steps across N_ENVS parallel envs.
  Compute GAE advantages.
  Run PPO_EPOCHS × MINIBATCH updates per rollout.
  Log to stdout + optionally to tensorboard.
  Save checkpoint every SAVE_INTERVAL updates.

Usage
-----
  python3 train_mappo.py --algo mappo --n_envs 4 --total_steps 20_000_000
  python3 train_mappo.py --algo ippo  --n_envs 4 --total_steps 20_000_000
  python3 train_mappo.py --algo mappo --resume runs/mappo/ckpt_0010.pt

Outputs
-------
  runs/{algo}/
    ckpt_{step:06d}.pt      policy + value weights, optimiser state, config
    best.pt                 best mean episode coverage so far
    log.csv                 step, mean_cov, mean_return, entropy, lr
"""

import os, sys, csv, time, math, copy, argparse
from pathlib import Path
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Categorical

# ── Import env (same directory or PYTHONPATH) ────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))
from multi_robot_env import (
    MultiRobotExplorationEnv, N_ROBOTS, MAX_FRONTIERS,
    LOCAL_CROP, COVERAGE_TARGET, MAX_STEPS
)

# ── Hyperparameters ───────────────────────────────────────────────────────────
CFG = {
    # PPO
    "lr":           3e-4,
    "gamma":        0.99,
    "gae_lambda":   0.95,
    "clip_eps":     0.2,
    "vf_coef":      0.5,
    "ent_coef":     0.01,
    "max_grad_norm":0.5,
    "ppo_epochs":   10,
    "n_minibatches":4,
    # collection
    "rollout_steps":256,     # steps per env per rollout (decision steps)
    "n_envs":       4,       # parallel environments
    "total_steps":  20_000_000,
    # schedule
    "lr_anneal":    True,    # linearly anneal lr to 0
    "ent_anneal":   True,    # anneal entropy bonus
    # value normalisation (Suggestion 1 from MAPPO paper)
    "value_norm":   True,
    # misc
    "save_interval":50,      # save checkpoint every N updates
    "log_interval": 10,
    "seed":         42,
}

ACTION_N   = MAX_FRONTIERS + 1   # 7
EXTRA_DIM  = 2 + (N_ROBOTS-1)*2 + MAX_FRONTIERS*2 + 1  # 19


# ── Networks ──────────────────────────────────────────────────────────────────
class MapEncoder(nn.Module):
    """64×64 occupancy crop → 128-dim. Shared between actor and critic."""
    def __init__(self, out_dim=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 16, 5, stride=2, padding=2),   # 64→32
            nn.ReLU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1),  # 32→16
            nn.ReLU(),
            nn.Conv2d(32, 32, 3, stride=2, padding=1),  # 16→8
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(32*8*8, out_dim),
            nn.ReLU(),
        )
    def forward(self, x):
        # x: (B, 64, 64)  int8 or float — normalise to [-1,1]
        return self.net(x.float().unsqueeze(1))  # add channel dim


class ActorNet(nn.Module):
    def __init__(self, obs_dim=EXTRA_DIM, hidden=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden), nn.Tanh(),
            nn.Linear(hidden, hidden),  nn.Tanh(),
            nn.Linear(hidden, ACTION_N),
        )
        nn.init.orthogonal_(self.net[-1].weight, gain=0.01)

    def forward(self, local_map, extra):
        return self.net(extra)   # map ignored — coordination signal is in extra


class CriticNet(nn.Module):
    def __init__(self, n_agents=N_ROBOTS, obs_dim=EXTRA_DIM, hidden=256):
        super().__init__()
        self.n_agents = n_agents
        self.net = nn.Sequential(
            nn.Linear(n_agents * obs_dim, hidden), nn.Tanh(),
            nn.Linear(hidden, hidden),              nn.Tanh(),
            nn.Linear(hidden, 1),
        )
        nn.init.orthogonal_(self.net[-1].weight, gain=1.0)

    def forward(self, maps, extras):
        B = extras.shape[0]
        return self.net(extras.reshape(B, -1))


class RunningMeanStd:
    """Online mean/variance for value normalisation."""
    def __init__(self, eps=1e-4):
        self.mean = np.float64(0.)
        self.var  = np.float64(1.)
        self.count = np.float64(eps)

    def update(self, x):
        x = np.asarray(x, dtype=np.float64).flatten()
        n = len(x)
        m = x.mean(); v = x.var()
        delta = m - self.mean
        self.mean  += delta * n / (self.count + n)
        self.var    = (self.var*self.count + v*n + delta**2 *
                       self.count*n/(self.count+n)) / (self.count+n)
        self.count += n

    def normalise(self, x):
        return (x - self.mean) / (np.sqrt(self.var) + 1e-8)


# ── Observation helper ────────────────────────────────────────────────────────
def obs_to_tensors(obs_list, device):
    """
    Convert list of per-robot obs dicts → tensors.
    Returns:
      maps:   (N_ROBOTS, 64, 64)  float32
      extras: (N_ROBOTS, EXTRA_DIM)  float32
    """
    maps, extras = [], []
    for o in obs_list:
        maps.append(torch.tensor(o["local_map"], dtype=torch.float32))
        e = np.concatenate([
            o["self_pos"],
            o["other_pos"].flatten(),
            o["frontiers"].flatten(),
            [o["n_frontiers"] / MAX_FRONTIERS],
        ]).astype(np.float32)
        extras.append(torch.tensor(e))
    return (torch.stack(maps).to(device),
            torch.stack(extras).to(device))


# ── Rollout buffer ────────────────────────────────────────────────────────────
class RolloutBuffer:
    """
    Stores experience for one rollout across all envs and all robots.
    Shapes: (rollout_steps, n_envs, N_ROBOTS, ...)
    """
    def __init__(self, rollout_steps, n_envs, device):
        T, E, N = rollout_steps, n_envs, N_ROBOTS
        self.maps    = torch.zeros(T, E, N, LOCAL_CROP, LOCAL_CROP)
        self.extras  = torch.zeros(T, E, N, EXTRA_DIM)
        self.actions = torch.zeros(T, E, N, dtype=torch.long)
        self.log_probs = torch.zeros(T, E, N)
        self.rewards = torch.zeros(T, E)
        self.dones   = torch.zeros(T, E)
        self.values  = torch.zeros(T, E)
        self.t = 0
        self.T = T
        self.device = device

    def add(self, maps, extras, actions, log_probs, reward, done, value):
        t = self.t
        self.maps[t]     = maps.cpu()
        self.extras[t]   = extras.cpu()
        self.actions[t]  = actions.cpu()
        self.log_probs[t]= log_probs.cpu()
        self.rewards[t]  = torch.tensor(reward, dtype=torch.float32)
        self.dones[t]    = torch.tensor(done,   dtype=torch.float32)
        self.values[t]   = value.cpu().squeeze(-1)
        self.t += 1

    def compute_gae(self, last_value, gamma, gae_lambda):
        """Compute advantages + returns in-place."""
        T, E = self.T, self.rewards.shape[1]
        adv  = torch.zeros(T, E)
        last_adv = torch.zeros(E)
        last_val = last_value.cpu().squeeze(-1)
        for t in reversed(range(T)):
            next_val  = last_val if t == T-1 else self.values[t+1]
            next_done = torch.zeros(E) if t == T-1 else self.dones[t+1]
            delta = (self.rewards[t]
                     + gamma * next_val * (1-self.dones[t])
                     - self.values[t])
            last_adv = delta + gamma * gae_lambda * (1-self.dones[t]) * last_adv
            adv[t] = last_adv
        self.returns = adv + self.values
        self.advantages = adv
        return adv

    def get_minibatches(self, n_minibatches, algo):
        """
        Flatten (T, E, N) → (T*E*N,) for actors,
                (T, E)   → (T*E,)   for critics.
        Yield shuffled minibatches.
        """
        T, E, N = self.T, self.rewards.shape[1], N_ROBOTS
        idx = torch.randperm(T*E)
        mb_size = T*E // n_minibatches

        # flatten time×env
        maps_   = self.maps.reshape(T*E, N, LOCAL_CROP, LOCAL_CROP)
        extras_ = self.extras.reshape(T*E, N, EXTRA_DIM)
        acts_   = self.actions.reshape(T*E, N)
        lps_    = self.log_probs.reshape(T*E, N)
        adv_    = self.advantages.reshape(T*E)
        ret_    = self.returns.reshape(T*E)
        val_    = self.values.reshape(T*E)

        for start in range(0, T*E, mb_size):
            i = idx[start:start+mb_size]
            yield {
                "maps":      maps_[i],
                "extras":    extras_[i],
                "actions":   acts_[i],
                "old_lps":   lps_[i],
                "advantages":adv_[i],
                "returns":   ret_[i],
                "old_values":val_[i],
            }


# ── Trainer ───────────────────────────────────────────────────────────────────
class MAPPOTrainer:
    def __init__(self, algo="mappo", cfg=None, run_dir=None, device="cpu"):
        self.algo   = algo
        self.cfg    = {**CFG, **(cfg or {})}
        self.device = torch.device(device)
        self.run_dir = Path(run_dir or f"runs/{algo}")
        self.run_dir.mkdir(parents=True, exist_ok=True)

        # Networks
        self.actor  = ActorNet().to(self.device)
        n_critic_agents = N_ROBOTS if algo == "mappo" else 1
        self.critic = CriticNet(n_agents=n_critic_agents).to(self.device)

        params = list(self.actor.parameters()) + list(self.critic.parameters())
        self.optim = torch.optim.Adam(params, lr=self.cfg["lr"], eps=1e-5)

        # Value normalisation
        self.val_rms = RunningMeanStd() if self.cfg["value_norm"] else None

        # Environments (N_ENVS independent instances)
        self.n_envs = self.cfg["n_envs"]
        self.envs   = [MultiRobotExplorationEnv(seed=self.cfg["seed"]+i)
                       for i in range(self.n_envs)]
        self.obs    = [None]*self.n_envs
        for e, env in enumerate(self.envs):
            self.obs[e], _ = env.reset()

        self.global_step  = 0
        self.update_count = 0
        self.best_cov     = 0.
        self.ep_returns   = deque(maxlen=50)
        self.ep_covs      = deque(maxlen=50)

        # CSV logger
        self.log_path = self.run_dir / "log.csv"
        with open(self.log_path, "w", newline="") as f:
            csv.writer(f).writerow(
                ["step","update","mean_cov","mean_return","entropy","value_loss","policy_loss","lr"])

    # ── Main training entry point ─────────────────────────────────────────────
    def train(self, total_steps=None):
        total_steps = total_steps or self.cfg["total_steps"]
        T = self.cfg["rollout_steps"]
        print(f"\n{'='*60}")
        print(f"  {self.algo.upper()} training")
        print(f"  Total steps: {total_steps:,}  |  Envs: {self.n_envs}")
        print(f"  Rollout: {T}  |  Epochs: {self.cfg['ppo_epochs']}")
        print(f"  Device: {self.device}")
        print(f"  Output: {self.run_dir}")
        print(f"{'='*60}\n")

        buf = RolloutBuffer(T, self.n_envs, self.device)
        t0  = time.perf_counter()

        while self.global_step < total_steps:
            frac = self.global_step / total_steps
            lr   = self.cfg["lr"] * (1-frac) if self.cfg["lr_anneal"] else self.cfg["lr"]
            ent  = self.cfg["ent_coef"] * (1-frac) if self.cfg["ent_anneal"] else self.cfg["ent_coef"]
            for pg in self.optim.param_groups: pg["lr"] = lr

            # ── Collect rollout ───────────────────────────────────────────────
            buf.t = 0
            self.actor.eval()
            self.critic.eval()
            with torch.no_grad():
                for _ in range(T):
                    # gather obs from all envs
                    all_maps   = []   # (E, N, 64, 64)
                    all_extras = []   # (E, N, 19)
                    for e in range(self.n_envs):
                        m, x = obs_to_tensors(self.obs[e], self.device)
                        all_maps.append(m); all_extras.append(x)
                    all_maps   = torch.stack(all_maps)    # (E,N,64,64)
                    all_extras = torch.stack(all_extras)  # (E,N,19)

                    # actor: per-robot logits
                    # reshape to (E*N, ...) for batch forward
                    EN = self.n_envs * N_ROBOTS
                    logits = self.actor(
                        all_maps.reshape(EN, LOCAL_CROP, LOCAL_CROP).to(self.device),
                        all_extras.reshape(EN, EXTRA_DIM).to(self.device),
                    ).reshape(self.n_envs, N_ROBOTS, ACTION_N)

                    # mask unavailable frontier actions
                    for e in range(self.n_envs):
                        nf = self.obs[e][0]["n_frontiers"]
                        if nf < MAX_FRONTIERS:
                            logits[e, :, nf:MAX_FRONTIERS] = -1e9

                    dist    = Categorical(logits=logits)
                    actions = dist.sample()          # (E, N)
                    lps     = dist.log_prob(actions) # (E, N)

                    # critic value
                    val = self._value(all_maps, all_extras)  # (E,1)

                    # step all envs
                    rewards = np.zeros(self.n_envs)
                    dones   = np.zeros(self.n_envs)
                    for e, env in enumerate(self.envs):
                        acts = actions[e].tolist()
                        next_obs, r, done, trunc, info = env.step(acts)
                        rewards[e] = r
                        dones[e]   = float(done or trunc)
                        if done or trunc:
                            self.ep_returns.append(r)
                            self.ep_covs.append(info["coverage"])
                            self.obs[e], _ = env.reset()
                        else:
                            self.obs[e] = next_obs

                    buf.add(all_maps, all_extras, actions, lps,
                            rewards, dones, val)
                    self.global_step += self.n_envs

                # bootstrap value for last step
                next_maps, next_extras = [], []
                for e in range(self.n_envs):
                    m, x = obs_to_tensors(self.obs[e], self.device)
                    next_maps.append(m); next_extras.append(x)
                next_maps   = torch.stack(next_maps)
                next_extras = torch.stack(next_extras)
                last_val = self._value(next_maps, next_extras)

            buf.compute_gae(last_val, self.cfg["gamma"], self.cfg["gae_lambda"])

            # ── PPO update ────────────────────────────────────────────────────
            self.actor.train(); self.critic.train()
            adv_flat = buf.advantages.reshape(-1).numpy()
            if self.val_rms:
                self.val_rms.update(buf.returns.reshape(-1).numpy())

            pl_list=[]; vl_list=[]; ent_list=[]
            for _ in range(self.cfg["ppo_epochs"]):
                for mb in buf.get_minibatches(self.cfg["n_minibatches"], self.algo):
                    B = mb["maps"].shape[0]
                    maps_b   = mb["maps"].to(self.device)    # (B, N, 64,64)
                    extras_b = mb["extras"].to(self.device)  # (B, N, 19)
                    acts_b   = mb["actions"].to(self.device) # (B, N)
                    old_lps  = mb["old_lps"].to(self.device) # (B, N)
                    adv_b    = mb["advantages"].to(self.device).unsqueeze(-1).expand(B, N_ROBOTS)
                    ret_b    = mb["returns"].to(self.device)
                    old_val_b= mb["old_values"].to(self.device)

                    # normalise advantages per minibatch
                    adv_b = (adv_b - adv_b.mean()) / (adv_b.std() + 1e-8)

                    # actor forward (B*N)
                    logits_b = self.actor(
                        maps_b.reshape(B*N_ROBOTS, LOCAL_CROP, LOCAL_CROP),
                        extras_b.reshape(B*N_ROBOTS, EXTRA_DIM),
                    ).reshape(B, N_ROBOTS, ACTION_N)
                    dist_b   = Categorical(logits=logits_b)
                    new_lps  = dist_b.log_prob(acts_b)   # (B,N)
                    entropy  = dist_b.entropy().mean()

                    # policy loss
                    ratio   = (new_lps - old_lps).exp()
                    pg_loss1 = -adv_b * ratio
                    pg_loss2 = -adv_b * ratio.clamp(1-self.cfg["clip_eps"],
                                                     1+self.cfg["clip_eps"])
                    policy_loss = torch.max(pg_loss1, pg_loss2).mean()

                    # value loss
                    val_b = self._value(maps_b, extras_b).squeeze(-1)  # (B,)
                    if self.val_rms:
                        ret_norm = torch.tensor(
                            self.val_rms.normalise(ret_b.cpu().numpy()),
                            dtype=torch.float32).to(self.device)
                    else:
                        ret_norm = ret_b

                    # clipped value loss
                    val_clipped = (old_val_b
                                   + (val_b - old_val_b).clamp(-self.cfg["clip_eps"],
                                                                 self.cfg["clip_eps"]))
                    vf_loss = torch.max(
                        F.mse_loss(val_b, ret_norm),
                        F.mse_loss(val_clipped, ret_norm)
                    )

                    loss = (policy_loss
                            + self.cfg["vf_coef"] * vf_loss
                            - ent * entropy)

                    self.optim.zero_grad()
                    loss.backward()
                    nn.utils.clip_grad_norm_(
                        list(self.actor.parameters())+list(self.critic.parameters()),
                        self.cfg["max_grad_norm"])
                    self.optim.step()

                    pl_list.append(policy_loss.item())
                    vl_list.append(vf_loss.item())
                    ent_list.append(entropy.item())

            self.update_count += 1

            # ── Logging ───────────────────────────────────────────────────────
            if self.update_count % self.cfg["log_interval"] == 0:
                mean_cov = np.mean(self.ep_covs) if self.ep_covs else 0.
                mean_ret = np.mean(self.ep_returns) if self.ep_returns else 0.
                mean_ent = np.mean(ent_list)
                mean_pl  = np.mean(pl_list)
                mean_vl  = np.mean(vl_list)
                fps = int(self.global_step / (time.perf_counter()-t0))
                print(f"[{self.global_step:>10,}] "
                      f"cov={mean_cov:.3f}  ret={mean_ret:+.0f}  "
                      f"ent={mean_ent:.3f}  pl={mean_pl:.4f}  "
                      f"vl={mean_vl:.4f}  lr={lr:.2e}  fps={fps}")
                with open(self.log_path, "a", newline="") as f:
                    csv.writer(f).writerow([
                        self.global_step, self.update_count,
                        f"{mean_cov:.4f}", f"{mean_ret:.2f}",
                        f"{mean_ent:.4f}", f"{mean_vl:.4f}",
                        f"{mean_pl:.4f}", f"{lr:.2e}"])

            # ── Checkpointing ─────────────────────────────────────────────────
            if self.update_count % self.cfg["save_interval"] == 0:
                self._save(f"ckpt_{self.update_count:06d}.pt")

            mean_cov = np.mean(self.ep_covs) if self.ep_covs else 0.
            if mean_cov > self.best_cov:
                self.best_cov = mean_cov
                self._save("best.pt")

        self._save("final.pt")
        print(f"\nTraining complete. Best coverage: {self.best_cov:.3f}")
        print(f"Checkpoints: {self.run_dir}")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _value(self, maps, extras):
        """Compute value estimate. Handles MAPPO (global) vs IPPO (local)."""
        if self.algo == "mappo":
            return self.critic(maps.to(self.device), extras.to(self.device))
        else:
            # IPPO: mean value across robots using critic with n_agents=1
            B = maps.shape[0]
            vals = []
            for i in range(N_ROBOTS):
                v = self.critic(
                    maps[:, i:i+1].to(self.device),
                    extras[:, i:i+1].to(self.device))
                vals.append(v)
            return torch.stack(vals, dim=1).mean(dim=1)  # (B,1)

    def _save(self, name):
        path = self.run_dir / name
        torch.save({
            "actor":       self.actor.state_dict(),
            "critic":      self.critic.state_dict(),
            "optim":       self.optim.state_dict(),
            "global_step": self.global_step,
            "update_count":self.update_count,
            "best_cov":    self.best_cov,
            "algo":        self.algo,
            "cfg":         self.cfg,
            "val_rms_mean":self.val_rms.mean if self.val_rms else None,
            "val_rms_var": self.val_rms.var  if self.val_rms else None,
        }, path)

    def load(self, path):
        ck = torch.load(path, map_location=self.device, weights_only=False)
        self.actor.load_state_dict(ck["actor"])
        self.critic.load_state_dict(ck["critic"])
        self.optim.load_state_dict(ck["optim"])
        self.global_step   = ck["global_step"]
        self.update_count  = ck["update_count"]
        self.best_cov      = ck["best_cov"]
        if self.val_rms and ck.get("val_rms_mean") is not None:
            self.val_rms.mean = ck["val_rms_mean"]
            self.val_rms.var  = ck["val_rms_var"]
        print(f"Resumed from {path} at step {self.global_step:,}")

    def save_actor_only(self, path):
        """Save just the actor weights for deployment in the Gazebo node."""
        torch.save({
            "actor": self.actor.state_dict(),
            "algo":  self.algo,
            "cfg":   self.cfg,
        }, path)
        print(f"Actor saved to {path}")


# ── Evaluation (greedy rollout, no training) ──────────────────────────────────
def evaluate(actor_path, n_episodes=10, seed=999):
    """
    Load a saved actor and run greedy (argmax) evaluation episodes.
    Prints per-episode coverage and mean.
    """
    ck  = torch.load(actor_path, map_location="cpu", weights_only=False)
    actor = ActorNet()
    actor.load_state_dict(ck["actor"])
    actor.eval()

    env = MultiRobotExplorationEnv(seed=seed)
    covs = []
    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = trunc = False
        while not (done or trunc):
            with torch.no_grad():
                maps, extras = obs_to_tensors(obs, torch.device("cpu"))
                logits = actor(maps, extras)  # (N, ACTION_N)
                nf = obs[0]["n_frontiers"]
                if nf < MAX_FRONTIERS:
                    logits[:, nf:MAX_FRONTIERS] = -1e9
                actions = logits.argmax(dim=-1).tolist()
            obs, _, done, trunc, info = env.step(actions)
        covs.append(info["coverage"])
        print(f"  ep {ep+1:2d}: coverage={info['coverage']*100:.1f}%  "
              f"steps={info['step']}  {'✓' if done else '✗'}")
    print(f"  Mean coverage: {np.mean(covs)*100:.1f}%")
    return covs


# ── CLI ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--algo",        default="mappo", choices=["mappo","ippo"])
    parser.add_argument("--n_envs",      type=int,   default=4)
    parser.add_argument("--total_steps", type=int,   default=20_000_000)
    parser.add_argument("--lr",          type=float, default=3e-4)
    parser.add_argument("--run_dir",     type=str,   default=None)
    parser.add_argument("--resume",      type=str,   default=None)
    parser.add_argument("--eval",        type=str,   default=None,
                        help="path to actor checkpoint for evaluation only")
    parser.add_argument("--device",      default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--seed",        type=int,   default=42)
    args = parser.parse_args()

    if args.eval:
        print(f"Evaluating {args.eval}")
        evaluate(args.eval)
        sys.exit(0)

    cfg_override = {
        "n_envs":       args.n_envs,
        "total_steps":  args.total_steps,
        "lr":           args.lr,
        "seed":         args.seed,
    }
    run_dir = args.run_dir or f"runs/{args.algo}"
    trainer = MAPPOTrainer(algo=args.algo, cfg=cfg_override,
                           run_dir=run_dir, device=args.device)
    if args.resume:
        trainer.load(args.resume)

    # Quick sanity check before committing to a long run
    print("Running 2-step sanity check...")
    obs, _ = trainer.envs[0].reset()
    maps, extras = obs_to_tensors(obs, trainer.device)
    with torch.no_grad():
        logits = trainer.actor(maps, extras)
        print(f"  logits shape: {logits.shape}  ✓")
        val = trainer._value(
            maps.unsqueeze(0).expand(1,-1,-1,-1),
            extras.unsqueeze(0).expand(1,-1,-1))
        print(f"  value shape:  {val.shape}  ✓")
    print("Sanity check passed. Starting training...\n")

    trainer.train(total_steps=args.total_steps)