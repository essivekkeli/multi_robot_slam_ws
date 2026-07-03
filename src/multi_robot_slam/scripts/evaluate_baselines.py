#!/usr/bin/env python3
"""
evaluate_baselines.py
=====================
Full evaluation of all policies for the multi-robot exploration thesis.

Methods:
  Random, Greedy Naive, Greedy No-Overlap,
  MAPPO deterministic, MAPPO stochastic,
  IPPO deterministic, IPPO stochastic

Evaluation metrics (all literature-backed):

  1. Coverage-over-time curve
     Primary exploration efficiency metric.
     Ref: Explore-Bench (Xu et al., ICRA 2022)
          Burgard et al. (2005) "Coordinated Multi-Robot Exploration"

  2. Accumulative Coverage Score (ACS)
     Integral of coverage over episode: ACS = sum(coverage_t for t in 1..T).
     Single scalar capturing both speed and final coverage.
     Ref: Yu et al. (2023) "Asynchronous Multi-Agent Reinforcement Learning
          for Efficient Real-Time Multi-Robot Cooperative Exploration"

  3. Steps to X% coverage
     Adaptation of trajectory-length-to-completion for fixed-horizon envs.
     Ref: ARiADNE (Cao et al., 2023) "A Reinforcement learning approach
          using Attention-based Deep Networks for Exploration"

  4. Exploration overlap ratio
     Fraction of cells explored by more than one robot / total explored.
     Measures multi-robot cooperation quality.
     Ref: Explore-Bench (Xu et al., ICRA 2022)
          Yu et al. (2023)

Usage:
  python3 evaluate_baselines.py \\
    --mappo_ckpt /ros2_ws/results_rl/mappo_v11/best.pt \\
    --ippo_ckpt  /ros2_ws/results_rl/ippo_v2/best.pt \\
    --n_episodes 50 \\
    --seed_offset 100 \\
    --out_dir /ros2_ws/results_rl/eval_v11_final
"""

import os, sys, math, argparse, json, csv
from pathlib import Path

import numpy as np
import torch
from torch.distributions import Categorical

sys.path.insert(0, str(Path(__file__).parent))
from multi_robot_env import (
    MultiRobotExplorationEnv, N_ROBOTS, MAX_FRONTIERS,
    COVERAGE_TARGET, MAX_STEPS
)
from train_mappo import ActorNet, obs_to_tensors


# ── Policies ──────────────────────────────────────────────────────────────────

def policy_random(env, rng):
    nf = len(env.frontiers)
    return [int(rng.integers(0, max(1, min(nf, MAX_FRONTIERS))))
            for _ in range(N_ROBOTS)]


def policy_greedy_naive(env):
    """All robots independently pick nearest frontier — no exclusion."""
    acts = []
    for i in range(N_ROBOTS):
        nf = len(env.frontiers)
        if nf == 0:
            acts.append(MAX_FRONTIERS)
            continue
        rx, ry = env.positions[i]
        dists = [math.hypot(fx - rx, fy - ry)
                 for fx, fy in env.frontiers[:MAX_FRONTIERS]]
        acts.append(int(np.argmin(dists)))
    return acts


def policy_greedy_no_overlap(env):
    """
    Exclusive assignment: assign robots to frontiers greedily,
    removing each assigned frontier from the pool.
    Ref: Burgard et al. (2005) coordination without learned policy.
    """
    available = list(range(min(len(env.frontiers), MAX_FRONTIERS)))
    acts = [MAX_FRONTIERS] * N_ROBOTS

    robot_order = sorted(range(N_ROBOTS), key=lambda i: min(
        (math.hypot(env.frontiers[k][0] - env.positions[i][0],
                    env.frontiers[k][1] - env.positions[i][1])
         for k in available), default=float('inf')
    ))

    for i in robot_order:
        if not available:
            break
        rx, ry = env.positions[i]
        dists = [(math.hypot(env.frontiers[k][0] - rx,
                             env.frontiers[k][1] - ry), k)
                 for k in available]
        _, best_k = min(dists)
        acts[i] = best_k
        available.remove(best_k)

    return acts


def policy_rl(actor, obs, stochastic=False):
    with torch.no_grad():
        maps, extras = obs_to_tensors(obs, torch.device('cpu'))
        logits = actor(maps, extras)
        nf = obs[0]['n_frontiers']
        if nf < MAX_FRONTIERS:
            logits[:, nf:MAX_FRONTIERS] = -1e9
        if stochastic:
            return Categorical(logits=logits).sample().tolist()
        else:
            return logits.argmax(dim=-1).tolist()


# ── Episode runner ────────────────────────────────────────────────────────────

def run_episode(env, policy_fn):
    """
    Run one full episode.
    Returns dict of per-episode metrics + per-step coverage curve.
    """
    obs, _ = env.reset()
    done = trunc = False
    step = 0

    cov_curve = []           # coverage at each step (for ACS + plotting)
    visited = [set() for _ in range(N_ROBOTS)]
    steps_to_30 = -1
    steps_to_40 = -1
    steps_to_50 = -1
    stall_step  = None
    stall_run   = 0

    while not (done or trunc):
        actions = policy_fn(obs)
        obs, _, done, trunc, info = env.step(actions)
        step += 1
        cov = info['coverage']
        cov_curve.append(cov)

        # track robot positions for overlap ratio
        for i in range(N_ROBOTS):
            rx, ry = env.positions[i]
            cell = (int((rx + 10) / 0.05), int((ry + 10) / 0.05))
            visited[i].add(cell)

        # steps to coverage milestones (ARiADNE-style)
        if steps_to_30 < 0 and cov >= 0.30:
            steps_to_30 = step
        if steps_to_40 < 0 and cov >= 0.40:
            steps_to_40 = step
        if steps_to_50 < 0 and cov >= 0.50:
            steps_to_50 = step

        # stall: coverage flat for 10+ consecutive steps
        if len(cov_curve) >= 2 and cov <= cov_curve[-2] + 0.001:
            stall_run += 1
            if stall_run >= 10 and stall_step is None:
                stall_step = step - 10
        else:
            stall_run = 0

    # Accumulative Coverage Score (Yu et al. 2023)
    # ACS = integral of coverage over time = sum of per-step coverage
    acs = float(np.sum(cov_curve + [cov_curve[-1]] * (MAX_STEPS - len(cov_curve))))

    # overlap ratio (Explore-Bench, Xu et al. 2022)
    all_visited = set()
    overlap_cells = 0
    for i in range(N_ROBOTS):
        for cell in visited[i]:
            if cell in all_visited:
                overlap_cells += 1
            all_visited.add(cell)
    total_visits = sum(len(v) for v in visited)
    overlap_ratio = overlap_cells / total_visits if total_visits > 0 else 0.0

    return {
        'coverage':            info['coverage'],
        'steps':               step,
        'acs':                 acs,
        'steps_to_30':         steps_to_30,
        'steps_to_40':         steps_to_40,
        'steps_to_50':         steps_to_50,
        'overlap_ratio':       overlap_ratio,
        'stall_step':          stall_step if stall_step is not None else -1,
        'stalled':             int(stall_step is not None),
        'frontiers_remaining': info['n_frontiers'],
        'cov_curve':           cov_curve,
    }


# ── Aggregation ───────────────────────────────────────────────────────────────

def aggregate(results):
    skip_keys = {'cov_curve'}
    keys = [k for k in results[0].keys() if k not in skip_keys]
    agg = {}
    for k in keys:
        vals = [r[k] for r in results]
        agg[f'{k}_mean'] = float(np.mean(vals))
        agg[f'{k}_std']  = float(np.std(vals))
        agg[f'{k}_min']  = float(np.min(vals))
        agg[f'{k}_max']  = float(np.max(vals))
    return agg


def compute_mean_curve(results, max_steps):
    """Compute mean coverage curve across episodes, padding shorter ones."""
    curves = []
    for r in results:
        curve = r['cov_curve']
        if len(curve) < max_steps:
            curve = curve + [curve[-1]] * (max_steps - len(curve))
        curves.append(curve[:max_steps])
    arr = np.array(curves)
    return arr.mean(axis=0), arr.std(axis=0)


# ── Run one method ────────────────────────────────────────────────────────────

def run_method(name, make_policy_fn, n_episodes, seed_offset):
    print(f'\n{"="*58}')
    print(f'  {name}')
    print(f'{"="*58}')

    results = []
    for ep in range(n_episodes):
        seed = seed_offset + ep
        env  = MultiRobotExplorationEnv(seed=seed)
        policy_fn = make_policy_fn(env)
        result = run_episode(env, policy_fn)
        results.append(result)

        if ep < 5 or ep % 10 == 9:
            print(f'  ep {ep+1:3d} seed={seed}: '
                  f'cov={result["coverage"]*100:.1f}%  '
                  f'ACS={result["acs"]:.0f}  '
                  f'overlap={result["overlap_ratio"]*100:.1f}%  '
                  f'stall@{result["stall_step"]}')

    agg = aggregate(results)
    s30 = [r['steps_to_30'] for r in results if r['steps_to_30'] > 0]
    s40 = [r['steps_to_40'] for r in results if r['steps_to_40'] > 0]
    s50 = [r['steps_to_50'] for r in results if r['steps_to_50'] > 0]

    print(f'\n  Mean coverage : {agg["coverage_mean"]*100:.1f}% ± {agg["coverage_std"]*100:.1f}%')
    print(f'  ACS (↑ better): {agg["acs_mean"]:.1f} ± {agg["acs_std"]:.1f}')
    print(f'  Overlap ratio : {agg["overlap_ratio_mean"]*100:.1f}% ± {agg["overlap_ratio_std"]*100:.1f}%')
    print(f'  Steps to 30%  : mean={np.mean(s30):.0f} ({len(s30)}/{n_episodes} reached)' if s30 else
          f'  Steps to 30%  : never reached')
    print(f'  Steps to 40%  : mean={np.mean(s40):.0f} ({len(s40)}/{n_episodes} reached)' if s40 else
          f'  Steps to 40%  : never reached')
    print(f'  Steps to 50%  : mean={np.mean(s50):.0f} ({len(s50)}/{n_episodes} reached)' if s50 else
          f'  Steps to 50%  : never reached')
    print(f'  Stall episodes: {sum(r["stalled"] for r in results)}/{n_episodes}')

    return results, agg


# ── Summary table ─────────────────────────────────────────────────────────────

def print_summary_table(all_results):
    print(f'\n\n{"="*100}')
    print(f'  FINAL RESULTS SUMMARY')
    print(f'  Metrics: Explore-Bench (Xu et al. 2022), ACS (Yu et al. 2023), Steps-to-X% (Cao et al. 2023)')
    print(f'{"="*100}')
    print(f'  {"Method":<28} {"Cov Mean":>9} {"Std":>6} {"ACS":>8} {"Overlap":>9} '
          f'{"S@30%":>7} {"S@40%":>7} {"S@50%":>7} {"Stalls":>8}')
    print(f'  {"-"*94}')

    for name, (results, agg) in all_results.items():
        n = len(results)
        s30 = [r['steps_to_30'] for r in results if r['steps_to_30'] > 0]
        s40 = [r['steps_to_40'] for r in results if r['steps_to_40'] > 0]
        s50 = [r['steps_to_50'] for r in results if r['steps_to_50'] > 0]
        stalls = sum(r['stalled'] for r in results)

        s30_str = f'{np.mean(s30):6.0f}' if s30 else f'{"N/A":>6}'
        s40_str = f'{np.mean(s40):6.0f}' if s40 else f'{"N/A":>6}'
        s50_str = f'{np.mean(s50):6.0f}' if s50 else f'{"N/A":>6}'

        print(f'  {name:<28} '
              f'{agg["coverage_mean"]*100:>8.1f}% '
              f'{agg["coverage_std"]*100:>5.1f}% '
              f'{agg["acs_mean"]:>7.0f} '
              f'{agg["overlap_ratio_mean"]*100:>8.1f}% '
              f'{s30_str}  {s40_str}  {s50_str}  '
              f'{stalls:>5}/{n}')

    print(f'{"="*100}')


# ── Save ──────────────────────────────────────────────────────────────────────

def save_results(all_results, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Per-episode CSV
    csv_path = out_dir / 'results_per_episode.csv'
    fieldnames = ['method', 'episode', 'seed', 'coverage', 'acs',
                  'steps', 'steps_to_30', 'steps_to_40', 'steps_to_50',
                  'overlap_ratio', 'stall_step', 'stalled', 'frontiers_remaining']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for name, (results, _) in all_results.items():
            for ep, r in enumerate(results):
                row = {'method': name, 'episode': ep, 'seed': 100 + ep}
                row.update({k: r[k] for k in fieldnames if k in r})
                writer.writerow(row)

    # 2. Coverage curves CSV (for plotting)
    curve_path = out_dir / 'coverage_curves.csv'
    # precompute all curves
    all_curves = {}
    for name, (results, _) in all_results.items():
        mean_c, std_c = compute_mean_curve(results, MAX_STEPS)
        all_curves[name] = (mean_c, std_c)

    with open(curve_path, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ['step']
        for name in all_results:
            header.extend([f'{name}_mean', f'{name}_std'])
        writer.writerow(header)

        for t in range(MAX_STEPS):
            row = [t + 1]
            for name in all_results:
                mean_c, std_c = all_curves[name]
                row.extend([f'{mean_c[t]:.6f}', f'{std_c[t]:.6f}'])
            writer.writerow(row)

    # 3. Summary JSON
    summary = {}
    for name, (results, agg) in all_results.items():
        n = len(results)
        s30 = [r['steps_to_30'] for r in results if r['steps_to_30'] > 0]
        s40 = [r['steps_to_40'] for r in results if r['steps_to_40'] > 0]
        s50 = [r['steps_to_50'] for r in results if r['steps_to_50'] > 0]
        summary[name] = {
            'coverage_mean':    round(agg['coverage_mean'], 4),
            'coverage_std':     round(agg['coverage_std'],  4),
            'coverage_min':     round(agg['coverage_min'],  4),
            'coverage_max':     round(agg['coverage_max'],  4),
            'acs_mean':         round(agg['acs_mean'], 2),
            'acs_std':          round(agg['acs_std'],  2),
            'overlap_mean':     round(agg['overlap_ratio_mean'], 4),
            'overlap_std':      round(agg['overlap_ratio_std'],  4),
            'steps_to_30_mean': round(float(np.mean(s30)), 1) if s30 else None,
            'steps_to_30_n':    len(s30),
            'steps_to_40_mean': round(float(np.mean(s40)), 1) if s40 else None,
            'steps_to_40_n':    len(s40),
            'steps_to_50_mean': round(float(np.mean(s50)), 1) if s50 else None,
            'steps_to_50_n':    len(s50),
            'stall_count':      int(sum(r['stalled'] for r in results)),
        }

    json_path = out_dir / 'summary.json'
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)

    print(f'\nResults saved:')
    print(f'  {csv_path}')
    print(f'  {curve_path}')
    print(f'  {json_path}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mappo_ckpt',  type=str, required=True)
    parser.add_argument('--ippo_ckpt',   type=str, required=True)
    parser.add_argument('--n_episodes',  type=int, default=50)
    parser.add_argument('--seed_offset', type=int, default=100)
    parser.add_argument('--out_dir',     type=str,
                        default='/ros2_ws/results_rl/eval_v11_final')
    parser.add_argument('--skip_rl',     action='store_true')
    args = parser.parse_args()

    print(f'Episodes : {args.n_episodes}  seeds {args.seed_offset}–'
          f'{args.seed_offset + args.n_episodes - 1}')
    print(f'MAX_STEPS={MAX_STEPS}')
    print(f'\nMetrics framework:')
    print(f'  Coverage curve + ACS : Explore-Bench (Xu et al. 2022), Yu et al. (2023)')
    print(f'  Steps to X%          : ARiADNE (Cao et al. 2023)')
    print(f'  Overlap ratio        : Explore-Bench (Xu et al. 2022)')

    all_results = {}

    # Random
    _ep_counter = [0]
    def make_random_policy(env):
        rng = np.random.default_rng(args.seed_offset + _ep_counter[0])
        _ep_counter[0] += 1
        return lambda obs, e=env, r=rng: policy_random(e, r)

    all_results['Random'] = run_method(
        'Random', make_random_policy,
        args.n_episodes, args.seed_offset
    )

    # Greedy naive
    all_results['Greedy Naive'] = run_method(
        'Greedy Naive',
        lambda env: (lambda obs, e=env: policy_greedy_naive(e)),
        args.n_episodes, args.seed_offset
    )

    # Greedy no-overlap
    all_results['Greedy No-Overlap'] = run_method(
        'Greedy No-Overlap',
        lambda env: (lambda obs, e=env: policy_greedy_no_overlap(e)),
        args.n_episodes, args.seed_offset
    )

    if not args.skip_rl:
        for ckpt_path, algo_name in [
            (args.mappo_ckpt, 'MAPPO'),
            (args.ippo_ckpt,  'IPPO'),
        ]:
            if not os.path.exists(ckpt_path):
                print(f'\nWARNING: {ckpt_path} not found — skipping {algo_name}')
                continue

            ck = torch.load(ckpt_path, map_location='cpu', weights_only=False)
            actor = ActorNet()
            actor.load_state_dict(ck['actor'])
            actor.eval()
            print(f'\nLoaded {algo_name}: '
                  f'best_cov={ck.get("best_cov", 0):.4f}  '
                  f'steps={ck.get("global_step", 0):,}  '
                  f'ent_coef={ck.get("cfg", {}).get("ent_coef", "?")}')

            for stochastic in [False, True]:
                mode = 'stochastic' if stochastic else 'deterministic'
                name = f'{algo_name} {mode}'
                all_results[name] = run_method(
                    name,
                    lambda env, a=actor, s=stochastic: (
                        lambda obs, _a=a, _s=s: policy_rl(_a, obs, _s)),
                    args.n_episodes, args.seed_offset
                )

    print_summary_table(all_results)
    save_results(all_results, args.out_dir)


if __name__ == '__main__':
    main()