#!/usr/bin/env python3
"""
Coverage curve plots for thesis.
Run on ROS machine:
  python3 plot_coverage_curves.py

Outputs (same directory as script, or set OUTPUT_DIR):
  coverage_curves_v11_v12.pdf   -- side-by-side comparison (main thesis figure)
  coverage_curves_v11_only.pdf  -- v11 standalone
  coverage_curves_v12_only.pdf  -- v12 standalone
  (+ .png versions of each)
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os

# ── Paths ──────────────────────────────────────────────────────────────────
V11_CSV    = '/ros2_ws/results_rl/eval_v11_corrected/coverage_curves.csv'
V12_CSV    = '/ros2_ws/results_rl/eval_v12_corrected/coverage_curves.csv'
OUTPUT_DIR = '/ros2_ws/results_rl/plots/'
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Load ───────────────────────────────────────────────────────────────────
v11 = pd.read_csv(V11_CSV)
v12 = pd.read_csv(V12_CSV)
print(f"v11: {len(v11)} steps,  columns: {list(v11.columns)}")
print(f"v12: {len(v12)} steps,  columns: {list(v12.columns)}")

# ── Style ──────────────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':       'serif',
    'font.size':          10,
    'axes.labelsize':     11,
    'axes.titlesize':     11,
    'legend.fontsize':     9,
    'xtick.labelsize':     9,
    'ytick.labelsize':     9,
    'axes.spines.top':   False,
    'axes.spines.right': False,
    'figure.dpi':         150,
})

# Colorblind-safe palette (Wong 2011).
# IPPO deterministic excluded: corrupted checkpoint artifact (global_step=1024 in best.pt)
METHODS = {
    'MAPPO stochastic':    {'color': '#0072B2', 'ls': '-',   'lw': 2.2, 'zorder': 5},
    'IPPO stochastic':     {'color': '#009E73', 'ls': '-',   'lw': 1.8, 'zorder': 4},
    'Random':              {'color': '#555555', 'ls': '--',  'lw': 1.8, 'zorder': 3},
    'MAPPO deterministic': {'color': '#56B4E9', 'ls': ':',   'lw': 1.6, 'zorder': 2},
    'Greedy No-Overlap':   {'color': '#E69F00', 'ls': '-.',  'lw': 1.6, 'zorder': 2},
    'Greedy Naive':        {'color': '#CC79A7', 'ls': '-.',  'lw': 1.4, 'zorder': 1},
}

def plot_env(ax, df, title, show_legend=False, annotate_mappo_lead=False):
    steps = df['step'].values
    plotted = []
    for name, style in METHODS.items():
        mean_col = f'{name}_mean'
        std_col  = f'{name}_std'
        if mean_col not in df.columns:
            continue
        mean = df[mean_col].values * 100
        std  = df[std_col].values  * 100
        line, = ax.plot(steps, mean,
                        color=style['color'], ls=style['ls'],
                        lw=style['lw'], zorder=style['zorder'],
                        label=name)
        ax.fill_between(steps, mean - std, mean + std,
                        color=style['color'], alpha=0.10,
                        zorder=style['zorder'] - 1)
        plotted.append(name)

    # 50% reference line
    ax.axhline(50, color='#aaaaaa', lw=0.8, ls=':', zorder=0)
    ax.text(steps[-1] * 0.98, 50.9, '50%',
            color='#aaaaaa', fontsize=8, ha='right', va='bottom')

    # Annotate MAPPO stochastic lead over random in v11
    if annotate_mappo_lead and 'MAPPO stochastic_mean' in df.columns:
        m_mean = df['MAPPO stochastic_mean'].values * 100
        r_mean = df['Random_mean'].values            * 100
        gap    = m_mean - r_mean
        idx    = np.where(gap > 3.0)[0]
        if len(idx):
            s   = steps[idx[0]]
            yv  = m_mean[idx[0]]
            ax.annotate('MAPPO\nlead', xy=(s, yv),
                        xytext=(s + 15, yv + 6),
                        fontsize=7.5, color='#0072B2',
                        arrowprops=dict(arrowstyle='->', color='#0072B2', lw=0.8))

    ax.set_xlim(1, steps[-1])
    ax.set_ylim(0, 70)
    ax.set_xlabel('Step')
    ax.set_ylabel('Coverage (%)')
    ax.set_title(title, pad=8)

    if show_legend:
        ax.legend(loc='lower right', frameon=True,
                  framealpha=0.92, edgecolor='#cccccc', ncol=1)

# ─────────────────────────────────────────────────────────────────────────────
# Figure 1: side-by-side (main thesis figure)
# ─────────────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 4.4), sharey=True)
fig.subplots_adjust(wspace=0.06, left=0.07, right=0.97, top=0.84, bottom=0.13)

plot_env(axes[0], v11,
         '(a) v11 — per-step goal reassignment',
         show_legend=True, annotate_mappo_lead=True)
plot_env(axes[1], v12,
         '(b) v12 — persistent goals (15-step timeout)',
         show_legend=False)
axes[1].set_ylabel('')   # shared y-axis

fig.suptitle(
    'Mean coverage over time ± 1 std (50 episodes per method)\n'
    'Left: v11 environment; Right: v12 environment',
    fontsize=10, y=1.00)

out = os.path.join(OUTPUT_DIR, 'coverage_curves_v11_v12')
fig.savefig(out + '.pdf', bbox_inches='tight')
fig.savefig(out + '.png', bbox_inches='tight', dpi=150)
plt.close()
print(f"Saved: {out}.pdf / .png")

# ─────────────────────────────────────────────────────────────────────────────
# Figure 2: v11 standalone
# ─────────────────────────────────────────────────────────────────────────────
fig2, ax2 = plt.subplots(figsize=(7, 4.6))
fig2.subplots_adjust(left=0.10, right=0.97, top=0.88, bottom=0.13)
plot_env(ax2, v11,
         'Coverage over time — v11 (per-step goal reassignment)\nmean ± 1 std, 50 episodes per method',
         show_legend=True, annotate_mappo_lead=True)
out = os.path.join(OUTPUT_DIR, 'coverage_curves_v11_only')
fig2.savefig(out + '.pdf', bbox_inches='tight')
fig2.savefig(out + '.png', bbox_inches='tight', dpi=150)
plt.close()
print(f"Saved: {out}.pdf / .png")

# ─────────────────────────────────────────────────────────────────────────────
# Figure 3: v12 standalone
# ─────────────────────────────────────────────────────────────────────────────
fig3, ax3 = plt.subplots(figsize=(7, 4.6))
fig3.subplots_adjust(left=0.10, right=0.97, top=0.88, bottom=0.13)
plot_env(ax3, v12,
         'Coverage over time — v12 (persistent goals, 15-step timeout)\nmean ± 1 std, 50 episodes per method',
         show_legend=True)
out = os.path.join(OUTPUT_DIR, 'coverage_curves_v12_only')
fig3.savefig(out + '.pdf', bbox_inches='tight')
fig3.savefig(out + '.png', bbox_inches='tight', dpi=150)
plt.close()
print(f"Saved: {out}.pdf / .png")

print(f"\nAll plots saved to {OUTPUT_DIR}")