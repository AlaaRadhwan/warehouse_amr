#!/usr/bin/env python3
"""Two figures for the addendum: (1) aligned e01/p03/e05 comparison from existing recordings; (2) reduced yaw-only replica sweeps."""
import pickle, sys
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import *
from addendum_yaw_replica import simulate
OUT = INV / "analysis" / "addendum"; style(plt)
ser = pickle.load(open(OUT / "series.pkl", "rb"))
COL = {"e01": "#eb6834", "p03": "#2a78d6", "e05": "#1baf7a"}; LAB = {"e01": "e01 (spin)", "p03": "p03 (success)", "e05": "e05 (success)"}

# ---- Figure 1
fig, ax = plt.subplots(4, 1, figsize=(10, 11.5), sharex=True)
for k, rows in ser.items():
    t1 = next(r["t"] for r in rows if r["dist"] < 1.0); ring = next(r["t"] for r in rows if r["dist"] < 0.5)
    tau = np.array([r["t"] - t1 for r in rows]); m = (tau >= -6) & (tau <= 22)
    f = lambda key, s=1.0: np.array([r[key] for r in rows]) * s
    ax[0].plot(tau[m], f("dist")[m], color=COL[k], lw=1.8, label=LAB[k])
    ax[1].plot(tau[m], np.degrees(f("e"))[m], color=COL[k], lw=1.8)
    ax[2].plot(tau[m], f("cmd_wz")[m], color=COL[k], lw=1.8)
    ax[3].plot(tau[m], np.degrees(f("pred_dyaw"))[m] if False else f("pred_dyaw", 180 / np.pi)[m], color=COL[k], lw=1.8)
    for a in ax: a.axvline(ring - t1, color=COL[k], lw=1, ls=":")
ax[0].set_ylabel("distance to goal (m)\n[ground truth]"); ax[1].set_ylabel("goal yaw − robot yaw (deg)\n[ground truth; + = CCW needed]")
ax[2].set_ylabel("MPPI yaw-rate command (rad/s)"); ax[3].set_ylabel("MPPI's own predicted yaw change\nover the next 1 s (deg)")
ax[0].legend(frameon=False, fontsize=8); ax[2].axhline(WZ_MAX, color=GRID, lw=1.5); ax[2].text(21.8, WZ_MAX + 0.02, "wz_max", ha="right", fontsize=7, color=INK2)
ax[1].axhline(0, color=GRID, lw=1); ax[1].set_ylim(-190, 190)
ax[3].set_xlabel("time since the robot first came within 1.0 m of the goal (s)\ndotted verticals: entry into the 0.5 m ring where GoalAngleCritic switches on (one per run)")
for a in ax: clean(a)
fig.suptitle("e01 vs p03 vs e05 (all multiplier 1.0): same entry state at 1 m (≈144° heading error, ≈0.3 m/s forward);\ne01's yaw command is already higher at entry", x=0.01, ha="left", fontsize=9.5)
fig.tight_layout(rect=(0, 0, 1, 0.955)); fig.savefig(OUT / "fig_addendum_aligned.png", dpi=140); plt.close(fig)

# ---- Figure 2 (reduced model)
seeds = range(8); fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
for g, col, ls in ((1.076, "#eb6834", "-"), (0.98, "#2a78d6", "-")):
    cs = [0.0, 0.1, 0.2, 0.3, 0.35, 0.4, 0.5, 0.7, 1.0]
    fin = [[simulate(s, g, np.radians(27), g * c, c, cycles=400)[-1, 1] for s in seeds] for c in cs]
    ax[0].plot(cs, [np.median(f) for f in fin], color=col, lw=2, marker="o", ms=5, label=f"plant gain {g}")
    ax[0].fill_between(cs, [min(f) for f in fin], [max(f) for f in fin], color=col, alpha=0.15)
ax[0].plot([0.58], [1.0], "*", color="#d03b3b", ms=13, label="e01 at ring entry (cmd +0.58) → locked"); ax[0].plot([0.14], [0.0], "*", color="#1baf7a", ms=13, label="p01 at ring entry (cmd +0.14) → recovered")
ax[0].set_xlabel("yaw command when 'rotating away' from the goal heading (rad/s)"); ax[0].set_ylabel("command after 12 sim-s (rad/s)"); ax[0].set_title("Reduced model: locks to the bound only if\ngain > 1 and the entry rate is high enough"); ax[0].legend(frameon=False, fontsize=7); clean(ax[0])
gs = [0.92, 0.95, 0.98, 1.0, 1.01, 1.02, 1.03, 1.05, 1.076, 1.1]
fin = [[simulate(s, g, np.radians(27), g, 1.0, cycles=400)[-1, 1] for s in seeds] for g in gs]
ax[1].plot(gs, [np.median(f) for f in fin], color=INK, lw=2, marker="o", ms=5); ax[1].fill_between(gs, [min(f) for f in fin], [max(f) for f in fin], color=INK, alpha=0.12)
ax[1].axvline(1.0, color="#d03b3b", lw=1, ls="--"); ax[1].text(1.002, -0.1, "measured rate = wz_max", color="#d03b3b", fontsize=7)
ax[1].set_xlabel("plant gain (executed / commanded yaw rate)"); ax[1].set_ylabel("command after 12 sim-s (rad/s)"); ax[1].set_title("Pinned start (+1.0): releases at gain ≤ 1.00,\nstays pinned at gain ≥ 1.01"); clean(ax[1])
for g, col in ((1.076, "#eb6834"), (0.98, "#2a78d6")):
    for s in range(4):
        L = simulate(s, g, np.radians(-145), 0.3, 0.3, cycles=700); ax[2].plot(L[:, 0], L[:, 1], color=col, lw=1, alpha=0.8, label=f"plant gain {g}" if s == 0 else None)
ax[2].axhline(WZ_MAX, color=GRID, lw=1.5); ax[2].set_xlabel("simulated time (s)"); ax[2].set_ylabel("yaw command (rad/s)"); ax[2].set_title("Entry from 145° heading error (4 noise seeds each)"); ax[2].legend(frameon=False, fontsize=8); clean(ax[2])
fig.suptitle("REDUCED MODEL (yaw-only re-implementation of MPPI 1.3.12 mechanics; GoalAngle cost only). Consistency check, not a reproduction of the robot.", x=0.01, ha="left", fontsize=9.5)
fig.tight_layout(rect=(0, 0, 1, 0.94)); fig.savefig(OUT / "fig_addendum_replica.png", dpi=140); plt.close(fig); print("ok")
