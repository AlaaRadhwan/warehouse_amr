#!/usr/bin/env python3
"""Per-run metrics and figures for the B -> A (start at station B) runs of 2026-10-23.
usage (from this folder): python3 -I analyze_runs.py data/runs .
Inputs per run dir: gt.csv (Gazebo ground truth pose, sim time), launch.log, params.txt, b_to_a_result.json (only if client finished).
"""
import csv, json, math, re, sys
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCR, OUT = Path(sys.argv[1]), Path(sys.argv[2])
GOAL = (0.099, -0.175, 0.0)
RUNS = [  # (label, dir, short)
    ("A  mult 1.0, spin",    "B1_atB_mult1.0_A_spin", "A"),
    ("B  mult 0.92754, stall", "B2_atB_mult0.92754_B_stall", "B"),
    ("C  mult 1.0, success", "B3_atB_mult1.0_C_success", "C"),
]
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))

def load(d):
    rows = [[float(v) for v in r] for r in list(csv.reader(open(SCR / d / "gt.csv")))[1:]]
    seen, out = set(), []
    for r in rows:                       # de-duplicate repeated stamps
        if r[1] in seen: continue
        seen.add(r[1]); out.append(r)
    t0 = out[0][1]
    t = [r[1] - t0 for r in out]; x = [r[2] for r in out]; y = [r[3] for r in out]
    yaw = [2 * math.atan2(r[7], r[8]) for r in out]
    return t, x, y, yaw

def analyse(d):
    t, x, y, yaw = load(d)
    dist = [math.hypot(a - GOAL[0], b - GOAL[1]) for a, b in zip(x, y)]
    cum, c = [0.0], 0.0
    for i in range(1, len(t)):
        c += wrap(yaw[i] - yaw[i - 1]); cum.append(c)
    # yaw rate: 0.5 s finite difference of unwrapped yaw
    wz = []
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > 0.5: j += 1
        wz.append((cum[i] - cum[j]) / (t[i] - t[j]) if t[i] > t[j] else 0.0)
    def first(cond):
        return next((round(t[i], 1) for i in range(len(t)) if cond(i)), None)
    m = {"gt_span_s": round(t[-1], 1), "samples": len(t),
         "t_dist_lt_1m": first(lambda i: dist[i] < 1.0), "t_dist_lt_0.25m": first(lambda i: dist[i] < 0.25),
         "t_dist_lt_0.10m": first(lambda i: dist[i] < 0.10),
         "min_dist_m": round(min(dist), 3), "t_min_dist": round(t[dist.index(min(dist))], 1),
         "final_xy": [round(x[-1], 3), round(y[-1], 3)], "final_dist_m": round(dist[-1], 3),
         "final_yaw_deg": round(math.degrees(wrap(yaw[-1])), 1),
         "total_rotation_deg": round(math.degrees(cum[-1])), "path_len_m": round(sum(math.hypot(x[i]-x[i-1], y[i]-y[i-1]) for i in range(1, len(t))), 1),
         "max_abs_wz_rad_s": round(max(abs(v) for v in wz), 3)}
    # last 20 s (or last 20 s of motion window)
    tail = [i for i in range(len(t)) if t[i] >= t[-1] - 20]
    m["last20s_mean_abs_wz"] = round(sum(abs(wz[i]) for i in tail) / len(tail), 3)
    m["last20s_signed_mean_wz"] = round(sum(wz[i] for i in tail) / len(tail), 3)
    m["last20s_xy_range_m"] = round(max(math.hypot(x[i]-x[tail[0]], y[i]-y[tail[0]]) for i in tail), 3)
    m["last20s_yaw_change_deg"] = round(math.degrees(cum[-1] - cum[tail[0]]))
    return (t, x, y, yaw, dist, cum, wz), m

def logfacts(d):
    L = (SCR / d / "launch.log").read_text(errors="replace")
    return {"passing_new_path": L.count("Passing new path"),
            "control_loop_missed_rate": len(re.findall(r"Control loop missed its desired rate", L)),
            "failed_to_make_progress": L.count("Failed to make progress"),
            "local_costmap_clear_requests": L.count("clear entirely the local_costmap")}

res, series = {}, {}
for label, d, k in RUNS:
    s, m = analyse(d); m.update(logfacts(d)); m["dir"] = d; m["label"] = label
    p = SCR / d / "b_to_a_result.json"
    if p.exists():
        r = json.loads(p.read_text()); m["client"] = {k2: r[k2] for k2 in ("status", "sim_duration_s", "min_dist_to_goal_xy", "last_feedback")}
    else:
        m["client"] = "no result json (client killed by 150 s wall-clock cap)"
    res[k] = m; series[k] = s
(OUT / "data" / "run_metrics.json").write_text(json.dumps(res, indent=2))

# ---- figures (palette: reference instance, series slots 1-3, light surface) ----
SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e1"
COL = {"A": "#2a78d6", "B": "#eb6834", "C": "#1baf7a"}
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "text.color": INK, "font.size": 10})
def clean(ax):
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)

# Fig 1: distance to goal and cumulative rotation vs time (shared time axis, two panels, one measure each)
fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6.2), sharex=True)
for k, (t, x, y, yaw, dist, cum, wz) in series.items():
    a1.plot(t, dist, color=COL[k], lw=2); a2.plot(t, [math.degrees(c) for c in cum], color=COL[k], lw=2)
    a1.text(t[-1] + 0.8, dist[-1], k, color=INK, va="center", weight="bold")
    a2.text(t[-1] + 0.8, math.degrees(cum[-1]), k, color=INK, va="center", weight="bold")
a1.set_ylabel("distance to goal XY (m)"); a2.set_ylabel("cumulative yaw rotation (deg)"); a2.set_xlabel("sim time since recorder start (s)  [goal sent ~3 s after t=0]")
a1.set_title("Ground truth, three runs started at Station B", loc="left", color=INK)
for a in (a1, a2): clean(a)
handles = [plt.Line2D([0], [0], color=COL[k], lw=2, label=res[k]["label"]) for k in res]
a1.legend(handles=handles, frameon=False, loc="upper right", labelcolor=INK2)
fig.tight_layout(); fig.savefig(OUT / "figures" / "fig1_distance_and_rotation.png", dpi=150); plt.close(fig)

# Fig 2: XY near the goal, one panel per run (same scale)
fig, axs = plt.subplots(1, 3, figsize=(11, 4), sharex=True, sharey=True)
for ax, (k, (t, x, y, yaw, dist, cum, wz)) in zip(axs, series.items()):
    idx = [i for i in range(len(t)) if x[i] < 1.5]
    ax.plot([x[i] for i in idx], [y[i] for i in idx], color=COL[k], lw=1.6)
    ax.plot(x[-1], y[-1], "o", color=COL[k], ms=8, mec=SURF, mew=2)
    ax.plot(GOAL[0], GOAL[1], "+", color=INK, ms=14, mew=2)
    ax.set_title(res[k]["label"], loc="left", fontsize=9, color=INK); clean(ax); ax.set_aspect("equal")
    ax.set_xlabel("x (m, map)")
axs[0].set_ylabel("y (m, map)")
axs[0].text(0.02, 0.04, "+ goal   • final pose", transform=axs[0].transAxes, color=INK2, fontsize=8)
fig.tight_layout(); fig.savefig(OUT / "figures" / "fig2_xy_near_goal.png", dpi=150); plt.close(fig)
print(json.dumps(res, indent=1))
