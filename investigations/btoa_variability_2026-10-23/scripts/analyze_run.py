#!/usr/bin/env python3
"""Per-run analysis: metrics per hypothesis (-> analysis/<run>/metrics.json) and figures (-> analysis/<run>/*.png).
usage: analyze_run.py RUN_DIR     (needs extracted.pkl.gz from extract.py and classification.json from classify.py)"""
import json, sys
from pathlib import Path

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import *

pct = lambda a, q: float(np.percentile(a, q)) if len(a) else None
mx = lambda a: float(np.max(a)) if len(a) else None
mn = lambda a: float(np.mean(a)) if len(a) else None


# ------------------------------------------------------------------ H3: command vs executed
def lag_gain(t_gt, y_gt, c_t, c_y, thr, lags=np.arange(0, 1.01, 0.05)):
    """Best lag (s) and through-origin gain of GT response on the delayed command, using |cmd| > thr."""
    best = None
    for lag in lags:
        c = np.interp(t_gt - lag, c_t, c_y); m = np.abs(c) > thr
        if m.sum() < 20: continue
        corr = np.corrcoef(c[m], y_gt[m])[0, 1]
        if best is None or corr > best[1]: best = (float(lag), float(corr), float(np.sum(c[m] * y_gt[m]) / np.sum(c[m] ** 2)), int(m.sum()))
    return None if best is None else {"lag_s": best[0], "corr": best[1], "gain": best[2], "n": best[3]}


def commands(R):
    D, g, out = R.D, R.gt, {}
    m = R.in_goal(g["t"]); t = g["t"][m]
    for name, key in (("cmd_nav(MPPI)", "cmd_nav"), ("cmd_smoothed", "cmd_smoothed"), ("cmd(final)", "cmd"), ("cmd_out(controller)", "cmd_out")):
        A = D[key]; s = (A[:, 0] >= R.tg) & (A[:, 0] <= R.tend); A = A[s]
        if len(A) == 0: out[name] = None; continue
        out[name] = {"n": len(A), "max_abs_wz": mx(np.abs(A[:, 3])), "frac_wz_at_limit(>=0.99)": float(np.mean(np.abs(A[:, 3]) >= 0.99 * WZ_MAX)),
                     "max_vx": mx(A[:, 2]), "min_vx": float(np.min(A[:, 2])), "frac_vx_reverse": float(np.mean(A[:, 2] < -0.01))}
    nav, fin = D["cmd_nav"], D["cmd"]
    s = (nav[:, 0] >= R.tg) & (nav[:, 0] <= R.tend)
    f_wz = np.interp(nav[s, 0], fin[:, 0], fin[:, 3]); big = np.abs(nav[s, 3]) > 0.2
    out["final_over_mppi_wz_gain_median"] = float(np.median(f_wz[big] / nav[s, 3][big])) if big.sum() > 10 else None
    out["gt_wz_vs_cmd_final"] = lag_gain(t, g["wz"][m], fin[:, 0], fin[:, 3], 0.2)
    out["gt_wz_vs_cmd_final_saturated"] = lag_gain(t, g["wz"][m], fin[:, 0], fin[:, 3], 0.9 * WZ_MAX)
    out["gt_vx_vs_cmd_final"] = lag_gain(t, g["vx"][m], fin[:, 0], fin[:, 2], 0.1)
    for nm, k, col in (("raw_odom_wz_vs_gt", "odom_raw", 6), ("fused_odom_wz_vs_gt", "odom_fused", 6)):
        A = D[k]; q = (A[:, 0] >= R.tg) & (A[:, 0] <= R.tend); big = np.abs(g["wz"][m]) > 0.2
        ow = np.interp(t, A[q, 0], A[q, col]); out[nm] = {"gain_odom_over_gt_median": float(np.median(ow[big] / g["wz"][m][big]))} if big.sum() > 20 else None
    I = D["imu"]; q = (I[:, 0] >= R.tg) & (I[:, 0] <= R.tend); iw = np.interp(t, I[q, 0], I[q, 2]); big = np.abs(g["wz"][m]) > 0.2
    out["imu_wz_vs_gt"] = {"gain_imu_over_gt_median": float(np.median(iw[big] / g["wz"][m][big]))} if big.sum() > 20 else None
    return out


def optimal_pred(R, horizon_steps=20, dt_step=0.05):
    """MPPI's predicted pose 1.0 s ahead (/optimal_trajectory) vs where the robot actually was (ground truth)."""
    H = horizon_steps * dt_step; rows = []
    for p in R.D["paths"]["optimal"][::5]:
        s = p["sim"]
        if not (R.tg <= s <= R.tend - H) or len(p["x"]) <= horizon_steps: continue
        x, y, yaw = R.to_map(p["frame"], s, p["x"], p["y"], p["yaw"])
        gx0, gy0, gyaw0 = R.gt_at(s, "x"), R.gt_at(s, "y"), R.gt_yaw_at(s)
        gx1, gy1, gyaw1 = R.gt_at(s + H, "x"), R.gt_at(s + H, "y"), R.gt_yaw_at(s + H)
        rows.append((s - R.tg, float(np.hypot(x[0] - gx0, y[0] - gy0)), float(np.hypot(x[horizon_steps] - gx1, y[horizon_steps] - gy1)),
                     float(wrap(yaw[horizon_steps] - yaw[0])) / H, float((gyaw1 - gyaw0) / H),
                     float(np.hypot(x[horizon_steps] - x[0], y[horizon_steps] - y[0])), float(np.hypot(gx1 - gx0, gy1 - gy0))))
    a = np.array(rows) if rows else np.zeros((0, 7))
    return {"n": len(a), "start_pose_error_m_p50": pct(a[:, 1], 50), "start_pose_error_m_p95": pct(a[:, 1], 95), "pred_pos_error_1s_m_p50": pct(a[:, 2], 50),
            "pred_pos_error_1s_m_p95": pct(a[:, 2], 95), "pred_wz_vs_actual_wz_abs_diff_p95": pct(np.abs(a[:, 3] - a[:, 4]), 95) if len(a) else None,
            "series_columns": ["t_goal", "start_err_m", "pos_err_1s_m", "pred_wz", "actual_wz", "pred_disp_1s_m", "actual_disp_1s_m"], "series": a.tolist()}


# ------------------------------------------------------------------ H4: localization
def localization(R):
    D, out = R.D, {}
    A = D["amcl"]; q = (A[:, 0] >= R.tg) & (A[:, 0] <= R.tend); A = A[q]
    if len(A):
        e = np.hypot(A[:, 2] - R.gt_at(A[:, 0], "x"), A[:, 3] - R.gt_at(A[:, 0], "y")); ey = np.abs(wrap(A[:, 4] - R.gt_yaw_at(A[:, 0])))
        out["amcl_vs_gt"] = {"n": len(A), "xy_err_m_p50": pct(e, 50), "xy_err_m_p95": pct(e, 95), "xy_err_m_max": mx(e),
                             "yaw_err_deg_p95": pct(np.degrees(ey), 95), "yaw_err_deg_max": float(np.degrees(np.max(ey))), "series": np.c_[A[:, 0] - R.tg, e, np.degrees(ey)].tolist()}
    F = D["feedback"]
    if len(F):
        ts = R.wall2sim(F[:, 0]); q = (ts >= R.tg) & (ts <= R.tend); F, ts = F[q], ts[q]
        e = np.hypot(F[:, 1] - R.gt_at(ts, "x"), F[:, 2] - R.gt_at(ts, "y")); ey = np.abs(wrap(F[:, 3] - R.gt_yaw_at(ts)))
        dg = np.hypot(R.gt_at(ts, "x") - GOAL[0], R.gt_at(ts, "y") - GOAL[1])
        out["nav2_feedback_pose_vs_gt"] = {"n": len(F), "xy_err_m_p50": pct(e, 50), "xy_err_m_p95": pct(e, 95), "xy_err_m_max": mx(e), "yaw_err_deg_p95": pct(np.degrees(ey), 95),
                                           "yaw_err_deg_max": float(np.degrees(np.max(ey))), "series": np.c_[ts - R.tg, e, np.degrees(ey), F[:, 4], dg].tolist(),
                                           "series_columns": ["t_goal", "xy_err_m", "yaw_err_deg", "nav2_distance_remaining_m", "gt_euclid_dist_to_goal_m"]}
    M = D["map_odom"]
    if len(M) > 2:
        q = (M[:, 0] >= R.tg) & (M[:, 0] <= R.tend); M = M[q]
        if len(M) > 2:
            j = np.hypot(np.diff(M[:, 2]), np.diff(M[:, 3])); jy = np.abs(wrap(np.diff(M[:, 4])))
            out["map_to_odom_jumps"] = {"max_step_m": mx(j), "n_steps_gt_0.05m": int(np.sum(j > 0.05)), "max_yaw_step_deg": float(np.degrees(np.max(jy)))}
    return out


# ------------------------------------------------------------------ H5: timing / load
def timing(R):
    D, out = R.D, {}
    A = D["cmd_nav"]; q = (A[:, 0] >= R.tg) & (A[:, 0] <= R.tend); A = A[q]
    if len(A) > 5:
        ds, dw = np.diff(A[:, 0]), np.diff(A[:, 1])
        out["mppi_cycle_sim_dt_s"] = {"n": len(ds), "p50": pct(ds, 50), "p95": pct(ds, 95), "p99": pct(ds, 99), "max": mx(ds),
                                      "frac_gt_0.075": float(np.mean(ds > 0.075)), "n_gt_0.2": int(np.sum(ds > 0.2)), "nominal": 0.05}
        out["mppi_cycle_wall_dt_s"] = {"p50": pct(dw, 50), "p95": pct(dw, 95), "p99": pct(dw, 99), "max": mx(dw)}
        out["cmd_nav_series"] = np.c_[A[1:, 0] - R.tg, ds, dw].tolist()
    ro = D["rosout"]
    if len(ro): out["rosout_missed_rate_warnings"] = int(sum("missed its desired rate" in str(r[4]) for r in ro))
    s = R.sm
    if s is not None:
        w0, w1 = R.cr["wall_goal_sent"], R.cr["wall_goal_sent"] + R.cr.get("wall_duration_s", 0)
        q = (s["wall"] >= w0) & (s["wall"] <= w1) & np.isfinite(s["rtf"])
        if q.sum() > 3:
            out["system_during_goal"] = {"n_samples": int(q.sum()), "rtf_mean": mn(s["rtf"][q]), "rtf_p5": pct(s["rtf"][q], 5), "rtf_min": float(np.min(s["rtf"][q])),
                                         "load1_mean": mn(s["load1"][q]), "load1_max": mx(s["load1"][q]), "cpu_total_pct_mean": mn(s["cpu_total_pct"][q]),
                                         "ncpu": int(s["ncpu"][0]), "controller_server_cpu_mean": mn(s["cpu_controller_server"][q]),
                                         "controller_server_cpu_max": mx(s["cpu_controller_server"][q]), "gz_server_cpu_mean": mn(s["cpu_gz_server"][q]),
                                         "mem_avail_mb_min": float(np.min(s["mem_avail_mb"][q]))}
    return out


# ------------------------------------------------------------------ H2: global path geometry / H1: association
def plans(R):
    out = {}
    for key in ("unsmoothed", "plan"):
        P = [p for p in R.D["paths"][key] if R.tg - 0.5 <= p["sim"] <= R.tend]
        if not P: continue
        st = [path_stats(p) for p in P]
        out[key] = {"n_plans": len(P), "first": st[0], "last": st[-1], "cusps_min_max": [min(s.get("cusps", 0) for s in st), max(s.get("cusps", 0) for s in st)],
                    "n_cusp_count_changes": int(sum(st[i]["cusps"] != st[i - 1]["cusps"] for i in range(1, len(st)) if "cusps" in st[i] and "cusps" in st[i - 1])),
                    "length_range_m": [min(s.get("length_m", 0) for s in st), max(s.get("length_m", 0) for s in st)],
                    "max_length_over_straight": max((s.get("length_over_straight", 0) for s in st if np.isfinite(s.get("length_over_straight", 0))), default=None),
                    "series_columns": ["t_goal", "length_m", "cusps", "length_over_straight", "reverse_length_m", "end_dist_to_goal_m", "total_tangent_turn_rad"],
                    "series": [[p["sim"] - R.tg, s.get("length_m", 0), s.get("cusps", 0), min(s.get("length_over_straight", 99), 99), s.get("reverse_length_m", 0),
                                float(np.hypot(p["x"][-1] - GOAL[0], p["y"][-1] - GOAL[1])) if len(p["x"]) else None, s.get("total_tangent_turn_rad", 0)] for p, s in zip(P, st)]}
    return out


def association(R):
    """For every MPPI transformed plan: does it end where the global plan says it should (robot-relative prune / cusp)?"""
    G = sorted([p for p in R.D["paths"]["plan"] if len(p["x"]) > 2], key=lambda p: p["sim"])
    gs = np.array([p["sim"] for p in G]); rows = []
    prev_i = None
    for p in R.D["paths"]["transformed"]:
        s = p["sim"]
        if not (R.tg <= s <= R.tend) or len(p["x"]) < 2: continue
        k = np.searchsorted(gs, s, side="right") - 1
        if k < 0: continue
        gp = G[k]; gx, gy, gyaw = gp["x"], gp["y"], gp["yaw"]
        rx, ry = R.gt_at(s, "x"), R.gt_at(s, "y")
        d = np.hypot(gx - rx, gy - ry); i = int(np.argmin(d)); arc = np.r_[0, np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))]
        L_rem = float(arc[-1] - arc[i]); cu = cusp_indices(gx, gy, gyaw); ahead = cu[cu > i]
        s_cusp = float(arc[ahead[0]] - arc[i]) if len(ahead) else None
        amb = bool(np.any((np.abs(arc - arc[i]) > 1.0) & (d <= d[i] + 0.3)))
        tx, ty = R.to_map(p["frame"], s, p["x"], p["y"]); L_tr = seg_len(tx, ty)
        expected = min([L_rem, PRUNE] + ([s_cusp] if s_cusp is not None else []))
        jump = None if prev_i is None else float(arc[i] - prev_i[0]) - 0.0
        prev_i = (arc[i], s)
        rows.append([s - R.tg, L_tr, expected, L_rem, -1.0 if s_cusp is None else s_cusp, float(d[i]), float(amb),
                     float(np.hypot(tx[-1] - gx[-1], ty[-1] - gy[-1])), float(L_tr < expected - 0.5), 0.0 if jump is None else jump,
                     float(np.hypot(tx[-1] - rx, ty[-1] - ry))])
    a = np.array(rows) if rows else np.zeros((0, 11))
    cols = ["t_goal", "L_transformed_m", "L_expected_m", "L_remaining_on_global_m", "arclen_to_next_cusp_m(-1 none)", "cross_track_m", "ambiguous_nearest(0/1)",
            "transformed_end_to_global_end_m", "short_transformed(0/1)", "d_nearest_arclen_step_m", "transformed_end_to_robot_m"]
    out = {"n": len(a), "columns": cols}
    if len(a):
        out.update({"frac_ambiguous_nearest": float(np.mean(a[:, 6])), "frac_short_transformed_plan": float(np.mean(a[:, 8])),
                    "max_cross_track_m": mx(a[:, 5]), "n_nearest_arclen_jumps_gt_0.5m": int(np.sum(np.abs(a[:, 9]) > 0.5)),
                    "first_short_transformed_t_goal": float(a[a[:, 8] > 0][0, 0]) if np.any(a[:, 8] > 0) else None, "series": a.tolist()})
    return out


# ------------------------------------------------------------------ failure context
def context(R):
    c = R.cls or {}; fl = c.get("flags", {}) or {}
    onset = fl.get("spin_onset_t_goal_s") if c.get("outcome") == "SPIN" else fl.get("stall_onset_t_goal_s") if c.get("outcome") == "STALL" else None
    tq = (R.tend - R.tg) - 5.0 if onset is None else onset + 5.0          # snapshot 5 s into the failure (or 5 s before the end)
    s = R.tg + tq; out = {"snapshot_t_goal": tq, "onset_t_goal": onset}
    x, y, yaw = R.gt_at(s, "x"), R.gt_at(s, "y"), R.gt_yaw_at(s)
    brg = np.arctan2(GOAL[1] - y, GOAL[0] - x)
    out["gt_pose"] = {"x": float(x), "y": float(y), "yaw_deg": float(np.degrees(wrap(yaw))), "dist_to_goal_m": float(np.hypot(GOAL[0] - x, GOAL[1] - y)),
                      "goal_bearing_rel_heading_deg": float(np.degrees(wrap(brg - yaw))), "yaw_err_to_goal_yaw_deg": float(np.degrees(wrap(GOAL[2] - yaw)))}
    for nm, key in (("cmd_nav", "cmd_nav"), ("cmd_final", "cmd")):
        A = R.D[key]; i = int(np.argmin(np.abs(A[:, 0] - s))); out[nm] = {"vx": float(A[i, 2]), "wz": float(A[i, 3])}
    P = R.D["paths"]
    for nm in ("plan", "unsmoothed"):
        L = [p for p in P[nm] if p["sim"] <= s]
        if L: out[nm + "_at_snapshot"] = path_stats(L[-1])
    T = [p for p in P["transformed"] if p["sim"] <= s]
    if T:
        p = T[-1]; X, Y, Yw = R.to_map(p["frame"], p["sim"], p["x"], p["y"], p["yaw"])
        out["transformed_at_snapshot"] = {"n": len(X), "length_m": seg_len(X, Y), "end_xy": [float(X[-1]), float(Y[-1])], "end_dist_to_goal_m": float(np.hypot(X[-1] - GOAL[0], Y[-1] - GOAL[1])),
                                          "cusps": int(len(cusp_indices(X, Y, Yw)))}
    O = [p for p in P["optimal"] if p["sim"] <= s]
    if O:
        p = O[-1]; X, Y, Yw = R.to_map(p["frame"], p["sim"], p["x"], p["y"], p["yaw"]); k = min(20, len(X) - 1)
        out["optimal_at_snapshot"] = {"disp_1s_m": float(np.hypot(X[k] - X[0], Y[k] - Y[0])), "heading_change_1s_deg": float(np.degrees(wrap(Yw[k] - Yw[0]))),
                                      "direction_1s": "reverse" if (X[k] - X[0]) * np.cos(Yw[0]) + (Y[k] - Y[0]) * np.sin(Yw[0]) < -0.01 else "forward_or_rotate"}
    ev = []
    for r in R.D["rosout"]:
        if R.tg <= r[0] <= R.tend: ev.append([round(r[0] - R.tg, 1), int(r[2]), str(r[3]), str(r[4])[:160]])
    out["rosout_events"] = ev[:200]
    return out


def events(R):
    ev = []
    for e in (R.cls or {}).get("flags", {}).items():
        if e[0].endswith("onset_t_goal_s") and e[1] is not None: ev.append((e[1], "spin onset" if "spin" in e[0] else "stall onset", "#d03b3b"))
    cr = R.cr or {}
    if cr.get("watchdog_trigger"): ev.append((cr["watchdog_trigger"]["t_goal_sim"], "watchdog: " + cr["watchdog_trigger"]["rule"], "#52514e"))
    for r in R.D["rosout"]:
        m = str(r[4])
        if R.tg <= r[0] <= R.tend and any(k in m for k in ("Failed to make progress", "recover", "Recover")): ev.append((r[0] - R.tg, m[:34], "#eda100"))
    for w, lst in R.D["follow_status"]:
        pass
    return sorted(ev)


# ------------------------------------------------------------------ figures
def fig_timeline(R, M, out):
    g = R.gt; m = R.in_goal(g["t"]); t = g["t"][m] - R.tg
    fig, ax = plt.subplots(5, 1, figsize=(10, 11), sharex=True)
    dist = np.hypot(g["x"][m] - GOAL[0], g["y"][m] - GOAL[1]); yerr = np.degrees(wrap(GOAL[2] - g["yaw"][m]))
    ax[0].plot(t, dist, color=INK, lw=1.6); ax[0].axhline(0.25, color="#d03b3b", lw=1, ls="--"); ax[0].set_ylabel("distance to goal (m)\n[ground truth]")
    ax[0].text(t[-1], 0.27, "xy tolerance 0.25 m", color="#d03b3b", ha="right", fontsize=8)
    ax[1].plot(t, yerr, color=INK, lw=1.6); ax[1].axhspan(-11.46, 11.46, color="#1baf7a", alpha=0.12); ax[1].set_ylabel("goal yaw − robot yaw (deg)\n[ground truth]")
    ax[1].text(t[-1], 14, "yaw tolerance ±0.20 rad", color=INK2, ha="right", fontsize=8)
    nav, fin = R.D["cmd_nav"], R.D["cmd"]
    for A, col, lab, lw in ((nav, "#2a78d6", "MPPI output (/cmd_vel_nav)", 1.4), (fin, "#eb6834", "to robot (/cmd_vel)", 1.0)):
        q = (A[:, 0] >= R.tg) & (A[:, 0] <= R.tend); ax[2].plot(A[q, 0] - R.tg, A[q, 3], color=col, lw=lw, label=lab); ax[3].plot(A[q, 0] - R.tg, A[q, 2], color=col, lw=lw, label=lab)
    ax[2].plot(t, g["wz"][m], color=INK, lw=1.0, ls="-", alpha=0.8, label="actual (ground truth)"); ax[3].plot(t, g["vx"][m], color=INK, lw=1.0, alpha=0.8, label="actual (ground truth)")
    for v in (WZ_MAX, -WZ_MAX): ax[2].axhline(v, color=GRID, lw=1.2)
    ax[2].set_ylabel("yaw rate (rad/s)"); ax[3].set_ylabel("forward speed (m/s)\n(negative = reverse)"); ax[2].legend(frameon=False, ncol=3, fontsize=8, loc="upper right")
    ctr = [r for r in R.D["follow_status"]]
    # cusp-aware plan summary: reverse length of the current global plan
    pl = M.get("plans", {}).get("plan")
    if pl: s = np.array(pl["series"]); ax[4].step(s[:, 0], s[:, 2], color="#6250d6", where="post", label="cusps in /plan"); ax[4].step(s[:, 0], s[:, 4], color="#1baf7a", where="post", label="reversing length (m)"); ax[4].legend(frameon=False, fontsize=8)
    ax[4].set_ylabel("global plan:\ncusps / reverse m"); ax[4].set_xlabel("sim time since goal sent (s)")
    for a in ax:
        clean(a)
        for tt, lab, col in events(R):
            a.axvline(tt, color=col, lw=1, ls=":")
    for tt, lab, col in events(R): ax[0].text(tt, ax[0].get_ylim()[1], " " + lab, color=col, fontsize=7, va="top", rotation=90)
    c = R.cls or {}
    fig.suptitle(f"{R.id}  |  multiplier {R.mult}  |  {c.get('outcome')} ({c.get('subtype')})  |  Nav2: {R.cr.get('status')}, ended by {R.cr.get('end_reason')}", x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(out / "fig_timeline.png", dpi=140); plt.close(fig)


def draw_foot(ax, x, y, yaw, **kw):
    c, s = np.cos(yaw), np.sin(yaw); P = FOOT @ np.array([[c, s], [-s, c]]); ax.add_patch(Polygon(P + [x, y], closed=True, fill=False, **kw))


def fig_paths(R, M, out):
    g = R.gt; m = R.in_goal(g["t"]); x, y, t = g["x"][m], g["y"][m], g["t"][m] - R.tg
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.2))
    for a, zoom in zip(ax, (False, True)):
        pts = np.c_[x, y].reshape(-1, 1, 2); lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], 1), cmap="Blues", norm=plt.Normalize(-0.3 * t.max(), t.max()), lw=2.4); lc.set_array(t[:-1]); a.add_collection(lc)
        P = [p for p in R.D["paths"]["unsmoothed"] if R.tg - 0.5 <= p["sim"] <= R.tend]
        if P:
            a.plot(P[0]["x"], P[0]["y"], color="#9a9a96", lw=1.2, ls="--", label="first global plan (raw lattice)"); a.plot(P[-1]["x"], P[-1]["y"], color="#eb6834", lw=1.2, label="last global plan (raw lattice)")
            for p, mk in ((P[0], "x"), (P[-1], "+")):
                cu = cusp_indices(p["x"], p["y"], p["yaw"]); a.plot(p["x"][cu], p["y"][cu], mk, color="#d03b3b", ms=9, mew=2, label="direction change (cusp)" if p is P[0] else None)
        draw_foot(a, x[-1], y[-1], g["yaw"][m][-1], color=INK, lw=1.4); a.plot(x[-1], y[-1], "o", color=INK, ms=5)
        a.annotate("", xy=(GOAL[0] + 0.5 * np.cos(GOAL[2]), GOAL[1]), xytext=(GOAL[0], GOAL[1]), arrowprops=dict(arrowstyle="->", color="#1baf7a", lw=2)); a.plot(*GOAL[:2], "s", color="#1baf7a", ms=7)
        if zoom: a.set_xlim(GOAL[0] - 2.2, GOAL[0] + 2.8); a.set_ylim(GOAL[1] - 2.5, GOAL[1] + 2.5)
        else: a.autoscale()
        a.set_aspect("equal"); clean(a); a.set_xlabel("x (m, map)"); a.set_ylabel("y (m, map)")
        a.set_title("Whole route" if not zoom else "Zoom on goal (green square + arrow = goal pose; box = final footprint)")
    ax[0].legend(frameon=False, fontsize=7, loc="best"); cb = fig.colorbar(lc, ax=ax, fraction=0.025, pad=0.01); cb.set_label("ground-truth time since goal (s)")
    fig.suptitle(f"{R.id}  |  multiplier {R.mult}  |  {(R.cls or {}).get('outcome')}", x=0.01, ha="left", fontsize=10)
    fig.savefig(out / "fig_paths.png", dpi=140, bbox_inches="tight"); plt.close(fig)


def fig_snapshots(R, M, out):
    c = M["context"]; tq = c["snapshot_t_goal"]; times = [tq - 4, tq, min(tq + 12, R.tend - R.tg - 0.5)]
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.9))
    for a, tt in zip(ax, times):
        s = R.tg + tt; X0, Y0 = R.gt_at(s, "x"), R.gt_at(s, "y")
        C = [q for q in R.D["costmap"] if q["sim"] <= s]
        if C:
            q = C[-1]; grid = q["data"].reshape(q["h"], q["w"]).astype(float); grid[grid < 0] = np.nan
            ox, oy = R.to_map(q["frame"], q["sim"], np.array(q["ox"]), np.array(q["oy"])); e = [ox, ox + q["w"] * q["res"], oy, oy + q["h"] * q["res"]]
            a.imshow(grid, origin="lower", extent=e, cmap="Greys", vmin=0, vmax=100, alpha=0.8, zorder=0)
        for nm, col, lw, lab in (("plan", "#9a9a96", 1.6, "global plan (/plan)"),):
            L = [p for p in R.D["paths"][nm] if p["sim"] <= s]
            if L: a.plot(L[-1]["x"], L[-1]["y"], color=col, lw=lw, label=lab)
        for nm, col, lw, lab in (("transformed", "#2a78d6", 2.2, "plan handed to MPPI (/transformed_global_plan)"), ("optimal", "#eb6834", 2.2, "MPPI predicted path 2.8 s (/optimal_trajectory)")):
            L = [p for p in R.D["paths"][nm] if p["sim"] <= s]
            if L: X, Y = R.to_map(L[-1]["frame"], L[-1]["sim"], L[-1]["x"], L[-1]["y"]); a.plot(X, Y, color=col, lw=lw, label=lab); a.plot(X[-1], Y[-1], "o", color=col, ms=4)
        draw_foot(a, X0, Y0, R.gt_yaw_at(s), color=INK, lw=1.6); a.plot(X0, Y0, "o", color=INK, ms=4)
        a.plot(*GOAL[:2], "s", color="#1baf7a", ms=8); a.annotate("", xy=(GOAL[0] + 0.5, GOAL[1]), xytext=GOAL[:2], arrowprops=dict(arrowstyle="->", color="#1baf7a", lw=2))
        a.set_xlim(X0 - 2.2, X0 + 2.2); a.set_ylim(Y0 - 2.2, Y0 + 2.2); a.set_aspect("equal"); clean(a); a.set_xlabel("x (m, map)")
        a.set_title(f"t = {tt:.0f} s after goal  (dist {np.hypot(X0 - GOAL[0], Y0 - GOAL[1]):.2f} m)")
    ax[0].set_ylabel("y (m, map)"); ax[0].legend(frameon=False, fontsize=6.5, loc="lower left")
    fig.suptitle(f"{R.id} | {(R.cls or {}).get('outcome')}: robot (black box), local costmap (grey, darker = costlier), goal (green)", x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(out / "fig_snapshots.png", dpi=140); plt.close(fig)


def fig_association(R, M, out):
    a = np.array(M["association"].get("series", []))
    if not len(a): return
    fig, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    ax[0].plot(a[:, 0], a[:, 1], color="#2a78d6", lw=1.4, label="transformed plan length handed to MPPI"); ax[0].plot(a[:, 0], a[:, 2], color=INK, lw=1, ls="--", label="expected: min(remaining on global plan, 2.5 m prune, distance to next cusp)")
    ax[0].set_ylabel("path length (m)"); ax[0].legend(frameon=False, fontsize=7)
    ax[1].plot(a[:, 0], a[:, 5], color="#eb6834", lw=1.4, label="robot-to-global-plan distance (cross-track)"); ax[1].plot(a[:, 0], a[:, 6] * 0.5, color="#d03b3b", lw=0.8, drawstyle="steps-post", label="nearest point ambiguous (loop within 0.3 m of best) [0.5 = yes]")
    ax[1].set_ylabel("m"); ax[1].legend(frameon=False, fontsize=7)
    ax[2].plot(a[:, 0], a[:, 3], color=INK, lw=1.2, label="remaining length on global plan from nearest point"); ax[2].plot(a[:, 0], np.where(a[:, 4] >= 0, a[:, 4], np.nan), color="#6250d6", lw=1.2, label="distance along plan to next cusp")
    ax[2].set_ylabel("m"); ax[2].set_xlabel("sim time since goal sent (s)"); ax[2].legend(frameon=False, fontsize=7)
    for q in ax: clean(q)
    fig.suptitle(f"{R.id} | MPPI path association check (H1/H2): is the local plan consistent with the global plan?", x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(out / "fig_association.png", dpi=140); plt.close(fig)


def fig_timing_loc(R, M, out):
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    s = np.array(M["timing"].get("cmd_nav_series", []))
    if len(s):
        ax[0].plot(s[:, 0], s[:, 2], color="#eb6834", lw=0.8, label="wall-clock interval between MPPI outputs"); ax[0].plot(s[:, 0], s[:, 1], color="#2a78d6", lw=0.8, label="sim-time interval between MPPI outputs")
        ax[0].axhline(0.05, color=INK, lw=0.8, ls="--"); ax[0].text(s[-1, 0], 0.052, "controller_frequency 20 Hz = 0.05 s", ha="right", fontsize=7, color=INK2)
        ax[0].set_ylabel("MPPI cycle interval (s)"); ax[0].legend(frameon=False, fontsize=7, loc="upper right"); ax[0].set_ylim(0, max(0.12, min(1.0, np.percentile(s[:, 2], 99.9) * 1.2)))
    sm = R.sm
    if sm is not None:
        t = R.rel(R.wall2sim(sm["wall"])) if False else sm["sim"] - R.tg
        ax[1].plot(t, sm["rtf"], color="#1baf7a", lw=1.2, label="real-time factor"); ax[1].set_ylabel("real-time factor"); ax[1].set_ylim(0, 1.1)
        ax[1].twinx().plot(t, sm["load1"], color="#eda100", lw=1.0)    # secondary axis: load average (labelled below)
        ax[1].text(0.01, 0.05, "orange (right axis): 1-min load average (16 cores)", transform=ax[1].transAxes, fontsize=7, color="#c98500")
        ax[2].plot(t, sm["cpu_controller_server"], color="#2a78d6", lw=1.2, label="controller_server (MPPI)"); ax[2].plot(t, sm["cpu_gz_server"], color="#eb6834", lw=1.2, label="gz sim server"); ax[2].plot(t, sm["cpu_total_pct"], color=INK, lw=1.0, label="whole machine (%)")
        ax[2].set_ylabel("CPU (%; 100 = one core)"); ax[2].legend(frameon=False, fontsize=7)
    L = M["localization"]; f = L.get("nav2_feedback_pose_vs_gt"); am = L.get("amcl_vs_gt")
    if f: q = np.array(f["series"]); ax[3].plot(q[:, 0], q[:, 1], color="#2a78d6", lw=1.2, label="Nav2 feedback pose − ground truth")
    if am: q = np.array(am["series"]); ax[3].plot(q[:, 0], q[:, 1], color="#eb6834", lw=1.2, label="AMCL pose − ground truth")
    ax[3].set_ylabel("position error (m)"); ax[3].legend(frameon=False, fontsize=7); ax[3].set_xlabel("sim time since goal sent (s)")
    for q in ax: clean(q)
    fig.suptitle(f"{R.id} | timing / load (H5) and localization (H4)", x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(out / "fig_timing_localization.png", dpi=140); plt.close(fig)


def main(run):
    R = Run(run); out = INV / "analysis" / R.id; out.mkdir(parents=True, exist_ok=True)
    if R.D is None or R.gt is None or R.cr is None: print("insufficient data for", R.id); return
    style(plt)
    M = {"run": R.id, "multiplier": R.mult, "classification": R.cls, "client": {k: R.cr.get(k) for k in ("status", "end_reason", "sim_duration_s", "wall_duration_s", "watchdog_trigger", "last_feedback")}}
    M["commands"] = commands(R); M["timing"] = timing(R); M["localization"] = localization(R); M["plans"] = plans(R)
    M["association"] = association(R); M["optimal_vs_actual"] = optimal_pred(R); M["context"] = context(R)
    (out / "metrics.json").write_text(json.dumps(M, indent=1, default=float))
    for f in (fig_timeline, fig_paths, fig_snapshots, fig_association, fig_timing_loc): f(R, M, out)
    print("analysed", R.id, "->", out)


if __name__ == "__main__": main(sys.argv[1])
