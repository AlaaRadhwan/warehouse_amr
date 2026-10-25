#!/usr/bin/env python3
"""Classify one run from ground truth + Nav2 result using config/classification_criteria.yaml.
usage: classify.py RUN_DIR   -> writes RUN_DIR/classification.json and prints it."""
import csv, json, math, sys
from pathlib import Path

import yaml

INV = Path(__file__).resolve().parents[1]
C = yaml.safe_load((INV / "config/classification_criteria.yaml").read_text())
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


def load_gt(run):
    rows, seen = [], set()
    for r in list(csv.reader(open(run / "gt.csv")))[1:]:
        try: v = [float(x) for x in r]
        except ValueError: continue
        if v[1] in seen: continue
        seen.add(v[1]); rows.append(v)
    t = [r[1] for r in rows]; x = [r[2] for r in rows]; y = [r[3] for r in rows]
    yaw0 = [2 * math.atan2(r[7], r[8]) for r in rows]
    yaw, acc = [yaw0[0]], 0.0                      # unwrapped yaw
    for i in range(1, len(yaw0)):
        acc += wrap(yaw0[i] - yaw0[i - 1]); yaw.append(yaw0[0] + acc)
    return t, x, y, yaw


def derivs(t, x, y, yaw, w):
    v, wz, j = [], [], 0
    for i in range(len(t)):
        while t[i] - t[j] > w: j += 1
        dt = t[i] - t[j]
        v.append(math.hypot(x[i] - x[j], y[i] - y[j]) / dt if dt > 0 else 0.0)
        wz.append((yaw[i] - yaw[j]) / dt if dt > 0 else 0.0)
    return v, wz


def classify(run):
    run = Path(run); out = {"run": run.name, "criteria_file": "config/classification_criteria.yaml"}
    st = json.loads((run / "run_status.json").read_text()) if (run / "run_status.json").exists() else {"status": "MISSING"}
    cr = json.loads((run / "client_result.json").read_text()) if (run / "client_result.json").exists() else None
    out["run_status"], out["nav2_status"] = st.get("status"), (cr or {}).get("status")
    out["end_reason"] = (cr or {}).get("end_reason")
    if st.get("status", "").startswith("INFRA") or cr is None or not (run / "gt.csv").exists():
        out.update(outcome="INCONCLUSIVE", subtype=f"infrastructure:{st.get('status')}", reason="run did not reach a recorded goal"); return out
    t, x, y, yaw = load_gt(run)
    if len(t) < 100:
        out.update(outcome="INCONCLUSIVE", subtype="gt_too_short", reason=f"{len(t)} GT samples"); return out
    g = C["goal"]; tol = C["tolerance"]
    t_end = cr["sim_goal_sent"] + cr.get("sim_duration_s", 0.0)
    idx = [i for i in range(len(t)) if cr["sim_goal_sent"] <= t[i] <= t_end]
    if len(idx) < 50:
        out.update(outcome="INCONCLUSIVE", subtype="gt_misses_goal_interval", reason="GT does not cover the goal interval"); return out
    tt = [t[i] for i in idx]; xx = [x[i] for i in idx]; yy = [y[i] for i in idx]; yw = [yaw[i] for i in idx]
    v, wz = derivs(tt, xx, yy, yw, C["derivative_window_s"])
    dist = [math.hypot(a - g["x"], b - g["y"]) for a, b in zip(xx, yy)]
    yerr = [abs(wrap(a - g["yaw"])) for a in yw]
    final = {"x": xx[-1], "y": yy[-1], "yaw": wrap(yw[-1]), "dist_xy_m": dist[-1], "abs_yaw_err_rad": yerr[-1]}
    out["gt_final"] = final; out["gt_min_dist_m"] = min(dist)
    inside = dist[-1] <= tol["xy_m"] and yerr[-1] <= tol["yaw_rad"]

    def windows(wlen):
        j = 0
        for i in range(len(tt)):
            while tt[i] - tt[j] > wlen: j += 1
            if tt[i] - tt[0] >= wlen: yield j, i

    sp, sc = C["spin"], C["stall"]
    spin_ws, stall_ws, last_s = [], [], -1e9
    for j, i in windows(sp["window_s"]):
        if tt[i] - last_s < 1.0: continue
        last_s = tt[i]
        mw = (yw[i] - yw[j]) / (tt[i] - tt[j]); ms = sum(v[j:i + 1]) / (i - j + 1)
        if abs(mw) >= sp["min_abs_mean_wz_rad_s"] and ms <= sp["max_mean_speed_m_s"]:
            spin_ws.append((tt[j], tt[i], mw, ms, min(dist[j:i + 1])))
    last_s = -1e9
    for j, i in windows(sc["window_s"]):
        if tt[i] - last_s < 1.0: continue
        last_s = tt[i]
        disp = math.hypot(xx[i] - xx[j], yy[i] - yy[j]); mw = sum(abs(a) for a in wz[j:i + 1]) / (i - j + 1)
        if disp <= sc["max_net_displacement_m"] and mw <= sc["max_mean_abs_wz_rad_s"] and not (dist[i] <= tol["xy_m"] and yerr[i] <= tol["yaw_rad"]):
            stall_ws.append((tt[j], tt[i], disp, mw, dist[i], yerr[i]))
    g0 = cr["sim_goal_sent"]
    out["flags"] = {"spin_windows": len(spin_ws), "stall_windows": len(stall_ws),
                    "spin_onset_t_goal_s": round(spin_ws[0][0] - g0, 1) if spin_ws else None,
                    "stall_onset_t_goal_s": round(stall_ws[0][0] - g0, 1) if stall_ws else None,
                    "max_abs_gt_wz_rad_s": round(max(abs(a) for a in wz), 3)}
    out["watchdog_trigger"] = cr.get("watchdog_trigger")
    if cr.get("status") == "STATUS_SUCCEEDED":
        out.update(outcome="SUCCESS", subtype="gt_inside_tolerance" if inside else "gt_outside_tolerance",
                   reason="Nav2 SUCCEEDED" + ("" if inside else f"; GT final xy {dist[-1]:.3f} m, yaw {math.degrees(yerr[-1]):.1f} deg violates tolerance"))
    elif spin_ws:
        mind = min(w[4] for w in spin_ws)
        out.update(outcome="SPIN", subtype="terminal" if mind <= sp["terminal_if_min_dist_to_goal_m"] else "far_field",
                   reason=f"{len(spin_ws)} window(s) with |mean wz| >= {sp['min_abs_mean_wz_rad_s']} rad/s; first onset {spin_ws[0][0]-g0:.1f} s after goal; "
                          f"mean wz {spin_ws[0][2]:+.2f} rad/s, min dist to goal in spin {mind:.2f} m")
    elif stall_ws:
        w = stall_ws[-1]
        out.update(outcome="STALL", subtype="short_of_goal" if w[4] > tol["xy_m"] else "yaw_not_converged",
                   reason=f"{len(stall_ws)} window(s) of {sc['window_s']:.0f} s with net displacement <= {sc['max_net_displacement_m']} m; "
                          f"first onset {stall_ws[0][0]-g0:.1f} s after goal; last: {w[4]:.2f} m from goal, yaw err {math.degrees(w[5]):.0f} deg")
    elif cr.get("status") in ("STATUS_ABORTED", "STATUS_CANCELED") and cr.get("end_reason") == "nav2_result":
        out.update(outcome="OTHER_FAILURE", subtype=cr["status"].lower(), reason=f"Nav2 result {cr['status']} without spin/stall evidence")
    elif cr.get("end_reason") in ("sim_limit", "wall_limit") or str(cr.get("end_reason", "")).startswith("watchdog"):
        out.update(outcome="OTHER_FAILURE", subtype=f"ended_by_{cr.get('end_reason')}_without_spin_or_stall_criteria",
                   reason=f"{cr.get('end_reason')} reached; GT final {dist[-1]:.2f} m from goal; criteria for spin/stall not met")
    else:
        out.update(outcome="INCONCLUSIVE", subtype="no_rule_matched", reason=f"status {cr.get('status')}, end {cr.get('end_reason')}")
    return out


if __name__ == "__main__":
    r = classify(sys.argv[1]); Path(sys.argv[1], "classification.json").write_text(json.dumps(r, indent=2)); print(json.dumps(r, indent=2))
