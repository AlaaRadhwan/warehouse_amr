#!/usr/bin/env python3
"""Addendum (offline, existing recordings only): compare e01 (spin), p03 and e05 (successes) aligned on the moment each
first came within 1.0 m of the goal, and evaluate quantities defined by the MPPI 1.3.12 source:
  * GoalAngleCritic target yaw = yaw of the LAST pose of the plan handed to MPPI (data.path.yaws(last)),
    active only within 0.5 m of the true goal;   * cost of the recorded optimal trajectory vs simple counterfactuals.
No simulation, no new data.   usage: addendum_compare.py"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import *

RUNS = {"e01": "e01_m1.0", "p03": "p03_m1.0", "e05": "e05_m1.0"}
OUT = INV / "analysis" / "addendum"; OUT.mkdir(parents=True, exist_ok=True)
DT, AZ, WZMAX, W_GA = 0.05, 1.0, 1.0, 3.0           # model_dt, az_max, wz_max, GoalAngleCritic cost_weight (nav2_params.yaml)


def series(R):
    D, g = R.D, R.gt
    nav = D["cmd_nav"]; s = (nav[:, 0] >= R.tg) & (nav[:, 0] <= R.tend); A = nav[s]
    t = A[:, 0]
    x, y, yaw = (np.interp(t, g["t"], g["x"]), np.interp(t, g["t"], g["y"]), np.interp(t, g["t"], g["yaw"]))
    dist = np.hypot(x - GOAL[0], y - GOAL[1])
    # robot yaw in the odom frame (what MPPI uses): fused odometry
    O = D["odom_fused"]; yo = np.unwrap(O[:, 4]); psi_o = np.interp(t, O[:, 0], yo); wz_o = np.interp(t, O[:, 0], O[:, 6])
    ta = R.map_odom(t)[2]                                # map->odom yaw: goal yaw in odom frame = -ta
    goal_o = wrap(GOAL[2] - ta)
    T = sorted(D["paths"]["transformed"], key=lambda p: p["sim"]); Ts = np.array([p["sim"] for p in T])
    Op = sorted(D["paths"]["optimal"], key=lambda p: p["sim"]); Os = np.array([p["sim"] for p in Op])
    rows = []
    for i in range(len(t)):
        j = max(np.searchsorted(Ts, t[i], side="right") - 1, 0); p = T[j]
        psiT = p["yaw"][-1] if p["frame"] != "map" else wrap(p["yaw"][-1] - ta[i])
        k = max(np.searchsorted(Os, t[i], side="right") - 1, 0); q = Op[k]
        qy = q["yaw"] if q["frame"] != "map" else wrap(q["yaw"] - ta[i])
        rows.append(dict(t=t[i] - R.tg, dist=dist[i], e=float(wrap(GOAL[2] - yaw[i])), vx=float(np.interp(t[i], g["t"], g["vx"])), wz_gt=float(np.interp(t[i], g["t"], g["wz"])),
                         cmd_wz=A[i, 3], cmd_vx=A[i, 2], d=float(wrap(psiT - psi_o[i])), tgt_true=bool(abs(wrap(psiT - goal_o[i])) < np.radians(5)), n_tr=len(p["x"]),
                         L_tr=seg_len(p["x"], p["y"]), opt_yaw=qy, psi_o=float(psi_o[i]), psiT=float(psiT), wz_o=float(wz_o[i]),
                         pred_dyaw=float(wrap(qy[min(20, len(qy) - 1)] - qy[0])) if len(qy) > 1 else 0.0))
    return rows


def rollout(psi0, w0, wz_cmds):
    """Integrate yaw with the DiffDrive accel clamp used by MotionModel::predict (first value from measured w0)."""
    w, psi, out = w0, psi0, [psi0]
    for c in wz_cmds:
        w = min(max(c, w - AZ * DT), w + AZ * DT); psi += w * DT; out.append(psi)
    return np.array(out)


def ga_cost(yaws, target): return W_GA * float(np.mean(np.abs(wrap(yaws - target))))


def counterfactuals(r, n=56):
    psi0, w0, tgt = r["psi_o"], r["wz_o"], r["psiT"]
    res = {"recorded optimal": ga_cost(r["opt_yaw"], tgt)}
    res["brake to stop, then hold"] = ga_cost(rollout(psi0, w0, np.zeros(n))[1:], tgt)
    res["keep rotating at +wz_max"] = ga_cost(rollout(psi0, w0, np.full(n, WZMAX))[1:], tgt)
    best = None
    for kp in (1.0, 2.0, 3.0):                         # simple shortest-direction controller toward the target yaw
        yaws, w, psi = [], w0, psi0
        for _ in range(n):
            cmd = float(np.clip(kp * wrap(tgt - psi), -WZMAX, WZMAX)); w = min(max(cmd, w - AZ * DT), w + AZ * DT); psi += w * DT; yaws.append(psi)
        c = ga_cost(np.array(yaws), tgt); best = c if best is None else min(best, c)
    res["shortest-way controller to target (best of Kp 1,2,3)"] = best
    return res


if __name__ == "__main__":
    S, lines = {}, []
    for k, rid in RUNS.items():
        R = Run(INV / "runs" / rid); S[k] = (R, series(R))
    t1 = {}
    for k, (R, rows) in S.items():
        t1[k] = next(r["t"] for r in rows if r["dist"] < 1.0)
    lines.append("Aligned on tau = t - t(first within 1.0 m of goal): " + ", ".join(f"{k}: t1={t1[k]:.1f}s" for k in S))
    hdr = "tau |  dist(m) e(deg) | vx(m/s) wz_gt  cmd_wz | target-yaw offset d(deg)  [target=true goal?]  n_tr L_tr | pred dyaw/1s(deg)"
    for k, (R, rows) in S.items():
        lines += ["", f"== {k} ({R.cls['outcome']}) ==", hdr]
        for tau in range(-6, 24):
            tt = t1[k] + tau
            r = min(rows, key=lambda q: abs(q["t"] - tt))
            if abs(r["t"] - tt) > 0.2: continue
            lines.append(f"{tau:+4d} | {r['dist']:6.2f} {np.degrees(r['e']):6.0f} | {r['vx']:+.2f} {r['wz_gt']:+.2f} {r['cmd_wz']:+.2f} | {np.degrees(r['d']):6.0f}  {'yes' if r['tgt_true'] else 'NO '}  {r['n_tr']:3d} {r['L_tr']:.2f} | {np.degrees(r['pred_dyaw']):+5.0f}")
    # who does MPPI follow near the goal: the plan's end yaw (what the source says GoalAngle uses) or the true goal yaw?
    lines += ["", "Cycles within 0.5 m of the goal (GoalAngleCritic active), |cmd wz| > 0.05, error > 10 deg:",
              "run   n   follows plan-end yaw (sign cmd wz == sign d)   follows true goal yaw (sign cmd wz == sign e)   plan-end yaw is the true goal yaw"]
    for k, (R, rows) in S.items():
        sel = [r for r in rows if r["dist"] < 0.5 and abs(r["cmd_wz"]) > 0.05]
        a = [r for r in sel if abs(r["d"]) > np.radians(10)]; b = [r for r in sel if abs(r["e"]) > np.radians(10)]
        fa = np.mean([np.sign(r["cmd_wz"]) == np.sign(r["d"]) for r in a]) if a else float("nan")
        fb = np.mean([np.sign(r["cmd_wz"]) == np.sign(r["e"]) for r in b]) if b else float("nan")
        ft = np.mean([r["tgt_true"] for r in [r for r in rows if r["dist"] < 0.5]]) if any(r["dist"] < 0.5 for r in rows) else float("nan")
        lines.append(f"{k}  {len(sel):4d}  {100*fa:5.1f}% (n={len(a)})                                  {100*fb:5.1f}% (n={len(b)})                                        {100*ft:5.1f}%")
    # counterfactual GoalAngle costs on recorded states
    lines += ["", "GoalAngleCritic cost (weight 3.0 x mean |yaw - target| over the 2.8 s horizon; target = plan-end yaw) of the RECORDED optimal trajectory vs simple counterfactuals, same state:"]
    for k, tl in (("e01", (38, 40, 42, 44, 46, 48, 50)), ("p03", None), ("e05", None)):
        R, rows = S[k]
        pick = [min(rows, key=lambda q: abs(q["t"] - x)) for x in tl] if tl else [r for r in rows if r["dist"] < 0.3 and abs(r["e"]) > np.radians(20)][::40][:4]
        for r in pick:
            c = counterfactuals(r)
            lines.append(f"{k} t={r['t']:5.1f}s d={np.degrees(r['d']):5.0f}deg e={np.degrees(r['e']):5.0f}deg w0={r['wz_o']:+.2f}: " + " | ".join(f"{n}: {v:.2f}" for n, v in c.items()))
    (OUT / "aligned_comparison.txt").write_text("\n".join(lines)); print("\n".join(lines))
    import pickle; pickle.dump({k: rows for k, (R, rows) in S.items()}, open(OUT / "series.pkl", "wb"))
