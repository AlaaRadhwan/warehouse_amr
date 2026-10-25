#!/usr/bin/env python3
"""Combine rotation + straight-line runs into effective differential-drive kinematics.

  k_v = actual / nominal longitudinal velocity      (linear run)
  k_w = actual / commanded yaw rate                 (rotation run; raw odometry reports the command)
  r_eff  = r_nom * k_v
  ws_eff = ws_nom * k_v / k_w

Cross-checks use the recorded wheel angular velocities and Gazebo ground truth directly:
  r_eff  = v_GT / mean(w_L, w_R)
  ws_eff = r_eff * (w_R - w_L) / w_GT
Optional diagnostic runs (scratch models with other wheel widths, joint spacing unchanged)
test whether ws_eff follows ws_nom - wheel_width.

Usage:
  identify_kinematics.py --rotation RUN --linear RUN [--diag RUN:WIDTH ...] [--out DIR]
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from analyze_rotation_test import read_csv, slope, tcol, yaw


def window_stats(run, mode, settle=2.5, trim=0.5):
    d = {k: read_csv(run / f"{k}.csv") for k in ("cmd", "gt", "joints")}
    meta = json.loads((run / "metadata.json").read_text())
    p = meta["controller_params"]
    r_nom, ws_nom = p["wheel_radius"], p["wheel_separation"] * p["wheel_separation_multiplier"]
    cmd = d["cmd"]
    hold = cmd["phase"] == "hold"
    tgt = cmd["target_wz" if mode == "rot" else "target_v"]
    out = []
    for idx in sorted(set(cmd["step"][hold])):
        m = hold & (cmd["step"] == idx)
        lo, hi = cmd["t"][m].min() + settle, cmd["t"][m].max() - trim
        g, tg = d["gt"], tcol(d["gt"])
        mg = (tg >= lo) & (tg <= hi)
        psi = np.unwrap(yaw(g.where(mg)))
        w_gt, _ = slope(tg[mg], psi)
        vx, _ = slope(tg[mg], g["x"][mg]); vy, _ = slope(tg[mg], g["y"][mg])
        v_gt = vx * math.cos(psi.mean()) + vy * math.sin(psi.mean())
        j, tj = d["joints"], tcol(d["joints"])
        mj = (tj >= lo) & (tj <= hi)
        wl, wr = j["left_vel"][mj].mean(), j["right_vel"][mj].mean()
        out.append(dict(target=float(tgt[m][0]), w_gt=w_gt, v_gt=v_gt, wl=wl, wr=wr))
    return out, r_nom, ws_nom


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rotation", required=True)
    ap.add_argument("--linear", required=True)
    ap.add_argument("--diag", nargs="*", default=[], help="RUN_DIR:WHEEL_WIDTH_M")
    ap.add_argument("--out", default=str(Path(__file__).parent / "reports"))
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)

    rot, r_nom, ws_nom = window_stats(Path(a.rotation), "rot")
    lin, r2, _ = window_stats(Path(a.linear), "lin")
    assert abs(r_nom - r2) < 1e-9

    # ---- k_w, k_v (primary estimates)
    kw_each = np.array([s["w_gt"] / s["target"] for s in rot])
    kv_each = np.array([s["v_gt"] / (r_nom * (s["wl"] + s["wr"]) / 2) for s in lin])
    kw, kv = kw_each.mean(), kv_each.mean()
    r_eff, ws_eff = r_nom * kv, ws_nom * kv / kw

    # ---- independent cross-checks from wheel rates + ground truth
    r_chk = np.array([s["v_gt"] / ((s["wl"] + s["wr"]) / 2) for s in lin])
    ws_chk = np.array([r_eff * (s["wr"] - s["wl"]) / s["w_gt"] for s in rot])
    # the commanded-rate path: wheel speeds are ws_nom*cmd/(2 r_nom) -> body rate r_eff*dOmega/ws_eff
    w_pred = np.array([r_eff * (s["wr"] - s["wl"]) / ws_eff for s in rot])
    w_pred_err = w_pred / np.array([s["w_gt"] for s in rot]) - 1

    diag = []
    for item in a.diag:
        run, width = item.rsplit(":", 1)
        st, rn, wsn = window_stats(Path(run), "rot")
        kwd = np.mean([s["w_gt"] / s["target"] for s in st])
        diag.append(dict(width=float(width), kw=kwd, ws_eff=wsn / kwd, run=Path(run).name))
    base_width = 0.05
    diag.append(dict(width=base_width, kw=kw, ws_eff=ws_eff, run=Path(a.rotation).name))
    diag.sort(key=lambda x: x["width"])

    # ---- report
    L = ["# AMR effective-kinematics identification", "",
         f"Rotation run: `{Path(a.rotation).name}`  |  Linear run: `{Path(a.linear).name}`", "",
         "## Result", "",
         f"| quantity | value |", "|---|---|",
         f"| nominal wheel radius | {r_nom:.4f} m |",
         f"| nominal wheel separation (controller) | {ws_nom:.4f} m |",
         f"| k_v (actual/nominal longitudinal) | {kv:.5f}  (per test: {', '.join(f'{x:.5f}' for x in kv_each)}) |",
         f"| k_w (actual/commanded yaw rate) | {kw:.5f}  (per test: {', '.join(f'{x:.5f}' for x in kw_each)}) |",
         f"| **effective radius** = r k_v | **{r_eff:.5f} m** |",
         f"| **effective separation** = ws k_v / k_w | **{ws_eff:.5f} m** |",
         f"| equivalent `wheel_separation_multiplier` (not applied) | {ws_eff/ws_nom:.5f} |", "",
         "## Cross-checks (recorded wheel rates + Gazebo ground truth)", "",
         f"- r_eff from v_GT / mean wheel rate, per straight test: "
         f"{', '.join(f'{x:.5f}' for x in r_chk)} m",
         f"- ws_eff from r_eff*(wR-wL)/w_GT, per rotation test: "
         f"{', '.join(f'{x:.5f}' for x in ws_chk)} m",
         f"- body yaw rate predicted from the identified model vs ground truth, per rotation test: "
         f"{', '.join(f'{x*100:+.4f}%' for x in w_pred_err)}",
         "- wheel-link centre separation read from Gazebo ground truth: 0.69000 m (nominal geometry is realised exactly)",
         "", "## Wheel-width diagnostic (scratch models, joint spacing fixed at 0.690 m)", "",
         "| wheel width [m] | k_w | ws_eff [m] | 0.69 - width [m] | residual [m] | run |", "|---|---|---|---|---|---|"]
    for dd in diag:
        L.append(f"| {dd['width']:.2f} | {dd['kw']:.4f} | {dd['ws_eff']:.4f} | {0.69-dd['width']:.4f} | "
                 f"{dd['ws_eff']-(0.69-dd['width']):+.4f} | {dd['run']} |")
    (out / "kinematic_identification_numbers.md").write_text("\n".join(L))
    print("\n".join(L))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    ax[0].bar(["k_v"] * 0 + [f"{s['target']:+.1f} m/s" for s in lin], kv_each)
    ax[0].axhline(1, color="k", lw=.8); ax[0].set_ylim(.95, 1.1); ax[0].set_title("k_v per straight test")
    ax[1].bar([f"{s['target']:+.1f}" for s in rot], kw_each, color="C1")
    ax[1].axhline(1, color="k", lw=.8); ax[1].set_ylim(.95, 1.1)
    ax[1].set_title("k_w per rotation test"); ax[1].set_xlabel("rad/s")
    wd = np.linspace(0, .12, 50)
    ax[2].plot(wd, .69 - wd, "k--", label="0.69 - wheel width")
    ax[2].plot([d_["width"] for d_ in diag], [d_["ws_eff"] for d_ in diag], "o", label="measured")
    ax[2].axhline(.69, color="gray", lw=.8, label="nominal 0.69")
    ax[2].set_xlabel("wheel width [m]"); ax[2].set_ylabel("effective separation [m]")
    ax[2].set_title("effective separation vs wheel width"); ax[2].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out / "kinematic_identification.png", dpi=120)


if __name__ == "__main__":
    main()
