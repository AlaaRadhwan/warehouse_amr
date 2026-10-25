#!/usr/bin/env python3
"""Before/after calibration comparison from the analyzers' summary CSVs.

Usage: compare_before_after.py --rot-before RUN --rot-after RUN --lin-before RUN --lin-after RUN [--out DIR]
Errors are signed percentages relative to Gazebo ground truth (or to the command).
"""
import argparse
import csv
from pathlib import Path

ROT = [("gt_yaw_slope/cmd", "true yaw rate / command"),
       ("odom_raw_twist/gt", "raw wheel odom twist / GT"),
       ("wheel_vel/gt", "wheel velocities / GT"),
       ("imu_gyro_z/gt", "IMU gyro / GT"),
       ("imu_yaw_slope/gt", "IMU orientation yaw slope / GT"),
       ("fused_twist/gt", "fused twist / GT"),
       ("fused_yaw_slope/gt", "fused pose-yaw slope / GT")]
LIN = [("gt/cmd", "true velocity / command"),
       ("odom_raw_twist/gt", "raw odom / GT"),
       ("fused_twist/gt", "fused twist / GT"),
       ("fused_pos/gt", "fused pose speed / GT")]


def load(run, name):
    return list(csv.DictReader(open(Path(run) / name)))


def worst(rows, key):
    v = [float(r[key]) for r in rows if r.get(key) not in (None, "")]
    return min(v), max(v), sum(v) / len(v)


def main():
    ap = argparse.ArgumentParser()
    for k in ("rot-before", "rot-after", "lin-before", "lin-after"):
        ap.add_argument("--" + k, required=True)
    ap.add_argument("--out", default=str(Path(__file__).parent / "reports"))
    a = ap.parse_args()
    rb, ra = load(a.rot_before, "yaw_rate_summary.csv"), load(a.rot_after, "yaw_rate_summary.csv")
    lb, la = load(a.lin_before, "linear_summary.csv"), load(a.lin_after, "linear_summary.csv")
    L = ["# Before / after kinematic calibration", "",
         f"before: rotation `{Path(a.rot_before).name}`, linear `{Path(a.lin_before).name}`",
         f"after:  rotation `{Path(a.rot_after).name}`, linear `{Path(a.lin_after).name}`", "",
         "Mean over steps, with worst-case |error| in brackets. Error = ratio - 1.", "",
         "## Rotation (6 steps: +-0.4, +-0.7, +-1.0 rad/s)", "",
         "| quantity | before | after |", "|---|---|---|"]
    def fmt(rows, key):
        lo, hi, m = worst(rows, key)
        w = max(abs(lo - 1), abs(hi - 1))
        return f"{m:.5f} ({(m-1)*100:+.3f}%, worst {w*100:.3f}%)"
    for k, n in ROT:
        L.append(f"| {n} | {fmt(rb, k)} | {fmt(ra, k)} |")
    L += ["", "## Straight line (+-0.3, +-0.6 m/s)", "", "| quantity | before | after |", "|---|---|---|"]
    for k, n in LIN:
        L.append(f"| {n} | {fmt(lb, k)} | {fmt(la, k)} |")
    L.append(f"| k_v = GT / nominal wheel | {fmt(lb, 'k_v')} | {fmt(la, 'k_v')} |")
    mx = lambda rows, k: max(abs(float(r[k])) for r in rows)
    L += ["", "## Unintended motion in straight-line windows (max |value|)", "",
          "| | before | after |", "|---|---|---|",
          f"| GT lateral velocity [m/s] | {mx(lb,'gt_lat'):.2e} | {mx(la,'gt_lat'):.2e} |",
          f"| GT yaw rate [rad/s] | {mx(lb,'gt_yaw_rate'):.2e} | {mx(la,'gt_yaw_rate'):.2e} |"]
    out = Path(a.out); out.mkdir(exist_ok=True, parents=True)
    (out / "before_after_calibration.md").write_text("\n".join(L))
    print("\n".join(L))


if __name__ == "__main__":
    main()
