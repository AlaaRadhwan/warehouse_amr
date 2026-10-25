#!/usr/bin/env python3
"""Synthetic scenarios for nav_watchdog.py (no ROS needed): python3 test_nav_watchdog.py"""
import math
from nav_watchdog import Config, Watchdog

GOAL = (10.0, 0.0)


def run(gen, T=120.0, dt=0.05, cfg=None):
    wd = Watchdog(cfg or Config(), GOAL)
    t = 0.0
    while t < T:
        x, y, yaw, wz, vx, rec = gen(t)
        d = math.hypot(x - GOAL[0], y - GOAL[1])
        tr = wd.update(t, x, y, yaw, wz, vx, d, rec)
        if tr:
            return tr
        t += dt
    return None


def cruise(t):                    # drive straight at 0.5 m/s
    return min(0.5 * t, 10.0), 0, 0, 0, 0.5, 0

def inplace_turn_then_drive(t):   # legit: pi rad at 1.0 rad/s (3.14 s), then drive
    if t < 3.2: return 0, 0, min(t, math.pi), 1.0, 0, 0
    return min(0.5 * (t - 3.2), 10.0), 0, math.pi, 0, 0.5, 0

def two_cusps_goal_correction(t):  # drive, arrive, correct heading 171 deg at 0.36 rad/s, then stop in tolerance
    if t < 20: return 0.5 * t, 0, 0, 0, 0.5, 0
    yaw = min(0.36 * (t - 20), 2.98)
    return 10.0, 0, yaw, 0.36 if yaw < 2.98 else 0, 0, 0

def pinned_spin_far(t):           # control: wz=1 pinned, drifts 0.05 m/s
    return 0.05 * t, 0, t * 1.0, 1.0, 0.1, 0

def slow_spin_far(t):             # non-saturated but continuous rotation, 0.5 rad/s, no progress
    return 0, 0, 0.5 * t, 0.5, 0, 0

def dither(t):                    # left/right wz flips every 2 s, small net motion
    return 0, 0, 0.1 * math.sin(t), 0.5 * (1 if int(t / 2) % 2 == 0 else -1), 0, 0

def stuck(t):
    return 0, 0, 0, 0, 0, 0

def settle_forever(t):            # inside tolerance, still wobbling, never succeeds
    return 9.9, 0, 0.2 * math.sin(t), 0.1, 0, 0

def recoveries(t):
    return 0.1 * t, 0, 0, 0, 0.1, 5 if t > 20 else 0


CASES = [("cruise", cruise, None), ("inplace_turn_then_drive", inplace_turn_then_drive, None),
         ("two_cusps_goal_correction", two_cusps_goal_correction, None),
         ("pinned_spin_far", pinned_spin_far, "saturated_spin|rotation_no_progress"),
         ("slow_spin_far", slow_spin_far, "rotation_no_progress"),
         ("dither", dither, "dithering"), ("stuck", stuck, "stalled"),
         ("settle_forever", settle_forever, "settle_timeout"), ("recoveries", recoveries, "recoveries")]

if __name__ == "__main__":
    bad = 0
    for name, gen, expect in CASES:
        tr = run(gen, T=45 if expect is None else (80 if name == "stuck" else 60))  # legit cases end (goal succeeds) before settle_timeout
        got = tr.rule if tr else None
        ok = (got is None) if expect is None else (got is not None and got in expect.split("|"))
        bad += not ok
        print(f"{'PASS' if ok else 'FAIL'} {name:28s} -> {got} {'t=%.1f' % tr.t if tr else ''} {tr.reason if tr else ''}")
    raise SystemExit(bad)
