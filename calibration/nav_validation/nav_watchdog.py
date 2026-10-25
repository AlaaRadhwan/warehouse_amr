#!/usr/bin/env python3
"""Test-harness watchdog for pathological rotation / lack of progress during a NavigateToPose goal.

Pure logic (no ROS imports) so it can drive the live client (nav_goal.py) and replay recorded bags
(replay_watchdog.py) with identical results.  It only *observes*; Nav2, MPPI and the progress/goal
checkers are never touched.

Inputs per sample (sim time): final /cmd_vel (vx, wz), Nav2 feedback (current pose x/y/yaw in the
map frame, distance_remaining, number_of_recoveries).  Every rule is a conjunction of
"rotating a lot" AND "not getting closer to the goal", so a legitimate in-place heading correction
(bounded rotation, then progress or goal reached) never trips it, while a spin that goes nowhere does.

Progress over a window = max(path progress, straight-line progress) of NET approach, where
  path progress    = mean distance_remaining over the first 1 s - mean over the last 1 s of the window
  straight progress= the same with the straight-line distance to the goal
Net (not best-case) progress is used because a spinning robot jitters around and drifts slowly; the min
over a window would credit it with progress it did not make.  Taking the larger of the two measures is
lenient to replanning jumps in distance_remaining and AMCL jitter, i.e. biased against false aborts.
The rotation rules judge progress *per turn of rotation* (m/turn), so a robot that drifts slowly while
spinning is still caught, and a robot that rotates a little while genuinely travelling is not.

Rules (all thresholds in nav_watchdog.yaml / Config):
  saturated_spin   >= sat_fraction of the last sat_window s has |wz| >= sat_wz with one sign
                   AND progress per turn < sat_min_m_per_turn
  rotation_no_progress   accumulated |d yaw| over rot_window s >= rot_max_turns*2pi
                   AND progress per turn < rot_min_m_per_turn
  dithering        >= dither_reversals sign flips of |wz|>dither_wz within dither_window s
                   AND progress < dither_min_progress
  stalled          progress < stall_min_progress over stall_window s (any motion, incl. none)
  settle_timeout   inside goal_xy_tol of the goal for settle_timeout s without the goal succeeding
  recoveries       Nav2 reports >= max_recoveries recovery attempts
  timeout          goal older than timeout s
"""
import math
from collections import deque
from dataclasses import dataclass, fields, asdict
from pathlib import Path
from typing import Optional


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Config:
    wz_max: float = 1.0               # MPPI wz_max in nav2_params.yaml (documentation + sat_wz default)
    sat_wz_frac: float = 0.9          # saturated = |wz| >= frac * wz_max
    sat_window: float = 8.0
    sat_fraction: float = 0.8
    sat_min_m_per_turn: float = 2.0
    rot_window: float = 20.0
    rot_max_turns: float = 1.0
    rot_min_m_per_turn: float = 2.0
    dither_wz: float = 0.3
    dither_window: float = 20.0
    dither_reversals: int = 6
    dither_min_progress: float = 0.3
    stall_window: float = 45.0
    stall_min_progress: float = 0.3
    goal_xy_tol: float = 0.25         # = StoppedGoalChecker xy_goal_tolerance
    settle_timeout: float = 30.0
    max_recoveries: int = 4
    timeout: float = 240.0
    grace: float = 3.0                # ignore the first seconds after the goal (planner start-up)
    min_dt: float = 0.1               # sample decimation

    @property
    def sat_wz(self):
        return self.sat_wz_frac * self.wz_max

    @classmethod
    def load(cls, path=None, **overrides):
        d = {}
        if path:
            import yaml
            d = yaml.safe_load(Path(path).read_text()) or {}
        d.update({k: v for k, v in overrides.items() if v is not None})
        known = {f.name for f in fields(cls)}
        bad = set(d) - known
        if bad:
            raise ValueError(f"unknown watchdog config keys: {sorted(bad)}")
        return cls(**d)


@dataclass
class Trigger:
    rule: str
    t: float
    reason: str
    metrics: dict


class Watchdog:
    """Call update() with each sample; returns a Trigger the first time any rule fires."""

    def __init__(self, cfg: Config, goal_xy):
        self.cfg, self.goal = cfg, goal_xy
        self.s = deque()              # (t, d_rem, d_euc, yaw, wz, vx, rec)
        self.t0 = None
        self.last_t = None
        self.in_tol_since = None
        # peak of each rule's gated margin; for the m/turn rules and stalled a value > 0 means "would trip"
        self.peak = {k: -9.0 for k in ("saturated_spin", "rotation_no_progress", "dithering",
                                      "stalled", "settle_timeout", "recoveries")}
        self.ctx = dict(max_sat_fraction=0.0, max_turns_in_rot_window=0.0, max_reversals=0, max_abs_wz=0.0)
        self.yaw_acc = deque()        # (t, |d yaw|)
        self.trigger: Optional[Trigger] = None

    # ---- helpers
    def _window(self, t, w):
        return [x for x in self.s if x[0] >= t - w]

    @staticmethod
    def _progress(win):
        """Net approach to the goal over the window [m], from 1 s averages at both ends."""
        if len(win) < 2:
            return 0.0
        t0, t1 = win[0][0], win[-1][0]
        head = [x for x in win if x[0] <= t0 + 1.0]
        tail = [x for x in win if x[0] >= t1 - 1.0]
        m = lambda xs, i: sum(x[i] for x in xs) / len(xs)
        return max(m(head, 1) - m(tail, 1), m(head, 2) - m(tail, 2))

    def _turns(self, t, w):
        return sum(a for tt, a in self.yaw_acc if tt >= t - w) / (2 * math.pi)

    def _covers(self, win, w):
        return bool(win) and (win[-1][0] - win[0][0]) >= 0.98 * w

    def update(self, t, x, y, yaw, wz, vx, dist_remaining, recoveries=0) -> Optional[Trigger]:
        c = self.cfg
        if self.t0 is None:
            self.t0 = t
        if self.last_t is not None and t - self.last_t < c.min_dt:
            return None
        d_euc = math.hypot(x - self.goal[0], y - self.goal[1])
        if self.s:
            self.yaw_acc.append((t, abs(wrap(yaw - self.s[-1][3]))))
        self.last_t = t
        self.s.append((t, dist_remaining, d_euc, yaw, wz, vx, recoveries))
        horizon = max(c.sat_window, c.rot_window, c.dither_window, c.stall_window) + 1
        while self.s and self.s[0][0] < t - horizon:
            self.s.popleft()
        while self.yaw_acc and self.yaw_acc[0][0] < t - horizon:
            self.yaw_acc.popleft()
        if self.trigger or t - self.t0 < c.grace:
            return None

        self.ctx['max_abs_wz'] = max(self.ctx['max_abs_wz'], abs(wz))
        found = []

        # 1. saturated rotation without progress
        win = self._window(t, c.sat_window)
        if self._covers(win, c.sat_window):
            pos = sum(1 for s in win if s[4] >= c.sat_wz) / len(win)
            neg = sum(1 for s in win if s[4] <= -c.sat_wz) / len(win)
            frac, prog = max(pos, neg), self._progress(win)
            self.ctx['max_sat_fraction'] = max(self.ctx['max_sat_fraction'], frac)
            ppt = prog / max(self._turns(t, c.sat_window), 0.25)
            if frac >= c.sat_fraction:
                self.peak["saturated_spin"] = max(self.peak["saturated_spin"], c.sat_min_m_per_turn - ppt)
                if ppt < c.sat_min_m_per_turn:
                    found.append(("saturated_spin",
                                  f"|wz|>={c.sat_wz:.2f} for {frac*100:.0f}% of {c.sat_window:.0f}s, "
                                  f"{prog:.2f} m progress ({ppt:.2f} m/turn)", dict(fraction=frac, progress=prog)))

        # 2. accumulated rotation without progress
        win = self._window(t, c.rot_window)
        if self._covers(win, c.rot_window):
            turns, prog = self._turns(t, c.rot_window), self._progress(win)
            self.ctx['max_turns_in_rot_window'] = max(self.ctx['max_turns_in_rot_window'], turns)
            if turns >= c.rot_max_turns:
                ppt = prog / turns
                self.peak["rotation_no_progress"] = max(self.peak["rotation_no_progress"],
                                                        c.rot_min_m_per_turn - ppt)
                if ppt < c.rot_min_m_per_turn:
                    found.append(("rotation_no_progress",
                                  f"{turns:.2f} turns in {c.rot_window:.0f}s, {prog:.2f} m progress "
                                  f"({ppt:.2f} m/turn)", dict(turns=turns, progress=prog)))

        # 3. direction-reversal dithering
        win = self._window(t, c.dither_window)
        if self._covers(win, c.dither_window):
            signs = [1 if s[4] > c.dither_wz else -1 for s in win if abs(s[4]) > c.dither_wz]
            flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
            prog = self._progress(win)
            self.ctx['max_reversals'] = max(self.ctx['max_reversals'], flips)
            if prog < c.dither_min_progress:
                self.peak["dithering"] = max(self.peak["dithering"], flips)   # compare with dither_reversals
                if flips >= c.dither_reversals:
                    found.append(("dithering", f"{flips} wz sign reversals in {c.dither_window:.0f}s "
                                  f"with {prog:.2f} m progress", dict(reversals=flips, progress=prog)))

        # 4. no meaningful progress at all
        win = self._window(t, c.stall_window)
        if self._covers(win, c.stall_window):
            prog = self._progress(win)
            self.peak["stalled"] = max(self.peak["stalled"], c.stall_min_progress - prog)  # >0 = would trip
            # a robot already at the goal position and settling is handled by settle_timeout
            if prog < c.stall_min_progress and d_euc > c.goal_xy_tol:
                found.append(("stalled", f"{prog:.2f} m progress in {c.stall_window:.0f}s",
                              dict(progress=prog)))

        # 5. settling inside the position tolerance for too long
        if d_euc <= c.goal_xy_tol:
            self.in_tol_since = self.in_tol_since if self.in_tol_since is not None else t
            dwell = t - self.in_tol_since
            self.peak["settle_timeout"] = max(self.peak["settle_timeout"], dwell)
            if dwell >= c.settle_timeout:
                found.append(("settle_timeout", f"inside {c.goal_xy_tol} m of goal for {dwell:.0f}s "
                              "without success", dict(dwell=dwell)))
        else:
            self.in_tol_since = None

        # 6. recoveries, 7. timeout
        self.peak["recoveries"] = max(self.peak["recoveries"], recoveries)
        if recoveries >= c.max_recoveries:
            found.append(("recoveries", f"{recoveries} recovery attempts", dict(recoveries=recoveries)))
        if t - self.t0 >= c.timeout:
            found.append(("timeout", f"goal older than {c.timeout:.0f}s", dict(elapsed=t - self.t0)))

        if found:
            rule, reason, m = found[0]
            self.trigger = Trigger(rule, t, reason + (f" [also: {', '.join(f[0] for f in found[1:])}]"
                                                      if len(found) > 1 else ""), m)
            return self.trigger
        return None

    def summary(self):
        return dict(config=asdict(self.cfg), peak_gated_metric=self.peak, context_peaks=self.ctx,
                    trigger=None if not self.trigger else asdict(self.trigger))
