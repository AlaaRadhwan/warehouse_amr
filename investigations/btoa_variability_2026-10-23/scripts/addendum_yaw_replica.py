#!/usr/bin/env python3
"""REDUCED-MODEL consistency check (not a reproduction of the full system, not proof of mechanism).
Yaw-only re-implementation of the nav2_mppi_controller 1.3.12 optimizer mechanics, taken from the upstream source
(optimizer.cpp, noise_generator.cpp, motion_models.hpp, utils.hpp, goal_angle_critic.cpp):
  fixed noise set (regenerate_noises:false), noised controls = sequence + noise (NOT clipped to wz_max), rollout wz clamped to
  +-az_max*dt around the PREVIOUS value starting from the MEASURED yaw rate, GoalAngle cost = 3.0*mean|yaw-target| over the horizon,
  regulariser gamma/std^2*sum(seq*(cwz-seq)), softmax with temperature 0.3, new sequence = weighted mean of the clamped noised
  controls, then clip to +-wz_max and accel-limit along the sequence, Savitzky-Golay 9-point smoothing with 4-sample history,
  output = sequence[1], shift by one.  Only GoalAngle is scored (robot assumed at the goal position: GoalCritic/Cost/Constraint
  do not depend on yaw there); PathFollow/PathAngle are off inside 0.1/0.5 m.  Plant: yaw rate = gain * command delayed 0.3 s.
usage: addendum_yaw_replica.py"""
import numpy as np
B, N, DT, AZ, WZMAX, STD, TEMP, GAMMA, W_GA = 2000, 56, 0.05, 1.0, 1.0, 0.4, 0.3, 0.015, 3.0
SG = np.array([-21, 14, 39, 54, 59, 54, 39, 14, -21], float) / 231.0
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi


def sg_filter(seq, hist):
    n = len(seq) - 1; init = seq.copy(); out = seq.copy()
    pm4, pm3, pm2, pm1 = hist; pt, p1, p2, p3, p4 = init[0], init[1], init[2], init[3], init[4]
    for idx in range(n):
        out[idx] = np.dot(SG, [pm4, pm3, pm2, pm1, pt, p1, p2, p3, p4])
        pm4, pm3, pm2, pm1, pt, p1, p2, p3 = pm3, pm2, pm1, pt, p1, p2, p3, p4
        p4 = init[idx + 5] if idx + 5 < n else init[n]
    return out


def optimizer_cycle(psi, w0, seq, hist, noise, target):
    cwz = seq[None, :] + noise
    wz = np.empty((B, N)); wz[:, 0] = w0; last = np.full(B, w0)
    for j in range(1, N):
        c = np.clip(cwz[:, j - 1], last - AZ * DT, last + AZ * DT); cwz[:, j - 1] = c; wz[:, j] = c; last = c
    yaws = psi + np.cumsum(wz * DT, axis=1)
    cost = W_GA * np.mean(np.abs(wrap(yaws - target)), axis=1) + GAMMA / STD ** 2 * np.sum(seq[None, :] * (cwz - seq[None, :]), axis=1)
    wgt = np.exp(-(cost - cost.min()) / TEMP); wgt /= wgt.sum()
    new = np.sum(cwz * wgt[:, None], axis=0)
    new = np.clip(new, -WZMAX, WZMAX); lastv = new[0]
    for i in range(1, N):
        new[i] = np.clip(new[i], lastv - AZ * DT, lastv + AZ * DT); lastv = new[i]
    new = sg_filter(new, hist)
    hist = [hist[1], hist[2], hist[3], new[1]]
    cmd = new[1]
    new = np.roll(new, -1); new[-1] = new[-2]
    return cmd, new, hist, float(np.min(cost)), float(np.max(wgt))


def simulate(seed, gain, psi0, w0, seq0, target=0.0, cycles=400, dt_sim=0.031, delay_s=0.3, tgt_fn=None):
    rng = np.random.RandomState(seed); noise = rng.normal(0, STD, (B, N)).astype(np.float64)
    seq = np.full(N, seq0, float) if np.isscalar(seq0) else seq0.copy(); hist = [seq0 if np.isscalar(seq0) else seq0[0]] * 4
    psi, w = psi0, w0; buf = [w0 / gain] * int(round(delay_s / dt_sim)); log = []
    for c in range(cycles):
        tgt = target if tgt_fn is None else tgt_fn(c * dt_sim)
        cmd, seq, hist, _, _ = optimizer_cycle(psi, w, seq, hist, noise, tgt)
        buf.append(cmd); w = gain * buf.pop(0); psi += w * dt_sim
        log.append((c * dt_sim, cmd, w, float(wrap(target - psi))))
    return np.array(log)


def summarize(name, gain, psi0, w0, seq0, seeds, cycles=400, **kw):
    fin, stopped, passes = [], 0, []
    for s in seeds:
        L = simulate(s, gain, psi0, w0, seq0, cycles=cycles, **kw)
        tail = L[-int(1.5 / 0.031):]
        ok = np.all(np.abs(tail[:, 2]) < 0.15) and np.all(np.abs(tail[:, 3]) < 0.2)
        stopped += ok; fin.append(L[-1, 1]); passes.append(np.sum(np.abs(np.diff(np.sign(L[:, 3]))) > 1))   # goal-heading crossings
    print(f"{name:62s} converged {stopped:2d}/{len(seeds)} | final cmd median {np.median(fin):+.2f} (min {np.min(fin):+.2f}, max {np.max(fin):+.2f}) | goal-heading crossings median {np.median(passes):.0f}")
    return stopped


if __name__ == "__main__":
    seeds = range(20)
    print("R1  persistence from the RECORDED locked state (e01 at t~45 s): robot yaw +27 deg (goal needs 27 deg CW), sequence pinned at +1.0, 400 cycles = 12.4 sim-s")
    summarize("R1a plant gain 1.076, measured wz 1.079", 1.076, np.radians(27), 1.079, 1.0, seeds)
    summarize("R1b plant gain 0.980, measured wz 0.980", 0.980, np.radians(27), 0.980, 1.0, seeds)
    print("\nR2  same, but the sequence starts at 0 (no pinned history) -> does the optimizer converge from rest-ish at the same pose?")
    summarize("R2a plant gain 1.076, measured wz 0.0", 1.076, np.radians(27), 0.0, 0.0, seeds)
    print("\nR3  entry from the recorded 'forward with 145 deg heading error' state (goal needs 145 deg CCW), sequence 0.3, measured wz 0.3, 700 cycles = 21.7 sim-s")
    summarize("R3a plant gain 1.076", 1.076, np.radians(-145), 0.3, 0.3, seeds, cycles=700)
    summarize("R3b plant gain 0.980", 0.980, np.radians(-145), 0.3, 0.3, seeds, cycles=700)
    print("\nR4  sensitivity of the pinned state to the measured-rate offset (sequence +1.0, pose +27 deg): gain 1.00/1.03/1.076")
    for g in (1.00, 1.03, 1.076):
        summarize(f"R4  gain {g:.3f}, measured wz {g:.3f}", g, np.radians(27), g, 1.0, seeds)
