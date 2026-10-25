#!/usr/bin/env python3
"""1 Hz wall-clock system monitor: real-time factor, load average, CPU use of the simulation/Nav2 processes.

Sim time comes from the header stamp of /diff_drive_controller/odom (no /clock subscription).
RTF is d(sim)/d(wall) over each interval.  CPU% is per-process utime+stime from /proc (100% = one core).
usage: sysmon.py OUT.csv
"""
import csv, os, re, signal, sys, time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

PROCS = {"gz_server": r"gz sim .*(-s|server)|gz sim -r", "gz_gui": r"gz sim -g", "controller_server": r"nav2_controller/controller_server",
         "planner_server": r"nav2_planner/planner_server", "bt_navigator": r"nav2_bt_navigator/bt_navigator", "amcl": r"nav2_amcl/amcl",
         "behavior_server": r"nav2_behaviors/behavior_server", "collision_monitor": r"nav2_collision_monitor", "velocity_smoother": r"velocity_smoother",
         "rviz2": r"rviz2", "ekf": r"ekf_node", "gz_bridge": r"parameter_bridge"}
CLK = os.sysconf("SC_CLK_TCK"); NCPU = os.cpu_count()


def find_pids():
    out = {k: [] for k in PROCS}
    for p in os.listdir("/proc"):
        if not p.isdigit(): continue
        try:
            cmd = open(f"/proc/{p}/cmdline").read().replace("\0", " ")
        except OSError: continue
        for k, rx in PROCS.items():
            if re.search(rx, cmd): out[k].append(int(p))
    return out


def ticks(pid):
    try:
        s = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
        return int(s[11]) + int(s[12])
    except (OSError, IndexError): return None


def cpu_total():
    v = list(map(int, open("/proc/stat").readline().split()[1:]))
    return sum(v), v[3] + v[4]


rclpy.init(); n = Node("sysmon"); sim = [None]
n.create_subscription(Odometry, "/diff_drive_controller/odom", lambda m: sim.__setitem__(0, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), qos_profile_sensor_data)
f = open(sys.argv[1], "w", newline=""); w = csv.writer(f)
w.writerow(["wall", "sim", "rtf", "load1", "cpu_total_pct", "mem_avail_mb", "ncpu"] + [f"cpu_{k}" for k in PROCS]); f.flush()
stop = []
signal.signal(signal.SIGINT, lambda *_: stop.append(1)); signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
pids = find_pids(); last_scan = time.time(); prev_w, prev_s = time.time(), sim[0]; prev_t = {p: ticks(p) for v in pids.values() for p in v}; prev_c = cpu_total()
while not stop:
    t_end = time.time() + 1.0
    while time.time() < t_end and not stop: rclpy.spin_once(n, timeout_sec=0.05)
    now = time.time()
    if now - last_scan > 5: pids = find_pids(); last_scan = now
    c = cpu_total(); dtot, didle = c[0] - prev_c[0], c[1] - prev_c[1]; prev_c = c
    dw = now - prev_w; per = []
    for k, ps in pids.items():
        tot = 0.0
        for p in ps:
            t = ticks(p)
            if t is not None and prev_t.get(p) is not None: tot += (t - prev_t[p]) / CLK / dw * 100
            prev_t[p] = t
        per.append(round(tot, 1))
    rtf = (sim[0] - prev_s) / dw if sim[0] is not None and prev_s is not None else ""
    prev_w, prev_s = now, sim[0]
    mem = next((int(l.split()[1]) // 1024 for l in open("/proc/meminfo") if l.startswith("MemAvailable")), "")
    w.writerow([f"{now:.3f}", sim[0] if sim[0] is not None else "", round(rtf, 3) if rtf != "" else "", os.getloadavg()[0],
                round(100 * (dtot - didle) / dtot, 1) if dtot else "", mem, NCPU] + per); f.flush()
f.close()
