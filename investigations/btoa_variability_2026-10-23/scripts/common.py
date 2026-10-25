"""Shared loaders and geometry for the B->A variability analysis. numpy/matplotlib only."""
import csv, gzip, json, math, pickle
from pathlib import Path

import numpy as np

INV = Path(__file__).resolve().parents[1]
GOAL = (0.099, -0.175, 0.0)                      # B->A staging pose (map)
FOOT = np.array([[0.5, 0.37], [0.5, -0.37], [-0.5, -0.37], [-0.5, 0.37]])   # nav footprint (nav2_params)
WZ_MAX, VX_MAX, VX_MIN, PRUNE = 1.0, 0.8, -0.35, 2.5   # MPPI limits / prune_distance from nav2_params.yaml
wrap = lambda a: (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi

# colour tokens (reference palette instance, light surface): arm = hue, outcome = marker/line style where needed
SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e1"
ARM_COL = {1.0: "#eb6834", 0.92754: "#2a78d6"}   # uncalibrated = orange, calibrated = blue


def style(plt):
    plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "text.color": INK, "font.size": 9, "axes.titlesize": 10, "axes.titlelocation": "left"})


def clean(ax):
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)


def yaw_of_q(qx, qy, qz, qw): return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def load_gt(run):
    rows, seen = [], set()
    for r in list(csv.reader(open(Path(run) / "gt.csv")))[1:]:
        try: v = [float(q) for q in r]
        except ValueError: continue
        if v[1] in seen: continue
        seen.add(v[1]); rows.append(v)
    a = np.array(rows)
    t, x, y = a[:, 1], a[:, 2], a[:, 3]
    yaw = np.unwrap(yaw_of_q(a[:, 5], a[:, 6], a[:, 7], a[:, 8]))
    j = np.searchsorted(t, t - 0.5)                    # 0.5 s finite-difference window
    j = np.minimum(j, np.arange(len(t)) - 1).clip(0)
    dt = np.maximum(t - t[j], 1e-6)
    v = np.hypot(x - x[j], y - y[j]) / dt
    vs = ((x - x[j]) * np.cos(yaw) + (y - y[j]) * np.sin(yaw)) / dt   # signed speed along heading
    wz = (yaw - yaw[j]) / dt
    return {"t": t, "x": x, "y": y, "yaw": yaw, "v": v, "vx": vs, "wz": wz, "wall": a[:, 0]}


class Run:
    def __init__(self, run):
        self.dir = Path(run); self.id = self.dir.name
        self.cr = json.loads((self.dir / "client_result.json").read_text()) if (self.dir / "client_result.json").exists() else None
        self.cls = json.loads((self.dir / "classification.json").read_text()) if (self.dir / "classification.json").exists() else None
        self.mult = float(json.loads((self.dir / "variant/variant.json").read_text())["multiplier"])
        self.gt = load_gt(self.dir) if (self.dir / "gt.csv").exists() else None
        p = self.dir / "extracted.pkl.gz"
        self.D = pickle.load(gzip.open(p, "rb")) if p.exists() else None
        self.sm = self._sysmon()
        self.tg = self.cr["sim_goal_sent"] if self.cr else None
        self.tend = self.tg + self.cr.get("sim_duration_s", 0.0) if self.cr else None

    def _sysmon(self):
        f = self.dir / "sysmon.csv"
        if not f.exists(): return None
        rows = list(csv.reader(open(f))); h = rows[0]; out = {k: [] for k in h}
        for r in rows[1:]:
            for k, v in zip(h, r):
                try: out[k].append(float(v))
                except ValueError: out[k].append(float("nan"))
        return {k: np.array(v) for k, v in out.items()}

    def rel(self, t): return np.asarray(t) - self.tg
    def wall2sim(self, w):
        m = self.D["wall2sim"]; return np.interp(w, m[:, 0], m[:, 1])
    def in_goal(self, t): return (t >= self.tg) & (t <= self.tend)
    def gt_at(self, t, key): return np.interp(t, self.gt["t"], self.gt[key])
    def gt_yaw_at(self, t): return np.interp(t, self.gt["t"], self.gt["yaw"])
    def map_odom(self, t):
        m = self.D["map_odom"]
        if len(m) == 0: return 0.0, 0.0, 0.0
        yaw = np.unwrap(m[:, 4])
        return np.interp(t, m[:, 0], m[:, 2]), np.interp(t, m[:, 0], m[:, 3]), np.interp(t, m[:, 0], yaw)
    def to_map(self, frame, t, x, y, yaw=None):
        """Express points given in `frame` in the map frame (map->odom from /tf at time t)."""
        if frame == "map": return (x, y, yaw) if yaw is not None else (x, y)
        tx, ty, ta = self.map_odom(t); c, s = np.cos(ta), np.sin(ta)
        X, Y = c * x - s * y + tx, s * x + c * y + ty
        return (X, Y, wrap(yaw + ta)) if yaw is not None else (X, Y)


def seg_len(x, y): return float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if len(x) > 1 else 0.0


def cusp_indices(x, y, yaw, eps=1e-4):
    """Indices (into the pose arrays) where the travel direction flips between forward and reverse.
    Direction of each segment = sign of its projection on the pose heading; rotate-in-place segments are skipped."""
    if len(x) < 3: return np.array([], dtype=int)
    dx, dy = np.diff(x), np.diff(y); seg = np.hypot(dx, dy)
    d = np.sign(dx * np.cos(yaw[:-1]) + dy * np.sin(yaw[:-1])); d[seg < eps] = 0
    idx = np.nonzero(d)[0]
    if len(idx) < 2: return np.array([], dtype=int)
    ch = np.nonzero(d[idx][1:] != d[idx][:-1])[0]
    return idx[ch + 1]


def path_stats(p):
    x, y, yaw = p["x"], p["y"], p["yaw"]; n = len(x)
    if n < 2: return {"n": n}
    L = seg_len(x, y); straight = float(np.hypot(x[-1] - x[0], y[-1] - y[0]))
    d = np.hypot(np.diff(x), np.diff(y)); mov = d > 1e-4
    tang = np.arctan2(np.diff(y), np.diff(x))[mov]
    turn = float(np.sum(np.abs(wrap(np.diff(tang))))) if len(tang) > 1 else 0.0
    rot_in_place = int(np.sum(~mov))
    cu = cusp_indices(x, y, yaw)
    rev_len = float(np.sum(d[(np.diff(x) * np.cos(yaw[:-1]) + np.diff(y) * np.sin(yaw[:-1])) < 0]))
    return {"n": n, "length_m": L, "straight_m": straight, "length_over_straight": L / straight if straight > 0.05 else float("inf"),
            "total_tangent_turn_rad": turn, "rotate_in_place_poses": rot_in_place, "cusps": int(len(cu)), "reverse_length_m": rev_len,
            "end_xy": [float(x[-1]), float(y[-1])], "end_yaw": float(yaw[-1])}
