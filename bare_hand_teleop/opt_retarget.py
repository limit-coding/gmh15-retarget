"""Optimisation retargeting for GMH15 on the vendor URDF (AnyTeleop vector retargeting + DexPilot
pinch terms), adapted to the GMH15's 7 controllable DOF.

Variables x (7): s_index, s_middle, s_ring, s_pinky in [0,1] (finger closure along a joint path)
                 + thumb joint_0/1/2 in degrees.
Each finger has ONE tendon motor driving MCP and PIP (SDK IK, tools/ik_coupling_probe.py); how the
mechanism splits closure between joints is not in the URDF. FINGER_PATHS holds the hypotheses:
  "mcp_first": MCP 0->80 first, then PIP/DIP (consistent with the hardware log: commanded MCP 45 ->
               encoder-based PIP 0). Default, NOT verified across the full range.
  "synergy":   all three joints close proportionally.
  "measured":  DEFAULT. Real GMH15 coupling from a motor sweep (scripts/motor_range_probe.py,
               2026-11-07, mean of middle/ring/little, SDK encoder FK). s = motor counts / 10000:
               s 0-0.67 bends MCP 0->80 only, then PIP 0->80 and DIP 0->75 curl together.
Thumb bounds are the measured hardware range (MCP/IP stop mechanically at ~63 / ~84 deg).

Cost, evaluated in matching canonical palm frames (x: pinky->index base, y: finger direction,
z = x × y), human vectors scaled by alpha = robot / human middle-finger length:
  - finger base -> tip vectors (4 fingers) and index base -> thumb tip
  - thumb tip -> finger tip vectors; when the human pair distance < pinch_near the target becomes a
    short vector of length pinch_eps in the same direction (DexPilot), blended up to pinch_far
  - temporal smoothness to the previous solution

Per-operator closure calibration (open_s / fist_s, captured with o/f in run_sim): the raw closure
is remapped (s - open) / (fist - open) so a relaxed open hand -> 0 and a fist -> 1. For the pinch
fingers the remap fades out as the human pinch closes (the pinch terms already placed the tip).

Thumb basin hysteresis: the thumb cost is multi-modal (an occluded thumb in a fist jumps between
CMC ~0 and ~110 deg). Each frame the solver starts from the global grid seed AND from the previous
thumb; the global solution only wins if its cost is < thumb_switch_ratio x the warm one.
"""
import json
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from .gmh15_model import GMH15Model, _axis_angle, _tf

FINGERS4 = ("index", "middle", "ring", "pinky")
HUMAN_TIP = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
HUMAN_MCP = {"index": 5, "middle": 9, "ring": 13, "pinky": 17}
# joint path table: s -> (joint_0, joint_1, joint_2) degrees
FINGER_PATHS = {
    "mcp_first": np.array([[0.0, 0, 0, 0], [0.5, 80, 0, 0], [1.0, 80, 81.6, 76.0]]),
    "synergy": np.array([[0.0, 0, 0, 0], [1.0, 80, 81.6, 76.0]]),
    "measured": np.array([
        [0.0000, 0.0, 0.0, 0.0], [0.0462, 6.6, 0.0, 0.0], [0.0972, 13.7, 0.0, 0.0],
        [0.1460, 20.2, 0.0, 0.0], [0.1981, 26.4, 0.0, 0.0], [0.2461, 31.8, 0.0, 0.0],
        [0.2972, 37.5, 0.0, 0.0], [0.3468, 42.8, 0.0, 0.0], [0.3968, 48.3, 0.0, 0.0],
        [0.4471, 53.7, 0.0, 0.0], [0.4970, 59.2, 0.0, 0.0], [0.5468, 64.7, 0.0, 0.0],
        [0.5969, 70.2, 0.0, 0.0], [0.6461, 75.9, 0.0, 0.0], [0.6718, 80.0, 0.0, 0.0],
        [0.6981, 80.0, 7.0, 3.9], [0.7211, 80.0, 15.3, 9.1], [0.7479, 80.0, 23.6, 15.0],
        [0.7712, 80.0, 30.1, 20.1], [0.7969, 80.0, 36.6, 25.7], [0.8231, 80.0, 42.8, 31.6],
        [0.8457, 80.0, 47.8, 36.7], [0.8719, 80.0, 53.4, 42.7], [0.8970, 80.0, 58.5, 48.6],
        [0.9208, 80.0, 63.4, 54.4], [0.9481, 80.0, 69.1, 61.4], [0.9719, 80.0, 74.4, 67.8],
        [1.0000, 80.0, 80.0, 75.0]]),
}
# measured hardware range (CMC full; MCP/IP software ceilings 7200/7700 counts below the
# mechanical end stops found at ~7500/~8000 counts)
THUMB_LIMITS_DEG = ((0.0, 109.2), (0.0, 63.0), (0.0, 84.3))


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def frame_from(index_base, pinky_base, finger_dir):
    x = _unit(index_base - pinky_base)
    z = _unit(np.cross(x, finger_dir))
    y = np.cross(z, x)
    return np.stack([x, y, z])          # rows: canonical axes; R @ v -> canonical coords


@dataclass
class OptConfig:
    path: str = "measured"
    w_finger: float = 1.0
    w_thumb: float = 1.0
    w_pair_far: float = 0.2
    w_pair_near: float = 10.0
    pinch_near_m: float = 0.02      # human thumb-finger tip distance: full pinch behaviour
    pinch_far_m: float = 0.05       # above this: plain scaled vector
    pinch_eps_m: float = 0.006      # robot pad-centroid target distance when pinching
    w_smooth: float = 2e-3
    pinch_fingers: tuple = ("index", "middle")
    w_thumb_prior: float = 5e-3     # weak hand-shape prior: robot thumb MCP/IP ~ human thumb MCP/IP
    open_s: tuple = (0.0, 0.0, 0.0, 0.0)   # raw closure of this operator's relaxed open hand
    fist_s: tuple = (1.0, 1.0, 1.0, 1.0)   # raw closure of this operator's fist
    thumb_switch_ratio: float = 0.6        # global thumb solution must be this much cheaper to switch


def load_calib(cfg: OptConfig, path):
    with open(path) as fh:
        d = json.load(fh)
    cfg.open_s, cfg.fist_s = tuple(d["open_s"]), tuple(d["fist_s"])
    return cfg


def save_calib(cfg: OptConfig, path):
    with open(path, "w") as fh:
        json.dump({"open_s": [round(float(v), 4) for v in cfg.open_s],
                   "fist_s": [round(float(v), 4) for v in cfg.fist_s]}, fh, indent=1)


class GMH15VectorRetargeter:
    needs_landmarks = True

    def __init__(self, model: GMH15Model, cfg: OptConfig = None):
        self.m, self.cfg = model, cfg or OptConfig()
        s = model.side
        z = model.q_from_cmd(np.zeros(11))
        self.base = {f: model.link_tf(f"{s}_{f}_link_0", z)[:3, 3] for f in FINGERS4}
        open_mid = model.tip("middle", z)
        self.R = frame_from(self.base["index"], self.base["pinky"], open_mid - self.base["middle"])
        self.robot_mid_len = float(np.linalg.norm(open_mid - self.base["middle"]))
        # finger tip lookup along the joint path (palm frame)
        self.s_grid = np.linspace(0, 1, 201)
        path = FINGER_PATHS[self.cfg.path]
        self.path_q = np.stack([np.interp(self.s_grid, path[:, 0], path[:, k]) for k in (1, 2, 3)], 1)
        self.tip_table = {}
        for f in FINGERS4:
            pts = []
            for q3 in self.path_q:
                q = {f"{s}_{f}_joint_{k}": np.deg2rad(q3[k]) for k in range(3)}
                pts.append(model.tip(f, q))
            self.tip_table[f] = np.array(pts)
        # thumb chain for fast FK
        chain, link = [], f"{s}_thumb_tactile_link_0"
        while link in model.child_joint:
            j = model.child_joint[link]
            chain.append(j)
            link = j["parent"]
        self.thumb_chain = list(reversed(chain))
        self.thumb_tip_local = np.r_[model.tip_local("thumb"), 1.0]
        # coarse thumb grid for global seeding (canonical frame)
        g = [np.linspace(a, b, n) for (a, b), n in zip(THUMB_LIMITS_DEG, (9, 7, 7))]
        self.thumb_grid = np.array(np.meshgrid(*g, indexing="ij")).reshape(3, -1).T
        self.thumb_grid_tips = np.array([self.R @ self.thumb_tip(t) for t in self.thumb_grid])
        self.finger_tips_c = {f: self.tip_table[f] @ self.R.T for f in FINGERS4}
        self.x_prev = None
        self.last = {}

    # ---------------- robot kinematics ----------------
    def finger_tip(self, f, s):
        k = np.clip(s, 0, 1) * (len(self.s_grid) - 1)
        i = int(min(np.floor(k), len(self.s_grid) - 2))
        a = k - i
        return (1 - a) * self.tip_table[f][i] + a * self.tip_table[f][i + 1]

    def thumb_tip(self, th_deg):
        T = np.eye(4)
        k = 0
        for j in self.thumb_chain:
            T = T @ j["T0"]
            if j["type"] == "revolute":
                T = T @ _tf(_axis_angle(j["axis"], np.deg2rad(th_deg[k])), np.zeros(3))
                k += 1
        return (T @ self.thumb_tip_local)[:3]

    def robot_tips(self, x):
        tips = {f: self.R @ self.finger_tip(f, x[i]) for i, f in enumerate(FINGERS4)}
        tips["thumb"] = self.R @ self.thumb_tip(x[4:7])
        return tips

    # ---------------- human side ----------------
    def human_targets(self, world):
        p = np.asarray(world, float)
        Rh = frame_from(p[5], p[17], p[9] - p[0])
        h_mid_len = sum(np.linalg.norm(p[a + 1] - p[a]) for a in (9, 10, 11))
        alpha = self.robot_mid_len / max(h_mid_len, 1e-6)
        P = (p - p[0]) @ Rh.T            # canonical human coords (metres)
        return P, alpha

    # ---------------- optimisation ----------------
    def _residuals_setup(self, P, alpha):
        c = self.cfg
        rb = {f: self.R @ self.base[f] for f in FINGERS4}
        terms = []   # (kind, finger, target_vec, weight)
        for f in FINGERS4:
            terms.append(("tip", f, alpha * (P[HUMAN_TIP[f]] - P[HUMAN_MCP[f]]), c.w_finger))
        terms.append(("thumb", "thumb", alpha * (P[HUMAN_TIP["thumb"]] - P[HUMAN_MCP["index"]]), c.w_thumb))
        pinch, self._pinch_t = {}, np.ones(4)
        for i, f in enumerate(FINGERS4):
            v = P[HUMAN_TIP[f]] - P[HUMAN_TIP["thumb"]]
            d = float(np.linalg.norm(v))
            pinch[f] = d
            if f not in c.pinch_fingers:
                continue        # ring/pinky: tip vectors only (pair terms let thumb errors leak in)
            t = np.clip((d - c.pinch_near_m) / (c.pinch_far_m - c.pinch_near_m), 0, 1)
            self._pinch_t[i] = t
            target = (1 - t) * _unit(v) * c.pinch_eps_m + t * alpha * v
            w = (1 - t) * c.w_pair_near + t * c.w_pair_far
            terms.append(("pair", f, target, w))
        return rb, terms, pinch

    def solve(self, world):
        P, alpha = self.human_targets(world)
        rb, terms, pinch = self._residuals_setup(P, alpha)
        c = self.cfg
        def _ang(u, v):
            return np.degrees(np.arccos(np.clip(_unit(u) @ _unit(v), -1, 1)))
        th_prior = np.array([_ang(P[2] - P[1], P[3] - P[2]), _ang(P[3] - P[2], P[4] - P[3])])
        self._th_prior = th_prior
        x_prev = self.x_prev
        scale = np.array([1, 1, 1, 1, 100, 100, 100.0])   # normalise thumb degrees for smoothing

        kinds = [(k, f, tgt, w) for k, f, tgt, w in terms]
        N = len(self.s_grid) - 1

        def cost_grad(x):
            # tips and their derivatives w.r.t. x (finger: table slope; thumb: forward differences)
            tips, dtips = {}, {}
            for i, f in enumerate(FINGERS4):
                k = np.clip(x[i], 0, 1) * N
                j = int(min(np.floor(k), N - 1))
                a = k - j
                tab = self.tip_table[f]
                tips[f] = self.R @ ((1 - a) * tab[j] + a * tab[j + 1])
                g = np.zeros((3, 7))
                g[:, i] = self.R @ (tab[j + 1] - tab[j]) * N
                dtips[f] = g
            th = x[4:7]
            t0 = self.R @ self.thumb_tip(th)
            g = np.zeros((3, 7))
            for k in range(3):
                d = th.copy()
                d[k] += 1e-3
                g[:, 4 + k] = (self.R @ self.thumb_tip(d) - t0) / 1e-3
            tips["thumb"], dtips["thumb"] = t0, g
            e, grad = 0.0, np.zeros(7)
            for kind, f, tgt, w in kinds:
                if kind == "tip":
                    v, J = tips[f] - rb[f], dtips[f]
                elif kind == "thumb":
                    v, J = tips["thumb"] - rb["index"], dtips["thumb"]
                else:
                    v, J = tips[f] - tips["thumb"], dtips[f] - dtips["thumb"]
                r_ = v - tgt
                e += w * float(r_ @ r_)
                grad += 2 * w * (r_ @ J)
            dp = (x[5:7] - th_prior) / 100.0
            e += c.w_thumb_prior * float(dp @ dp)
            grad[5:7] += 2 * c.w_thumb_prior * dp / 100.0
            if x_prev is not None:
                dx = (x - x_prev) / scale
                e += c.w_smooth * float(dx @ dx)
                grad += 2 * c.w_smooth * dx / scale
            return e, grad

        bounds = [(0, 1)] * 4 + list(THUMB_LIMITS_DEG)
        opts = dict(jac=True, method="L-BFGS-B", bounds=bounds, options=dict(maxiter=100))
        x0 = self._global_seed(rb, terms, th_prior)
        r = minimize(cost_grad, x0, **opts)
        switched = True
        if x_prev is not None:
            # same (globally seeded) fingers, previous thumb: keep the current thumb basin unless
            # the global one is clearly better
            rw = minimize(cost_grad, np.r_[x0[:4], x_prev[4:7]], **opts)
            if not r.fun < c.thumb_switch_ratio * rw.fun:
                r, switched = rw, False
        x = r.x.copy()
        self.x_prev = x.copy()
        x[:4] = self.calibrate_s(x[:4])
        tips = self.robot_tips(x)
        self.last = dict(alpha=alpha, cost=float(r.fun), iters=int(r.nit), human_pinch_m=pinch,
                         s_raw=r.x[:4].copy(), thumb_switched=switched,
                         robot_pinch_m={f: float(np.linalg.norm(tips[f] - tips["thumb"])) for f in FINGERS4})
        return x

    def calibrate_s(self, s_raw):
        o, f = np.asarray(self.cfg.open_s, float), np.asarray(self.cfg.fist_s, float)
        remapped = np.clip((s_raw - o) / np.maximum(f - o, 0.05), 0, 1)
        t = self._pinch_t           # 1 far from pinch -> full remap; 0 pinching -> raw (pinch fit kept)
        return (1 - t) * s_raw + t * remapped

    def _global_seed(self, rb, terms, th_prior=None):
        """Fingers are 1-D: pick each closure by a global grid search on its own tip term (avoids the
        local minimum where a curled finger cannot 'see' the open target). Then pick the thumb grid
        point minimising thumb + pair terms with those fingers."""
        tip_t = {f: (tgt, w) for k, f, tgt, w in terms if k == "tip"}
        s0 = np.zeros(4)
        for i, f in enumerate(FINGERS4):
            tgt, w = tip_t[f]
            e = np.sum((self.finger_tips_c[f] - rb[f] - tgt) ** 2, axis=1)
            s0[i] = self.s_grid[int(np.argmin(e))]
        ftips = {f: self.finger_tips_c[f][int(round(s0[i] * (len(self.s_grid) - 1)))] for i, f in enumerate(FINGERS4)}
        e = np.zeros(len(self.thumb_grid))
        for k, f, tgt, w in terms:
            if k == "thumb":
                e += w * np.sum((self.thumb_grid_tips - rb["index"] - tgt) ** 2, axis=1)
            elif k == "pair":
                e += w * np.sum((ftips[f] - self.thumb_grid_tips - tgt) ** 2, axis=1)
        if th_prior is not None:
            e += self.cfg.w_thumb_prior * np.sum(((self.thumb_grid[:, 1:] - th_prior) / 100.0) ** 2, axis=1)
        return np.r_[s0, self.thumb_grid[int(np.argmin(e))]]

    def reset(self):
        self.x_prev = None

    # ---------------- outputs ----------------
    def finger_joints_deg(self, s):
        k = np.clip(s, 0, 1)
        path = FINGER_PATHS[self.cfg.path]
        return [float(np.interp(k, path[:, 0], path[:, j])) for j in (1, 2, 3)]

    def to_cmd11(self, x):
        """SDK 11-joint order: (joint1, joint2) per finger + thumb 3, degrees."""
        out = []
        for i, f in enumerate(FINGERS4):
            j0, j1, _ = self.finger_joints_deg(x[i])
            out += [j0, j1]
        return np.array(out + list(x[4:7]), float)

    def to_urdf_q(self, x):
        s = self.m.side
        q = {}
        for i, f in enumerate(FINGERS4):
            for k, v in enumerate(self.finger_joints_deg(x[i])):
                q[f"{s}_{f}_joint_{k}"] = np.deg2rad(v)
        for k in range(3):
            q[f"{s}_thumb_joint_{k}"] = np.deg2rad(x[4 + k])
        return q

    def tips_palm(self, x):
        """Robot tip positions in the URDF palm frame (for sim markers)."""
        return {f: self.R.T @ v for f, v in self.robot_tips(x).items()}
