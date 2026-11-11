"""GMH15 fingertip workspace + task feasibility, on the vendor URDF (model only, not hardware).

Compares four kinematic regimes:
  A  15 independent joints (URDF limits)             - the "15 DOF" the URDF suggests
  B  7 motors, tendon coupling "synergy"             - each finger: 3 joints close proportionally
  C  7 motors, tendon coupling "mcp_first"           - each finger: MCP first, then PIP/DIP
  D  current teleop server limits                    - fingers MCP 0-45, PIP=DIP=0, thumb 0-30 each
B/C are hypotheses (real tendon split not measured yet); the truth is one curve per finger, like B/C.

Outputs (runs/workspace/): summary.json, overview.png, viewer.html (interactive, open in a browser).
Usage: .venv/bin/python tools/workspace_analysis.py
"""
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from bare_hand_teleop.gmh15_model import GMH15Model, read_stl  # noqa: E402
from bare_hand_teleop.opt_retarget import FINGER_PATHS  # noqa: E402

OUT = os.path.join(ROOT, "runs/workspace")
URDF = os.path.join(ROOT, "vendor/yubi-pi_gmh15-sdk/models/right/meshes/DexH15_right_urdf.urdf")
FINGERS4 = ("index", "middle", "ring", "pinky")
CN = {"thumb": "拇指", "index": "食指", "middle": "中指", "ring": "无名指", "pinky": "小指"}
REGIMES = {
    "A": "A 15个关节都独立（URDF 理想）",
    "B": "B 7电机·腱耦合(三关节同比)",
    "C": "C 7电机·腱耦合(先MCP后PIP)",
    "D": "D 当前服务器限位",
}
CONTACT_MM = 10.0      # pad-centroid distance treated as "pads touch" (pads are a few mm thick each)
OPPOSE_DEG = 120.0     # angle between pad normals needed to call it an opposed (squeezing) pinch

m = GMH15Model(URDF)
S = m.side
LIM = {n: np.rad2deg(j["limit"]) for n, j in m.joints.items() if j["limit"] is not None}


def tf_point(T, p):
    return (T @ np.r_[p, 1.0])[:3]


def pad_frame(finger, q):
    """Pad centroid and outward pad normal (local +z of the tactile link: thinnest pad axis, = flexion direction; checked below)."""
    T = m.link_tf(f"{S}_{finger}_tactile_link_0", q)
    return tf_point(T, m.tip_local(finger)), T[:3, 2].copy()


def finger_samples(finger, regime, n_grid=16, n_curve=241):
    names = [f"{S}_{finger}_joint_{k}" for k in range(3)]
    if regime == "A":
        g = [np.linspace(*LIM[n], n_grid) for n in names]
        Q = np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)
    elif regime in ("B", "C"):
        path = FINGER_PATHS["synergy" if regime == "B" else "mcp_first"]
        s = np.linspace(0, 1, n_curve)
        Q = np.stack([np.interp(s, path[:, 0], path[:, k]) for k in (1, 2, 3)], 1)
    else:
        Q = np.c_[np.linspace(0, 45, n_curve), np.zeros(n_curve), np.zeros(n_curve)]
    P, N, L1 = [], [], []
    for qd in Q:
        q = dict(zip(names, np.deg2rad(qd)))
        p, nrm = pad_frame(finger, q)
        P.append(p), N.append(nrm)
        L1.append(m.link_tf(f"{S}_{finger}_link_1", q))
    return Q, np.array(P), np.array(N), L1


def thumb_samples(regime, n_grid=22):
    names = [f"{S}_thumb_joint_{k}" for k in range(3)]
    hi = [30, 30, 30] if regime == "D" else [LIM[n][1] for n in names]
    g = [np.linspace(0, h, n_grid) for h in hi]
    Q = np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)
    P, N = [], []
    for qd in Q:
        p, nrm = pad_frame("thumb", dict(zip(names, np.deg2rad(qd))))
        P.append(p), N.append(nrm)
    return Q, np.array(P), np.array(N)


def voxel_volume_cm3(P, h=0.002):
    return len({tuple(v) for v in np.floor(P / h).astype(int)}) * h ** 3 * 1e6


def planar_area_mm2(P, normal, h=0.001):
    """Area of a planar point set (fingers in regime A) by 1 mm occupancy in the finger plane."""
    n = normal / np.linalg.norm(normal)
    u = np.cross(n, [1, 0, 0]); u /= np.linalg.norm(u); v = np.cross(n, u)
    uv = np.c_[(P - P.mean(0)) @ u, (P - P.mean(0)) @ v]
    return len({tuple(c) for c in np.floor(uv / h).astype(int)}) * (h * 1000) ** 2


class Collider:
    """MuJoCo mesh collision check (meshes are convex-hulled by MuJoCo -> slightly conservative).
    The converted hand.xml ships with collisions off; we switch them on here."""

    PEN_MM = 2.0

    def __init__(self):
        import mujoco
        self.mj = mujoco
        d = os.path.join(ROOT, "vendor/yubi-pi_gmh15-sdk/models", S)
        spec = mujoco.MjSpec.from_file(os.path.join(d, "hand.xml"))
        spec.meshdir = os.path.join(d, "meshes") + os.sep
        for g in spec.geoms:
            g.contype, g.conaffinity = 1, 1
        self.M = spec.compile()
        self.D = mujoco.MjData(self.M)
        self.qadr = {mujoco.mj_id2name(self.M, mujoco.mjtObj.mjOBJ_JOINT, j): self.M.jnt_qposadr[j]
                     for j in range(self.M.njnt)}
        self.bname = [mujoco.mj_id2name(self.M, mujoco.mjtObj.mjOBJ_BODY, self.M.geom_bodyid[g]).replace(S + "_", "")
                      for g in range(self.M.ngeom)]
        self.parent = {mujoco.mj_id2name(self.M, mujoco.mjtObj.mjOBJ_BODY, b).replace(S + "_", ""):
                       mujoco.mj_id2name(self.M, mujoco.mjtObj.mjOBJ_BODY, self.M.body_parentid[b]).replace(S + "_", "")
                       for b in range(1, self.M.nbody)}

    def collisions(self, q, allow=()):
        """q: {joint: rad}. Returns {(bodyA, bodyB): depth_mm} for non-adjacent, non-allowed pairs."""
        self.D.qpos[:] = 0
        for k, v in q.items():
            self.D.qpos[self.qadr[k]] = v
        self.mj.mj_forward(self.M, self.D)
        out = {}
        for i in range(self.D.ncon):
            c = self.D.contact[i]
            a, b = self.bname[c.geom1], self.bname[c.geom2]
            if self.parent.get(a) == b or self.parent.get(b) == a or c.dist * 1000 > -self.PEN_MM:
                continue
            if any({a, b} == set(p) for p in allow):
                continue
            key = tuple(sorted((a, b)))
            out[key] = min(out.get(key, 0.0), round(c.dist * 1000, 1))
        return out


COLLIDER = None


def best_pinch(Pt, Nt, Pf, Nf, valid=None, max_checks=4000):
    """min pad distance; best opposition angle among pairs within CONTACT_MM whose pose passes
    valid(thumb_idx, finger_idx) (collision check)."""
    tree = cKDTree(Pf)
    d, i = tree.query(Pt)
    k = int(np.argmin(d))
    out = {"min_mm": float(d[k] * 1000), "thumb_idx": k, "finger_idx": int(i[k]),
           "oppose_deg": None, "oppose_pair": None, "rejected_collision": 0}
    cand = []
    for a, lst in enumerate(tree.query_ball_point(Pt, CONTACT_MM / 1000)):
        for b in lst:
            cand.append((np.degrees(np.arccos(np.clip(Nt[a] @ Nf[b], -1, 1))), a, b))
    cand.sort(reverse=True)
    for ang, a, b in cand[:max_checks]:
        if valid is None or valid(a, b):
            out["oppose_deg"], out["oppose_pair"] = float(ang), (a, b)
            break
        out["rejected_collision"] += 1
    out["n_touch_poses"] = len(cand)
    return out


def pinch_valid(Qt, Qf, finger, allow_extra=()):
    allow = [(t, g) for t in ("thumb_link_2", "thumb_tactile_link_0")
             for g in (f"{finger}_link_2", f"{finger}_tactile_link_0")] + list(allow_extra)

    def ok(a, b):
        q = {f"{S}_thumb_joint_{k}": np.deg2rad(Qt[a][k]) for k in range(3)}
        q.update({f"{S}_{finger}_joint_{k}": np.deg2rad(Qf[b][k]) for k in range(3)})
        return not COLLIDER.collisions(q, allow)
    return ok


def tip_jacobian_rank(regime, finger):
    """Number of independent fingertip motion directions available from the actuators."""
    if finger == "thumb":
        return 3
    return 2 if regime == "A" else 1   # A: 3 parallel axes -> planar (2D) tip motion; B/C/D: 1 motor


# ---------------------------------------------------------------------------------------------
def main():
    os.makedirs(OUT, exist_ok=True)
    q0 = {}
    global COLLIDER
    COLLIDER = Collider()
    assert not COLLIDER.collisions({}), "open hand should be collision free"
    # sanity check: pad normal (+z of tactile link) points the way the fingertip moves when flexing
    p_open, n_open = pad_frame("index", q0)
    p_flex, _ = pad_frame("index", {f"{S}_index_joint_0": np.deg2rad(5)})
    assert (p_flex - p_open) @ n_open > 0, "pad normal sign check failed"

    data, summary = {}, {"regimes": REGIMES, "contact_mm": CONTACT_MM, "tasks": {}, "workspace": {}}
    for r in REGIMES:
        print(f"sampling {REGIMES[r]} ...")
        Qt, Pt, Nt = thumb_samples(r)
        d = {"thumb": (Qt, Pt, Nt, None)}
        for f in FINGERS4:
            d[f] = finger_samples(f, r)
        data[r] = d

    # ---------------- workspace size ----------------
    for r, d in data.items():
        ws = {}
        for f, (Q, P, N, _) in d.items():
            if f == "thumb":
                ws[f] = {"kind": "3D 体积", "value": round(voxel_volume_cm3(P), 1), "unit": "cm³"}
            elif r == "A":
                axis = m.link_tf(f"{S}_{f}_link_0", {})[:3, 2]
                off = np.abs((P - P.mean(0)) @ axis).max() * 1000
                ws[f] = {"kind": "平面面积", "value": round(planar_area_mm2(P, axis) / 100, 1), "unit": "cm²",
                         "out_of_plane_mm": round(off, 2)}
            else:
                ws[f] = {"kind": "曲线长度", "value": round(np.linalg.norm(np.diff(P, axis=0), axis=1).sum() * 1000, 1),
                         "unit": "mm"}
            ws[f]["tip_dirs"] = tip_jacobian_rank(r, f)
        summary["workspace"][r] = ws

    # ---------------- tasks ----------------
    tasks = {}

    def add(key, title, need, per_regime):
        tasks[key] = {"title": title, "need": need, "result": per_regime}

    # T1/T2 thumb opposes each finger
    pinch_poses = {}
    for f in FINGERS4:
        res = {}
        for r, d in data.items():
            Qt, Pt, Nt, _ = d["thumb"]
            Qf, Pf, Nf, _ = d[f]
            b = best_pinch(Pt, Nt, Pf, Nf, valid=pinch_valid(Qt, Qf, f))
            ok_touch = b["oppose_deg"] is not None
            ok_oppose = ok_touch and b["oppose_deg"] >= OPPOSE_DEG
            if ok_oppose:
                verdict = "能"
            elif ok_touch:
                verdict = "勉强（碰到但指腹不相对）"
            elif b["min_mm"] <= CONTACT_MM:
                verdict = "不能（碰得到但会穿模）"
            else:
                verdict = "不能"
            res[r] = {"verdict": verdict, "min_mm": round(b["min_mm"], 1),
                      "oppose_deg": None if b["oppose_deg"] is None else round(b["oppose_deg"], 0),
                      "rejected_collision": b["rejected_collision"]}
            if b["oppose_pair"] is not None:
                a, c = b["oppose_pair"]
            else:
                a, c = b["thumb_idx"], b["finger_idx"]
            pinch_poses[(r, f)] = (Qt[a].tolist(), Qf[c].tolist())
            res[r]["thumb_deg"] = np.round(Qt[a], 1).tolist()
            res[r]["finger_deg"] = np.round(Qf[c], 1).tolist()
        add(f"pinch_{f}", f"拇指–{CN[f]} 指尖对捏", f"两指腹中心距 ≤{CONTACT_MM:.0f} mm 且指腹法线夹角 ≥{OPPOSE_DEG:.0f}°", res)

    # T3 tripod: thumb, index, middle tips all within a 30 mm ball
    res = {}
    for r, d in data.items():
        Pt = d["thumb"][1]
        ti, tm = cKDTree(d["index"][1]), cKDTree(d["middle"][1])
        di, ii = ti.query(Pt); dm, im = tm.query(Pt)
        dim = np.linalg.norm(d["index"][1][ii] - d["middle"][1][im], axis=1)
        spread = np.max(np.c_[di, dm, dim], 1) * 1000
        Qt, Qi, Qm = d["thumb"][0], d["index"][0], d["middle"][0]
        allow = [(t, g) for t in ("thumb_link_2", "thumb_tactile_link_0")
                 for g in ("index_link_2", "index_tactile_link_0", "middle_link_2", "middle_tactile_link_0")]
        allow += [(a, b) for a in ("index_link_1", "index_link_2", "index_tactile_link_0")
                  for b in ("middle_link_1", "middle_link_2", "middle_tactile_link_0")]
        best, rej = None, 0
        for k in np.argsort(spread)[:3000]:
            if spread[k] > 30:
                break
            q = {f"{S}_thumb_joint_{j}": np.deg2rad(Qt[k][j]) for j in range(3)}
            q.update({f"{S}_index_joint_{j}": np.deg2rad(Qi[ii[k]][j]) for j in range(3)})
            q.update({f"{S}_middle_joint_{j}": np.deg2rad(Qm[im[k]][j]) for j in range(3)})
            if not COLLIDER.collisions(q, allow):
                best = k
                break
            rej += 1
        k0 = int(np.argmin(spread))
        res[r] = {"verdict": "能" if best is not None else ("不能（会穿模）" if spread[k0] <= 30 else "不能"),
                  "max_pair_mm": round(float(spread[best if best is not None else k0]), 1), "rejected_collision": rej}
    add("tripod", "三指捏（拇指+食指+中指，捏笔/小球）", "三个指腹两两距离 ≤30 mm", res)

    # T4 lateral key pinch: thumb pad on the thumb-side face of index middle phalanx
    res = {}
    v1 = read_stl(os.path.join(m.root_dir, m.meshes[f"{S}_index_link_1"]))
    for r, d in data.items():
        Qt, Pt, Nt, _ = d["thumb"]
        _, _, _, L1 = d["index"]
        targets, tnorm = [], []
        for T in L1:
            lat = T[:3, 2] * np.sign(T[:3, 2] @ np.array([1.0, 0, 0]))   # joint axis, toward thumb (+x)
            c = tf_point(T, v1.mean(0))
            half = np.abs((v1 - v1.mean(0)) @ (T[:3, :3].T @ lat)).max()
            targets.append(c + lat * half), tnorm.append(lat)
        Qi = d["index"][0]
        b = best_pinch(Pt, Nt, np.array(targets), np.array(tnorm),
                       valid=pinch_valid(Qt, Qi, "index", [(t, "index_link_1") for t in ("thumb_link_2", "thumb_tactile_link_0")]))
        ok = b["oppose_deg"] is not None and b["oppose_deg"] >= OPPOSE_DEG
        res[r] = {"verdict": "能" if ok else ("勉强" if b["oppose_deg"] is not None else
                                             "不能（会穿模）" if b["min_mm"] <= CONTACT_MM else "不能"),
                  "min_mm": round(b["min_mm"], 1),
                  "oppose_deg": None if b["oppose_deg"] is None else round(b["oppose_deg"], 0)}
    add("key_pinch", "侧捏（拇指压食指侧面，拿钥匙/卡片）", "拇指腹到食指中节侧面 ≤10 mm 且相对", res)

    # T5 finger spread / abduction
    res = {}
    for r, d in data.items():
        lat = []
        for f in FINGERS4:
            axis = m.link_tf(f"{S}_{f}_link_0", {})[:3, 2]
            P = d[f][1]
            lat.append(np.ptp(P @ axis) * 1000)
        res[r] = {"verdict": "不能", "lateral_range_mm": round(float(max(lat)), 2)}
    add("spread", "手指张开/并拢（外展，抓大盘子、比 V 字张开、按琴键和弦）",
        "指尖要能左右移动（人手约 ±20° 外展）", res)

    # T6 place fingertip on an arbitrary point in its plane (touch-screen tap, button, in-hand repositioning)
    res = {}
    for r in REGIMES:
        k = summary["workspace"][r]["index"]["tip_dirs"]
        res[r] = {"verdict": "能（仅限手指平面内）" if k >= 2 else "不能（只能沿一条固定轨迹）", "tip_dirs": k}
    add("place_tip", "指尖点到指定位置（按按钮/点屏幕/手内换位）", "指尖至少 2 个独立运动方向（平面内）；任意 3D 需 3 个", res)

    # T7 in-hand rotation / finger gaiting (needs ~3 tip DOF per contact finger)
    res = {}
    for r in REGIMES:
        k = summary["workspace"][r]["index"]["tip_dirs"]
        res[r] = {"verdict": "不能" if k < 3 else "能", "tip_dirs": k}
    add("in_hand", "手内旋转/搓动物体（转笔、拧小瓶盖）", "参与接触的手指指尖需要 3 个独立方向", res)

    # T8 power / hook grasp (all fingers curl toward palm)
    res = {}
    for r, d in data.items():
        ext = []
        for f in FINGERS4:
            Q, P, _, _ = d[f]
            ext.append(np.linalg.norm(P[-1] - P[0]) * 1000)
        res[r] = {"verdict": "能" if r != "D" else "部分（只弯到 45°，握不紧）",
                  "tip_travel_mm": round(float(np.mean(ext)), 1)}
    add("power", "整手握/钩握（握杯子、提袋子）", "四指能卷向掌心", res)

    summary["tasks"] = tasks
    with open(os.path.join(OUT, "summary.json"), "w") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=1)

    print_report(summary)
    make_png(data, pinch_poses)
    make_html(data, summary, pinch_poses)
    print("wrote", OUT)


def print_report(s):
    print("\n=== 指尖工作空间 ===")
    for r, ws in s["workspace"].items():
        print(REGIMES[r])
        for f, w in ws.items():
            extra = f"  (离开手指平面 {w['out_of_plane_mm']} mm)" if "out_of_plane_mm" in w else ""
            print(f"   {CN[f]:4s} {w['kind']} {w['value']} {w['unit']}  指尖独立方向数={w['tip_dirs']}{extra}")
    print("\n=== 任务 ===")
    for t in s["tasks"].values():
        print(f"{t['title']}  [{t['need']}]")
        for r, v in t["result"].items():
            print(f"   {r}: {v}")


# ---------------------------------------------------------------------------------------------
COLORS = {"thumb": "#d62728", "index": "#1f77b4", "middle": "#2ca02c", "ring": "#9467bd", "pinky": "#ff7f0e"}


def skeleton(qdeg=None):
    """Polyline of joint origins per finger + tip, palm frame (m)."""
    q = {}
    if qdeg:
        for k, v in qdeg.items():
            q[k] = np.deg2rad(v)
    lines = {}
    for f in ("thumb",) + FINGERS4:
        pts = [np.zeros(3)]
        for k in range(3):
            pts.append(m.link_tf(f"{S}_{f}_link_{k}", q)[:3, 3])
        pts.append(pad_frame(f, q)[0])
        lines[f] = np.array(pts)
    return lines


def pose_q(r, f, pose):
    th, fi = pose
    q = {f"{S}_thumb_joint_{k}": th[k] for k in range(3)}
    q.update({f"{S}_{f}_joint_{k}": fi[k] for k in range(3)})
    return q


def make_png(data, pinch_poses):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for name in ("PingFang SC", "Heiti SC", "STHeiti", "Arial Unicode MS", "Songti SC"):
        if any(name in f.name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.sans-serif"] = [name]
            break
    plt.rcParams["axes.unicode_minus"] = False

    fig = plt.figure(figsize=(18, 10))
    for i, r in enumerate(REGIMES):
        ax = fig.add_subplot(2, 4, i + 1, projection="3d")
        for f, (Q, P, N, _) in data[r].items():
            Pm = P * 1000
            if len(P) < 300:
                ax.plot(Pm[:, 0], Pm[:, 1], Pm[:, 2], color=COLORS[f], lw=2.5, label=CN[f])
            else:
                step = max(1, len(P) // 2500)
                ax.scatter(Pm[::step, 0], Pm[::step, 1], Pm[::step, 2], s=1, alpha=0.25, color=COLORS[f], label=CN[f])
        for f, L in skeleton().items():
            ax.plot(*(L * 1000).T, color="k", lw=1)
        ax.set_title(REGIMES[r], fontsize=10)
        ax.set_xlabel("x mm"); ax.set_ylabel("y mm"); ax.set_zlabel("z mm")
        ax.view_init(elev=20, azim=-60)
        ax.set_box_aspect((1, 1, 1))
        lim = [-60, 160]
        ax.set_xlim(-60, 160); ax.set_ylim(-110, 110); ax.set_zlim(-200, 20)
        if i == 0:
            ax.legend(fontsize=8, markerscale=6, loc="upper left")

    # index finger plane: A region vs coupled curves; thumb cloud projected onto that plane
    ax = fig.add_subplot(2, 4, 5)
    axis = m.link_tf(f"{S}_index_link_0", {})[:3, 2]
    o = m.link_tf(f"{S}_index_link_0", {})[:3, 3]
    u = np.cross(axis, [1, 0, 0]); u /= np.linalg.norm(u); v = np.cross(axis, u)
    proj = lambda P: np.c_[(P - o) @ u, (P - o) @ v] * 1000  # noqa: E731
    PA = proj(data["A"]["index"][1])
    ax.scatter(PA[:, 0], PA[:, 1], s=2, color="#9ecae1", label="A 15独立：一片区域")
    for r, c in (("B", "#08519c"), ("C", "#e6550d"), ("D", "#31a354")):
        Pc = proj(data[r]["index"][1])
        ax.plot(Pc[:, 0], Pc[:, 1], color=c, lw=2.5, label=REGIMES[r][:12] + "：一条线")
    ax.set_aspect("equal"); ax.set_title("食指指尖在自己平面内的可达范围", fontsize=10)
    ax.legend(fontsize=7); ax.set_xlabel("mm"); ax.set_ylabel("mm")

    # top view showing no lateral spread
    ax = fig.add_subplot(2, 4, 6)
    for f in FINGERS4:
        P = data["A"][f][1] * 1000
        ax.scatter(P[:, 0], P[:, 2], s=1, color=COLORS[f], label=CN[f])
    P = data["A"]["thumb"][1][::4] * 1000
    ax.scatter(P[:, 0], P[:, 2], s=1, alpha=0.15, color=COLORS["thumb"], label="拇指")
    ax.set_aspect("equal"); ax.set_title("从手背方向看（x–z）：四指只有前后，没有左右", fontsize=10)
    ax.set_xlabel("x mm（左右）"); ax.set_ylabel("z mm（指向）")

    # pinch poses (best opposed pose found, regime B)
    for j, (r, f) in enumerate((("B", "index"), ("B", "pinky"))):
        ax = fig.add_subplot(2, 4, 7 + j, projection="3d")
        q = pose_q(r, f, pinch_poses[(r, f)])
        for g, L in skeleton({k: np.rad2deg(v) for k, v in q.items()}).items():
            ax.plot(*(L * 1000).T, color=COLORS[g], lw=2.5, marker="o", ms=3)
        ax.set_title(f"{REGIMES[r][:10]}：拇指–{CN[f]} 最佳对捏姿态", fontsize=10)
        ax.set_xlim(-60, 160); ax.set_ylim(-110, 110); ax.set_zlim(-200, 20); ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=20, azim=-60)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "overview.png"), dpi=110)


def make_html(data, summary, pinch_poses):
    pts = {}
    for r, d in data.items():
        pts[r] = {}
        for f, (Q, P, N, _) in d.items():
            Pm = np.round(P * 1000, 1)
            # voxel thin to 2 mm to keep file small
            _, keep = np.unique(np.floor(Pm / 2).astype(int), axis=0, return_index=True)
            pts[r][f] = {"curve": len(P) < 300, "p": Pm[np.sort(keep)].tolist() if len(P) >= 300 else Pm.tolist()}
    skel = {f: np.round(L * 1000, 1).tolist() for f, L in skeleton().items()}
    poses = {}
    for (r, f), pose in pinch_poses.items():
        q = pose_q(r, f, pose)
        poses[f"{r}_{f}"] = {g: np.round(L * 1000, 1).tolist()
                             for g, L in skeleton({k: np.rad2deg(v) for k, v in q.items()}).items()}
    payload = json.dumps({"pts": pts, "skel": skel, "poses": poses, "summary": summary,
                          "colors": COLORS, "cn": CN}, ensure_ascii=False)
    with open(os.path.join(os.path.dirname(__file__), "workspace_viewer_template.html"), encoding="utf-8") as fh:
        html = fh.read().replace("/*__DATA__*/null", payload)
    with open(os.path.join(OUT, "viewer.html"), "w", encoding="utf-8") as fh:
        fh.write(html)


if __name__ == "__main__":
    main()
