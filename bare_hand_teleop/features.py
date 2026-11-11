"""Landmarks -> joint-angle features. All features are angles between vectors or distance
ratios, so they are invariant to mirroring (same code for left and right hands)."""
import numpy as np

from .hand_types import HandFeatures

WRIST = 0
THUMB = (1, 2, 3, 4)                     # CMC, MCP, IP, TIP
FINGERS = {"index": (5, 6, 7, 8), "middle": (9, 10, 11, 12),
           "ring": (13, 14, 15, 16), "little": (17, 18, 19, 20)}
EPS = 1e-9


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > EPS else v * 0.0


def angle(u, v):
    u, v = _unit(u), _unit(v)
    return float(np.arccos(np.clip(np.dot(u, v), -1.0, 1.0)))


def palm_normal(p):
    return _unit(np.cross(p[5] - p[WRIST], p[17] - p[WRIST]))


def mcp_flexion(p, mcp, pip, n):
    """Flexion at MCP measured in the plane spanned by the metacarpal direction and the palm
    normal, so finger spreading (abduction) does not read as flexion."""
    d = _unit(p[mcp] - p[WRIST])
    seg = p[pip] - p[mcp]
    proj = np.dot(seg, d) * d + np.dot(seg, n) * n
    if np.linalg.norm(proj) < EPS:
        return 0.0
    return angle(d, proj)


def extract(world):
    """world: (21,3) landmarks (metres preferred). Returns HandFeatures."""
    p = np.asarray(world, dtype=float)
    if p.shape != (21, 3) or not np.all(np.isfinite(p)):
        raise ValueError(f"bad landmark array {p.shape}")
    n = palm_normal(p)
    scale = float(np.linalg.norm(p[9] - p[WRIST]))
    f = HandFeatures(hand_scale_m=scale)
    for name, (a, b, c, d) in FINGERS.items():
        m = mcp_flexion(p, a, b, n)
        pi = angle(p[b] - p[a], p[c] - p[b])
        di = angle(p[c] - p[b], p[d] - p[c])
        f.finger_mcp[name], f.finger_pip[name], f.finger_dip[name] = m, pi, di
        f.finger_total[name] = m + pi + di
        dist = float(np.linalg.norm(p[THUMB[3]] - p[d]))
        f.pinch_m[name] = dist
        f.pinch_ratio[name] = dist / max(scale, EPS)
    f.thumb_mcp = angle(p[2] - p[1], p[3] - p[2])
    f.thumb_ip = angle(p[3] - p[2], p[4] - p[3])
    f.thumb_opp = float(np.linalg.norm(p[4] - p[17])) / max(scale, EPS)
    return f
