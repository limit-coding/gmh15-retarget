"""Synthetic 21-landmark hand with known joint angles (for tests and offline demos).
Palm in the x-y plane, fingers along +y, flexion bends toward -z (palm side)."""
import numpy as np

MCP_POS = {"index": (0.025, 0.090), "middle": (0.0, 0.095), "ring": (-0.020, 0.088), "little": (-0.038, 0.075)}
SEG = {"index": (0.040, 0.025, 0.020), "middle": (0.045, 0.028, 0.021),
       "ring": (0.042, 0.026, 0.020), "little": (0.033, 0.020, 0.018)}
BASE = {"index": 5, "middle": 9, "ring": 13, "little": 17}
Z = np.array([0.0, 0.0, 1.0])


def _unit(v):
    return v / np.linalg.norm(v)


def _chain(p0, d0, bend_dir, lengths, angles):
    pts, p, cum = [], np.asarray(p0, float), 0.0
    for L, a in zip(lengths, angles):
        cum += a
        p = p + L * (np.cos(cum) * d0 + np.sin(cum) * bend_dir)
        pts.append(p)
    return pts


def make_hand(flex=None, thumb=(0.0, 0.1, 0.1), mirror=False):
    """flex: {finger: (mcp, pip, dip)} radians. thumb: (opposition 0..1, mcp, ip) radians."""
    flex = flex or {}
    p = np.zeros((21, 3))
    for name, (x, y) in MCP_POS.items():
        b = BASE[name]
        p[b] = (x, y, 0.0)
        d0 = _unit(p[b])
        for k, q in enumerate(_chain(p[b], d0, -Z, SEG[name], flex.get(name, (0.0, 0.0, 0.0)))):
            p[b + 1 + k] = q
    opp, t_mcp, t_ip = thumb
    p[1] = (0.022, 0.025, 0.0)
    d_open, d_opp = _unit(np.array([0.7, 0.7, 0.0])), _unit(np.array([-0.35, 0.55, -0.75]))
    d0 = _unit((1 - opp) * d_open + opp * d_opp)
    bend = -Z - np.dot(-Z, d0) * d0
    bend = _unit(bend) if np.linalg.norm(bend) > 1e-6 else _unit(np.cross(d0, Z))
    pts = _chain(p[1], d0, bend, (0.040, 0.032, 0.025), (0.0, t_mcp, t_ip))
    p[2], p[3], p[4] = pts
    if mirror:
        p[:, 0] *= -1
    return p


def open_hand(**kw):
    return make_hand({f: (0.05, 0.05, 0.03) for f in MCP_POS}, thumb=(0.0, 0.1, 0.1), **kw)


def fist(**kw):
    return make_hand({f: (1.45, 1.6, 1.0) for f in MCP_POS}, thumb=(0.6, 0.8, 1.0), **kw)


def blend(a, b, s):
    return (1 - s) * a + s * b
