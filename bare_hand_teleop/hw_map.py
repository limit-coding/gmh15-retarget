"""Sim (full URDF range) -> hardware command (conservative verified range).

The sim shows the full retargeting result. The hardware receives the same motion scaled into its
verified range instead of being clipped (clipping froze joints at the limit, e.g. thumb MCP sat at
30 deg 99% of the time):
  fingers: closure s in [0, 1] along the finger path -> MCP = s * hw_hi, PIP = 0
           (one tendon motor per finger; the hardware's own coupling bends PIP)
  thumb  : angle * hw_hi / urdf_hi (CMC, MCP, IP each)
"""
import numpy as np

from . import gmh15_spec as spec
from .opt_retarget import FINGER_PATHS

FINGER_MCP = (0, 2, 4, 6)
THUMB = (8, 9, 10)
NAMES = ("index", "middle", "ring", "little", "thumb CMC", "thumb MCP", "thumb IP")


def finger_closure(cmd11, path="measured"):
    """Recover s for each finger from its (MCP, PIP) pair on the path table."""
    tab = FINGER_PATHS[path]
    grid = np.linspace(0, 1, 401)
    j0 = np.interp(grid, tab[:, 0], tab[:, 1])
    j1 = np.interp(grid, tab[:, 0], tab[:, 2])
    out = []
    for k in range(4):
        i = int(np.argmin((j0 - cmd11[2 * k]) ** 2 + (j1 - cmd11[2 * k + 1]) ** 2))
        out.append(float(grid[i]))
    return np.array(out)


def hw_from_sim(cmd11, hw_lo, hw_hi, path="measured"):
    cmd11 = np.asarray(cmd11, float)
    lo_u, hi_u = spec.range_arrays(spec.URDF_LIMIT_DEG)
    out = np.zeros(spec.N_CMD)
    s = finger_closure(cmd11, path)
    for k, i in enumerate(FINGER_MCP):
        out[i] = hw_lo[i] + s[k] * (hw_hi[i] - hw_lo[i])
    for i in THUMB:
        frac = (cmd11[i] - lo_u[i]) / (hi_u[i] - lo_u[i])
        out[i] = hw_lo[i] + np.clip(frac, 0, 1) * (hw_hi[i] - hw_lo[i])
    out[list(spec.PASSIVE)] = 0.0
    return np.clip(out, hw_lo, hw_hi)


def seven(cmd11):
    """11-joint SDK order -> the 7 driven DOF (finger MCPs + thumb)."""
    c = np.asarray(cmd11, float)
    return [float(c[i]) for i in FINGER_MCP + THUMB]


# ---------------------------------------------------------------------------------------------
# Raw motor mode (full range). Tables measured 2026-11-07 with scripts/motor_range_probe.py
# (runs/motor_range.jsonl): SDK forward-kinematics joint angle (deg) vs motor counts.
# Fingers (tendon): counts 0-~6700 bend MCP 0->80; above that MCP stays 80 and PIP/DIP curl.
FINGER_MCP_TABLE = ((0, 0), (10, 712), (20, 1457), (30, 2272), (40, 3193), (50, 4099),
                    (60, 5084), (70, 5962), (79, 6465), (80, 6737))
FINGER_PIP_TABLE = ((0, 6737), (10, 7063), (20, 7362), (40, 8112), (60, 9041), (80, 9984))
THUMB_TABLES = {   # motor 5 CMC, 6 MCP, 7 IP
    8: ((0, 0), (20, 1615), (40, 3563), (60, 5528), (80, 7367), (100, 9128), (109.2, 9963)),
    9: ((0, 0), (20, 2047), (40, 4330), (60, 6818), (65.4, 7494)),
    10: ((0, 0), (20, 2068), (40, 4120), (60, 5915), (80, 7413), (88.7, 7996)),
}
# Software ceilings: thumb MCP/IP hit a mechanical end (current > 500) at ~7500 / ~8000.
MOTOR_HI = (10000, 10000, 10000, 10000, 10000, 7200, 7700)


def _interp(x, table):
    t = np.asarray(table, float)
    return float(np.interp(x, t[:, 0], t[:, 1]))


def counts_from_sim(cmd11):
    """Sim command (SDK 11-joint order, URDF degrees, finger PIP included) -> 7 motor counts."""
    c = np.asarray(cmd11, float)
    # The "measured" finger path is parameterised by s = counts / 10000 (same sweep data).
    out = list(finger_closure(c, "measured") * 10000.0)
    out += [_interp(c[i], THUMB_TABLES[i]) for i in THUMB]
    return [int(round(min(max(v, 0), hi))) for v, hi in zip(out, MOTOR_HI)]
