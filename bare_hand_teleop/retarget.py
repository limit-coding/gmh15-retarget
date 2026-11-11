"""Human hand features -> GMH15 command (11 angles, degrees).

Two layers, both swappable:
  1. Calibration.normalize(): raw features -> 0..1 "closure" per feature (per-operator).
  2. Retargeter: closures -> robot command. LinearCurlRetargeter is the baseline
     (one human feature per robot motor) plus an optional DexPilot-style pinch blend that pulls
     index MCP + thumb joints toward a pose that was measured on the real hand to make
     thumb-index tip contact. A fingertip-optimisation retargeter (dex-retargeting) can be
     dropped in with the same interface once a GMH15 URDF whose kinematics match the real
     hand is available.
"""
import json
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import gmh15_spec as spec

# feature key -> command index it drives in the baseline map
FEATURE_TO_CMD = {
    "index_total": 0, "middle_total": 2, "ring_total": 4, "little_total": 6,
    "thumb_opp": 8, "thumb_mcp": 9, "thumb_ip": 10,
}
FEATURE_KEYS = tuple(FEATURE_TO_CMD) + ("pinch_index",)

# (open/far, closed/contact) raw values. Rough MediaPipe-world-landmark values; meant to be
# overwritten per operator with the in-app calibration (keys o / f / p).
DEFAULT_REFS = {
    "index_total": (0.35, 3.4), "middle_total": (0.35, 3.4),
    "ring_total": (0.35, 3.4), "little_total": (0.35, 3.4),
    "thumb_opp": (1.0, 0.4),          # tip-to-little-MCP ratio shrinks as thumb opposes
    "thumb_mcp": (0.15, 0.9), "thumb_ip": (0.1, 1.1),
    "pinch_index": (0.6, 0.22),       # thumb-index tip distance / hand scale
}


@dataclass
class Calibration:
    refs: dict = field(default_factory=lambda: dict(DEFAULT_REFS))

    def normalize(self, raw: dict) -> np.ndarray:
        out = np.zeros(len(FEATURE_KEYS))
        for i, k in enumerate(FEATURE_KEYS):
            a, b = self.refs[k]
            x = raw.get(k, np.nan)
            out[i] = 0.0 if (not np.isfinite(x) or abs(b - a) < 1e-9) else np.clip((x - a) / (b - a), 0, 1)
        return out

    def set_open(self, raw: dict):
        """Open flat hand: sets the 'open' end of every feature except pinch's contact end."""
        for k in FEATURE_KEYS:
            a, b = self.refs[k]
            self.refs[k] = (float(raw[k]), b)

    def set_fist(self, raw: dict):
        for k in ("index_total", "middle_total", "ring_total", "little_total", "thumb_mcp", "thumb_ip"):
            a, _ = self.refs[k]
            self.refs[k] = (a, float(raw[k]))

    def set_pinch(self, raw: dict):
        """Thumb-index tip contact: sets pinch contact distance and the opposed thumb end."""
        a, _ = self.refs["pinch_index"]
        self.refs["pinch_index"] = (a, float(raw["pinch_index"]))
        a, _ = self.refs["thumb_opp"]
        self.refs["thumb_opp"] = (a, float(raw["thumb_opp"]))

    def save(self, path):
        with open(path, "w") as fh:
            json.dump({"refs": self.refs}, fh, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as fh:
            d = json.load(fh)
        refs = dict(DEFAULT_REFS)
        refs.update({k: tuple(v) for k, v in d["refs"].items()})
        return cls(refs)


@dataclass
class RetargetConfig:
    ranges_deg: dict = field(default_factory=lambda: dict(spec.VERIFIED_RANGE_DEG))
    # Measured on the real hand (e.g. jog in the teleop web UI until thumb and index tips
    # touch, read back the joint angles). Keys are command indices. None disables blending.
    pinch_pose_deg: Optional[dict] = None
    pinch_blend_start: float = 0.5    # pinch closure where blending starts
    pinch_blend_full: float = 1.0     # pinch closure where robot reaches pinch pose

    @classmethod
    def from_json(cls, path):
        with open(path) as fh:
            d = json.load(fh)
        cfg = cls()
        if "ranges_deg" in d:
            cfg.ranges_deg = {int(k): tuple(v) for k, v in d["ranges_deg"].items()}
        if d.get("pinch_pose_deg"):
            cfg.pinch_pose_deg = {int(k): float(v) for k, v in d["pinch_pose_deg"].items()}
        for k in ("pinch_blend_start", "pinch_blend_full"):
            if k in d:
                setattr(cfg, k, float(d[k]))
        return cfg


def _smoothstep(x, a, b):
    if b <= a:
        return float(x >= b)
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return float(t * t * (3 - 2 * t))


class Retargeter:
    """Interface: closures (len(FEATURE_KEYS), 0..1) -> command (11,) degrees."""

    def __call__(self, closure: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def debug(self) -> dict:
        return {}


class LinearCurlRetargeter(Retargeter):
    def __init__(self, cfg: RetargetConfig = None):
        self.cfg = cfg or RetargetConfig()
        self.lo, self.hi = spec.range_arrays(self.cfg.ranges_deg)
        self._w = 0.0

    def __call__(self, closure):
        c = dict(zip(FEATURE_KEYS, closure))
        q = np.zeros(spec.N_CMD)
        for k, i in FEATURE_TO_CMD.items():
            q[i] = self.lo[i] + c[k] * (self.hi[i] - self.lo[i])
        self._w = 0.0
        if self.cfg.pinch_pose_deg:
            self._w = _smoothstep(c["pinch_index"], self.cfg.pinch_blend_start, self.cfg.pinch_blend_full)
            for i, v in self.cfg.pinch_pose_deg.items():
                q[i] = (1 - self._w) * q[i] + self._w * v
        q[list(spec.PASSIVE)] = 0.0
        return np.clip(q, self.lo, self.hi)

    def debug(self):
        return {"pinch_blend": self._w}
