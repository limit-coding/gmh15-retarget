"""Tracking validity gate + command guard (clamp, rate limit, hold on loss, soft resume).

These run on the client side regardless of driver. They do not replace hardware-side
protection (server limits, slew, heartbeat watchdog) — they are an extra layer.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import gmh15_spec as spec
from .hand_types import HandObservation


@dataclass
class GateConfig:
    expected_handedness: Optional[str] = None   # "Left"/"Right" or None to accept any
    min_score: float = 0.6
    max_age_s: float = 0.15                     # frame older than this -> stale
    scale_range_m: tuple = (0.04, 0.16)         # plausible wrist->middle-MCP length
    max_jump: float = 0.5                       # per-frame closure change (0..1) counted as a jump
    min_jump_features: int = 4                  # ...in at least this many features at once = glitch


class TrackingGate:
    def __init__(self, cfg: GateConfig = None):
        self.cfg = cfg or GateConfig()
        self._prev = None

    def reset(self):
        self._prev = None

    def check(self, obs: HandObservation, now: float, closure=None, hand_scale=None):
        """Returns (valid, reason). reason == "jump" is a soft reject: skip one frame only."""
        c = self.cfg
        if not obs.detected:
            self._prev = None
            return False, "no_hand"
        if c.expected_handedness and obs.handedness != c.expected_handedness:
            self._prev = None
            return False, f"wrong_hand({obs.handedness})"
        if obs.score < c.min_score:
            return False, f"low_score({obs.score:.2f})"
        if now - obs.t > c.max_age_s:
            return False, f"stale({(now - obs.t) * 1000:.0f}ms)"
        if hand_scale is not None and not (c.scale_range_m[0] <= hand_scale <= c.scale_range_m[1]):
            return False, f"bad_scale({hand_scale:.3f})"
        if closure is not None:
            prev, self._prev = self._prev, np.asarray(closure, float).copy()
            # Single noisy features (e.g. distal angles in a tight fist) are left to the filter; a
            # simultaneous jump of many features looks like a misdetection. Rejected for one frame;
            # if the new value persists it is accepted on the next frame.
            if prev is not None and np.sum(np.abs(self._prev - prev) > c.max_jump) >= c.min_jump_features:
                return False, "jump"
        return True, "ok"


@dataclass
class GuardConfig:
    max_rate_deg_s: float = 90.0     # per joint
    reacquire_frames: int = 5        # consecutive valid frames before resuming after loss
    resume_ramp_s: float = 0.6       # rate limit ramps from resume_min_scale to 1 over this
    resume_min_scale: float = 0.25
    max_dt_s: float = 0.1            # dt used for rate limiting is capped (no big jump after gaps)


class TargetGuard:
    """States: TRACKING (sending), LOST (hold: return None), REACQUIRING (hold, counting)."""

    def __init__(self, lo=None, hi=None, cfg: GuardConfig = None, zero_passive=True):
        if lo is None or hi is None:
            lo, hi = spec.range_arrays()
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        self.cfg = cfg or GuardConfig()
        self.zero_passive = zero_passive    # False for sim: finger joint2 (PIP) is part of the command
        self.reset()

    def reset(self, q_start=None):
        """q_start: measured robot position when known (hardware); defaults to open (lo)."""
        self.last = self._clamp(self.lo.copy() if q_start is None else np.asarray(q_start, float))
        self.last_t = None
        self.state = "REACQUIRING"
        self.valid_count = 0
        self.resume_t = None
        self.reason = "start"

    def _clamp(self, q):
        q = np.clip(q, self.lo, self.hi)
        if self.zero_passive:
            q[list(spec.PASSIVE)] = 0.0
        return q

    def update(self, t, target, valid, reason="ok", soft=False):
        """Returns the command to send, or None meaning: do not send a new target (hold).
        soft=True: skip this frame (hold) without leaving TRACKING (single-frame glitch)."""
        self.reason = reason
        if soft and not valid and self.state == "TRACKING":
            self.last_t = t
            return None
        if not valid or target is None:
            self.state = "LOST"
            self.valid_count = 0
            self.last_t = t
            return None
        self.valid_count += 1
        if self.state != "TRACKING":
            if self.valid_count < self.cfg.reacquire_frames:
                self.state = "REACQUIRING"
                self.last_t = t
                return None
            self.state = "TRACKING"
            self.resume_t = t
        dt = self.cfg.max_dt_s if self.last_t is None else float(np.clip(t - self.last_t, 0.0, self.cfg.max_dt_s))
        scale = 1.0
        if self.resume_t is not None and self.cfg.resume_ramp_s > 0:
            k = np.clip((t - self.resume_t) / self.cfg.resume_ramp_s, 0.0, 1.0)
            scale = self.cfg.resume_min_scale + (1 - self.cfg.resume_min_scale) * k
        step = self.cfg.max_rate_deg_s * scale * dt
        tgt = self._clamp(np.asarray(target, float))
        self.last = self._clamp(self.last + np.clip(tgt - self.last, -step, step))
        self.last_t = t
        return self.last.copy()
