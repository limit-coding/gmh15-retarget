"""Shared per-frame pipeline used by the live camera app and offline replay:

    HandObservation -> features -> closures (calibrated 0..1) -> gate -> One-Euro
                    -> Retargeter -> TargetGuard -> command (or None = hold)
"""
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import features as F
from .filters import OneEuro
from .hand_types import HandFeatures, HandObservation
from .retarget import FEATURE_KEYS, Calibration, LinearCurlRetargeter, Retargeter
from .safety import GateConfig, GuardConfig, TargetGuard, TrackingGate


@dataclass
class StepResult:
    t: float
    valid: bool
    reason: str
    state: str
    features: Optional[HandFeatures] = None
    raw: dict = field(default_factory=dict)
    closure: Optional[np.ndarray] = None        # filtered 0..1
    target: Optional[np.ndarray] = None         # retargeter output (deg)
    command: Optional[np.ndarray] = None        # after guard; None = hold / send nothing
    held: Optional[np.ndarray] = None           # last command the guard holds
    debug: dict = field(default_factory=dict)


class Pipeline:
    def __init__(self, calib: Calibration = None, retargeter: Retargeter = None,
                 gate: GateConfig = None, guard: GuardConfig = None,
                 one_euro=(1.0, 0.3), lo=None, hi=None):
        self.calib = calib or Calibration()
        self.retargeter = retargeter or LinearCurlRetargeter()
        if lo is None and hasattr(self.retargeter, "lo"):
            lo, hi = self.retargeter.lo, self.retargeter.hi
        self.gate = TrackingGate(gate)
        self.guard = TargetGuard(lo, hi, guard)
        self.filter = OneEuro(*one_euro)
        self.raw_history = deque(maxlen=30)     # for calibration snapshots

    def reset(self, q_start=None):
        self.gate.reset()
        self.filter.reset()
        self.guard.reset(q_start)

    def step(self, obs: HandObservation, now: float) -> StepResult:
        feats, raw, closure_raw = None, {}, None
        if obs.detected and obs.world is not None:
            try:
                feats = F.extract(obs.world)
                raw = feats.scalar_vector()
                closure_raw = self.calib.normalize(raw)
                self.raw_history.append(raw)
            except ValueError:
                feats = None
        if obs.detected and feats is None:
            valid, reason = False, "bad_landmarks"
        else:
            valid, reason = self.gate.check(obs, now, closure_raw, feats.hand_scale_m if feats else None)
        closure = target = None
        soft = reason == "jump"
        if valid:
            closure = np.clip(self.filter(closure_raw, obs.t), 0, 1)
            target = self.retargeter(closure)
        elif not soft:
            self.filter.reset()
        cmd = self.guard.update(now, target, valid, reason, soft=soft)
        return StepResult(t=now, valid=valid, reason=reason, state=self.guard.state, features=feats,
                          raw=raw, closure=closure, target=target, command=cmd,
                          held=self.guard.last.copy(), debug=self.retargeter.debug())

    def snapshot_raw(self, n=10):
        """Median of the last n raw feature dicts (for calibration key presses)."""
        hist = list(self.raw_history)[-n:]
        if not hist:
            return None
        return {k: float(np.nanmedian([h[k] for h in hist])) for k in FEATURE_KEYS}
