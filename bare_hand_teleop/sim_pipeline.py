"""Per-frame pipeline for the optimisation retargeter (sim stage):

  HandObservation -> gate -> GMH15VectorRetargeter (7 vars) -> One-Euro on x -> cmd11 (SDK order)
                  -> TargetGuard (URDF limits, rate limit, hold on loss)

cmd11 here is the SDK 11-joint order with finger joint2 (PIP) included — it targets the sim, not
the current teleop server (which still forces PIP = 0).
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import features as F
from . import gmh15_spec as spec
from .filters import MotionShaper, OneEuro
from .gmh15_model import GMH15Model
from .hand_types import HandObservation
from .opt_retarget import GMH15VectorRetargeter, OptConfig
from .retarget import Calibration
from .safety import GateConfig, GuardConfig, TargetGuard, TrackingGate

DEFAULT_URDF = "vendor/yubi-pi_gmh15-sdk/models/{side}/meshes/DexH15_{side}_urdf.urdf"


def chirality_score(world):
    """> 0: fingers curl toward the side a RIGHT hand's palm faces (canonical frame +z).
    Used to catch a wrong mirror setting (left-hand geometry fed to a right robot)."""
    from .opt_retarget import frame_from
    p = np.asarray(world, float)
    R = frame_from(p[5], p[17], p[9] - p[0])
    return float(np.mean([(R @ (p[t] - p[m]))[2] for m, t in ((5, 8), (9, 12), (13, 16), (17, 20))]))


@dataclass
class SimStep:
    t: float
    valid: bool
    reason: str
    state: str
    x: Optional[np.ndarray] = None
    command: Optional[np.ndarray] = None
    held: Optional[np.ndarray] = None
    closure: Optional[np.ndarray] = None
    info: dict = field(default_factory=dict)


class SimPipeline:
    def __init__(self, side="right", urdf=None, opt: OptConfig = None, gate: GateConfig = None,
                 guard: GuardConfig = None, one_euro=(1.5, 0.5), mirror_world=False,
                 shaper: MotionShaper = None):
        self.side = side
        self.model = GMH15Model(urdf or DEFAULT_URDF.format(side=side))
        self.rt = GMH15VectorRetargeter(self.model, opt)
        lo, hi = spec.range_arrays(spec.HARDWARE_LIMIT_DEG)
        self.guard = TargetGuard(lo, hi, guard or GuardConfig(max_rate_deg_s=360.0), zero_passive=False)
        self.gate = TrackingGate(gate)
        self.calib = Calibration()           # only for the gate's jump check / display closures
        self.filter = OneEuro(*one_euro)
        self.mirror_world = mirror_world      # set True when the image was flipped before tracking
        # final smoothing stage (velocity + acceleration limited); None disables
        self.shaper = MotionShaper() if shaper is None else (shaper or None)

    def reset(self, q_start=None):
        """q_start: measured robot command-space position (hardware arm) so output starts there."""
        self.gate.reset(); self.filter.reset(); self.guard.reset(q_start); self.rt.reset()
        if self.shaper:
            self.shaper.reset(None if q_start is None else self.guard.last)

    def step(self, obs: HandObservation, now: float) -> SimStep:
        world = None
        closure = feats = None
        if obs.detected and obs.world is not None:
            world = np.array(obs.world, float)
            if self.mirror_world:
                world[:, 0] *= -1
            try:
                feats = F.extract(world)
                closure = self.calib.normalize(feats.scalar_vector())
            except ValueError:
                world = None
        if obs.detected and world is None:
            valid, reason = False, "bad_landmarks"
        else:
            valid, reason = self.gate.check(obs, now, closure, feats.hand_scale_m if feats else None)
        soft = reason == "jump"
        x = target = None
        info = {}
        if valid:
            x_raw = self.rt.solve(world)
            x = self.filter(x_raw, obs.t)
            target = self.rt.to_cmd11(x)
            info = dict(self.rt.last, chirality=chirality_score(world))
        elif not soft:
            self.filter.reset()
            self.rt.reset()
        cmd = self.guard.update(now, target, valid, reason, soft=soft)
        held = self.guard.last.copy()
        if self.shaper:
            # shaped output follows the guard's command (or its held value while lost)
            shaped = self.shaper(cmd if cmd is not None else held, now)
            held = shaped
            if cmd is not None:
                cmd = shaped
        return SimStep(t=now, valid=valid, reason=reason, state=self.guard.state, x=x, command=cmd,
                       held=held, closure=closure, info=info)

    def urdf_q_from_cmd(self, cmd11):
        """cmd11 (deg, SDK order) -> URDF joints for the sim; finger joint_2 follows the path."""
        q = self.model.q_from_cmd(cmd11)
        path = self.rt.cfg.path
        from .opt_retarget import FINGER_PATHS
        tab = FINGER_PATHS[path]
        for k, f in enumerate(("index", "middle", "ring", "pinky")):
            # recover s from (joint0, joint1) along the path, then joint2
            s_grid = np.linspace(0, 1, 201)
            j0 = np.interp(s_grid, tab[:, 0], tab[:, 1]); j1 = np.interp(s_grid, tab[:, 0], tab[:, 2])
            i = int(np.argmin((j0 - cmd11[2 * k]) ** 2 + (j1 - cmd11[2 * k + 1]) ** 2))
            q[f"{self.side}_{f}_joint_2"] = np.deg2rad(np.interp(s_grid[i], tab[:, 0], tab[:, 3]))
        return q

    def tips_from_cmd(self, cmd11):
        q = self.urdf_q_from_cmd(cmd11)
        return {f: self.model.tip(f, q) for f in ("thumb", "index", "middle", "ring", "pinky")}
