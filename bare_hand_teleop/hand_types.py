from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class HandObservation:
    """One tracker output for one hand. Coordinates follow MediaPipe's 21-landmark layout."""
    t: float                                  # capture time, seconds (monotonic clock)
    detected: bool
    handedness: Optional[str] = None          # "Left" / "Right" as seen by the operator
    score: float = 0.0
    world: Optional[np.ndarray] = None        # (21,3) metric, hand-centred
    image: Optional[np.ndarray] = None        # (21,3) x,y normalised to image, z relative


@dataclass
class HandFeatures:
    """Mirror-invariant scalars extracted from landmarks (radians / ratios)."""
    finger_mcp: dict = field(default_factory=dict)
    finger_pip: dict = field(default_factory=dict)
    finger_dip: dict = field(default_factory=dict)
    finger_total: dict = field(default_factory=dict)   # mcp+pip+dip per finger
    thumb_mcp: float = 0.0
    thumb_ip: float = 0.0
    thumb_opp: float = 0.0          # |thumb tip - little MCP| / hand scale (small = opposed)
    pinch_ratio: dict = field(default_factory=dict)    # |thumb tip - finger tip| / hand scale
    pinch_m: dict = field(default_factory=dict)        # same in metres (world landmarks)
    hand_scale_m: float = 0.0       # |wrist - middle MCP|

    def scalar_vector(self):
        """Named scalars that feed the 0..1 normalisation (see retarget.FEATURE_KEYS)."""
        d = {f"{k}_total": v for k, v in self.finger_total.items()}
        d.update(thumb_mcp=self.thumb_mcp, thumb_ip=self.thumb_ip, thumb_opp=self.thumb_opp,
                 pinch_index=self.pinch_ratio.get("index", np.nan))
        return d
