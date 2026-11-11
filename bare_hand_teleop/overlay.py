"""OpenCV overlay: landmarks, tracking/guard state, per-finger closure, pinch, robot targets."""
import cv2
import numpy as np

from . import gmh15_spec as spec
from .retarget import FEATURE_KEYS

CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
               (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]
STATE_COLOR = {"TRACKING": (60, 200, 60), "REACQUIRING": (0, 200, 255), "LOST": (40, 40, 230)}
LABELS = {"index_total": "index", "middle_total": "middle", "ring_total": "ring", "little_total": "little",
          "thumb_opp": "th_opp", "thumb_mcp": "th_mcp", "thumb_ip": "th_ip", "pinch_index": "pinch"}


def _bar(img, x, y, w, h, frac, color, label, value_txt=""):
    cv2.rectangle(img, (x, y), (x + w, y + h), (80, 80, 80), 1)
    if frac is not None and np.isfinite(frac):
        cv2.rectangle(img, (x, y), (x + int(w * float(np.clip(frac, 0, 1))), y + h), color, -1)
    cv2.putText(img, label, (x - 78, y + h - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1)
    if value_txt:
        cv2.putText(img, value_txt, (x + w + 6, y + h - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1)


def draw(frame, obs, res, lo, hi, driver_status=None, fps=None, note=""):
    h, w = frame.shape[:2]
    out = frame.copy()
    if obs is not None and obs.detected and obs.image is not None:
        pts = [(int(x * w), int(y * h)) for x, y, _ in obs.image]
        col = STATE_COLOR.get(res.state, (200, 200, 200))
        for a, b in CONNECTIONS:
            cv2.line(out, pts[a], pts[b], col, 2)
        for i, pt in enumerate(pts):
            cv2.circle(out, pt, 5 if i in (4, 8) else 3, (255, 255, 255) if i in (4, 8) else col, -1)
        cv2.line(out, pts[4], pts[8], (255, 0, 255), 1)
    panel = np.zeros((h, 330, 3), np.uint8)
    st_col = STATE_COLOR.get(res.state, (200, 200, 200))
    cv2.putText(panel, f"{res.state}  {res.reason}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, st_col, 2)
    hand_txt = f"hand={obs.handedness} score={obs.score:.2f}" if obs and obs.detected else "hand=none"
    cv2.putText(panel, hand_txt + (f"  {fps:.0f}fps" if fps else ""), (8, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
    y = 62
    cv2.putText(panel, "human closure (0 open .. 1 closed)", (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)
    y += 8
    for i, k in enumerate(FEATURE_KEYS):
        frac = None if res.closure is None else res.closure[i]
        _bar(panel, 84, y, 160, 12, frac, (200, 160, 60), LABELS[k], "" if frac is None else f"{frac:.2f}")
        y += 17
    if res.features is not None:
        f = res.features
        cv2.putText(panel, f"pinch idx {f.pinch_m['index'] * 100:.1f}cm  ratio {f.pinch_ratio['index']:.2f}",
                    (8, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1)
    cv2.putText(panel, f"pinch blend w={res.debug.get('pinch_blend', 0):.2f}", (8, y + 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1)
    y += 44
    cv2.putText(panel, "GMH15 cmd deg (held if not TRACKING)", (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)
    y += 8
    q = res.command if res.command is not None else res.held
    for i in spec.ACTUATED:
        frac = (q[i] - lo[i]) / (hi[i] - lo[i]) if hi[i] > lo[i] else 0
        _bar(panel, 84, y, 160, 12, frac, st_col, spec.JOINT_NAMES[i][:9], f"{q[i]:.1f}")
        y += 17
    if driver_status:
        y += 6
        txt = f"{driver_status.get('driver')} armed={driver_status.get('armed')}"
        if driver_status.get("fault"):
            txt += f" FAULT {driver_status['fault']}"
        cv2.putText(panel, txt[:48], (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)
        if driver_status.get("measured"):
            cv2.putText(panel, "meas " + " ".join(f"{driver_status['measured'][i]:.0f}" for i in spec.ACTUATED),
                        (8, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
    cv2.putText(panel, note[:50], (8, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)
    cv2.putText(panel, "q quit o open f fist p pinch s save a arm", (8, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)
    return np.hstack([out, panel])
