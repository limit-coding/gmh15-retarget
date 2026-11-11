"""Render synthetic open / fist / pinch retargeted onto the GMH15 model, for both finger paths."""
import os, sys
import numpy as np, cv2
ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "scripts"))
from bare_hand_teleop.gmh15_model import GMH15Model
from bare_hand_teleop.opt_retarget import GMH15VectorRetargeter, OptConfig
from bare_hand_teleop.sim import HandSim
from bare_hand_teleop import synthetic as S
from replay import synthetic_pinch

m = GMH15Model(os.path.join(ROOT, "vendor/yubi-pi_gmh15-sdk/models/right/meshes/DexH15_right_urdf.urdf"))
sim = HandSim(size=(360, 360))
pinch = synthetic_pinch(); pinch[:, 0] *= -1            # right-hand chirality
poses = {"open": S.open_hand(mirror=True), "fist": S.fist(mirror=True), "pinch": pinch}
rows = []
for path in ("mcp_first", "synergy"):
    r = GMH15VectorRetargeter(m, OptConfig(path=path))
    sim.cam.lookat[:] = r.R.T @ np.array([0.0, 0.09, 0.0])
    sim.look_from(r.R.T @ np.array([0.25, -0.35, 1.0]))   # palm side, a little from the thumb side and below
    imgs = []
    for name, w in poses.items():
        r.reset()
        for _ in range(5):
            x = r.solve(w)
        L = r.last
        print(f"{path:9s} {name:5s} s={np.round(x[:4], 2)} thumb={np.round(x[4:], 1)}  "
              f"human pinch {L['human_pinch_m']['index'] * 1000:5.1f} mm -> robot {L['robot_pinch_m']['index'] * 1000:5.1f} mm")
        sim.set_q(r.to_urdf_q(x)); tp = r.tips_palm(x)
        img = sim.render([(tp["thumb"], [1, 0, 1, 1]), (tp["index"], [0, 1, 1, 1])]).copy()
        cv2.putText(img, f"{path} {name}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
        imgs.append(img)
    rows.append(np.hstack(imgs))
cv2.imwrite(os.path.join(ROOT, "runs/sim_snapshots.png"), np.vstack(rows)[:, :, ::-1])
