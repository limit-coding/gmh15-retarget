"""Run SimPipeline over the synthetic trajectory (or a run_camera/run_sim JSONL) and plot
human vs robot thumb-index distance, closures and tracking state."""
import argparse, json, os, sys
import numpy as np
ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "scripts"))
from bare_hand_teleop.hand_types import HandObservation
from bare_hand_teleop.opt_retarget import OptConfig, load_calib
from bare_hand_teleop.safety import GateConfig
from bare_hand_teleop.sim_pipeline import SimPipeline
from replay import synthetic_stream

ap = argparse.ArgumentParser()
ap.add_argument("--jsonl"); ap.add_argument("--path", default="measured")
ap.add_argument("--mirror-world", action="store_true", help="set if the recording was made with a flipped image")
ap.add_argument("--calib", help="closure calibration JSON (config/sim_calib_right.json)")
ap.add_argument("--out", default=os.path.join(ROOT, "runs/sim_replay.png"))
a = ap.parse_args()

def stream():
    if a.jsonl:
        for line in open(a.jsonl):
            o = json.loads(line)["obs"]
            yield HandObservation(t=o["t"], detected=o["detected"], handedness=o["handedness"], score=o["score"],
                                  world=None if o["world"] is None else np.array(o["world"]))
    else:
        for o in synthetic_stream():
            if o.world is not None:
                o.world = o.world.copy(); o.world[:, 0] *= -1      # right-hand chirality
            yield o

opt = OptConfig(path=a.path)
if a.calib:
    load_calib(opt, a.calib)
p = SimPipeline("right", opt=opt, gate=GateConfig(expected_handedness=None),
                mirror_world=a.mirror_world)
T, hp, rp, S, st, held = [], [], [], [], [], []
for o in stream():
    r = p.step(o, o.t)
    T.append(o.t); st.append(r.state); held.append(r.held.copy())
    hp.append(r.info.get("human_pinch_m", {}).get("index", np.nan) if r.info else np.nan)
    rp.append(r.info.get("robot_pinch_m", {}).get("index", np.nan) if r.info else np.nan)
    S.append(r.x[:4] if r.x is not None else [np.nan] * 4)
T, hp, rp, S, held = map(np.array, (T, hp, rp, S, held))
rate = np.nanmax(np.abs(np.diff(held, axis=0)) / np.diff(T)[:, None])
print(f"frames={len(T)} tracking={st.count('TRACKING')} lost={st.count('LOST')} max_rate={rate:.0f} deg/s")
m = hp < 0.02
if m.any(): print(f"human pinch <20 mm on {m.sum()} frames: robot distance median {np.nanmedian(rp[m])*1000:.1f} mm, max {np.nanmax(rp[m])*1000:.1f} mm")
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
ax[0].plot(T, hp * 1000, label="human thumb-index"); ax[0].plot(T, rp * 1000, label="robot thumb-index (pad centroids)")
ax[0].set_ylabel("mm"); ax[0].legend(); ax[0].set_yscale("log")
for k, n in enumerate(("index", "middle", "ring", "pinky")): ax[1].plot(T, S[:, k], label=n)
ax[1].set_ylabel("finger closure s"); ax[1].legend(fontsize=8)
for i, n in ((0, "index j1"), (1, "index j2"), (8, "thumb0"), (9, "thumb1"), (10, "thumb2")): ax[2].plot(T, held[:, i], label=n)
ax[2].set_ylabel("command deg (held)"); ax[2].legend(fontsize=8, ncol=3); ax[2].set_xlabel("t (s)")
fig.tight_layout(); fig.savefig(a.out, dpi=110); print("plot ->", a.out)
