"""Quantitative metrics for a run_sim JSONL recording (flipped image => --mirror-world default on).

  .venv/bin/python tools/eval_recording.py runs/sim_live2.jsonl [--calib config/sim_calib_right.json]

Prints: tracking counts, open-hand closure (10th percentile of s), pinch fidelity, thumb jumpiness
(frames where a thumb command moves > 20 deg between consecutive frames, total variation deg/s).
"""
import argparse, json, os, sys
import numpy as np
ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)
from bare_hand_teleop.hand_types import HandObservation          # noqa: E402
from bare_hand_teleop.opt_retarget import OptConfig, load_calib   # noqa: E402
from bare_hand_teleop.safety import GateConfig                    # noqa: E402
from bare_hand_teleop.sim_pipeline import SimPipeline             # noqa: E402


def run(path, cfg=None, mirror=True):
    p = SimPipeline("right", opt=cfg or OptConfig(), gate=GateConfig(expected_handedness=None), mirror_world=mirror)
    rows = []
    for line in open(path):
        o = json.loads(line)["obs"]
        obs = HandObservation(t=o["t"], detected=o["detected"], handedness=o["handedness"], score=o["score"] or 0.0,
                              world=None if o["world"] is None else np.array(o["world"]))
        r = p.step(obs, o["t"])
        rows.append(dict(t=o["t"], state=r.state, x=r.x, held=r.held, info=r.info))
    return rows


def metrics(rows):
    tr = [r for r in rows if r["x"] is not None]
    X = np.array([r["x"] for r in tr]); H = np.array([r["held"] for r in rows]); T = np.array([r["t"] for r in rows])
    hp = np.array([r["info"]["human_pinch_m"]["index"] for r in tr]); rp = np.array([r["info"]["robot_pinch_m"]["index"] for r in tr])
    dth = np.abs(np.diff(H[:, 8:11], axis=0))
    m20 = hp < 0.02
    return dict(
        tracked=len(tr), frames=len(rows),
        open_s_p10=np.round(np.percentile(X[:, :4], 10, axis=0), 3).tolist(),
        fist_s_p95=np.round(np.percentile(X[:, :4], 95, axis=0), 3).tolist(),
        pinch20_n=int(m20.sum()),
        pinch20_robot_mm=(round(float(np.median(rp[m20]) * 1000), 1), round(float(rp[m20].max() * 1000), 1)) if m20.any() else None,
        thumb_jumps_gt20=int((dth > 20).any(axis=1).sum()),
        thumb_tv_deg_s=np.round(dth.sum(axis=0) / (T[-1] - T[0]), 1).tolist(),
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl"); ap.add_argument("--calib"); ap.add_argument("--no-mirror", action="store_true")
    a = ap.parse_args()
    cfg = OptConfig()
    if a.calib:
        load_calib(cfg, a.calib)
    m = metrics(run(a.jsonl, cfg, not a.no_mirror))
    for k, v in m.items():
        print(f"{k:18s} {v}")
