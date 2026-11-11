#!/usr/bin/env python3
"""Offline validation: run the pipeline on a recorded JSONL (from run_camera.py --record) or on a
synthetic trajectory, write a CSV of human closure / target / command, and print a summary.

  python scripts/replay.py --synthetic --out runs/synthetic.csv --plot runs/synthetic.png
  python scripts/replay.py --jsonl runs/clip.jsonl --calib config/calib_right.json --out runs/clip.csv
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bare_hand_teleop import gmh15_spec as spec, synthetic as S            # noqa: E402
from bare_hand_teleop.hand_types import HandObservation                    # noqa: E402
from bare_hand_teleop.pipeline import Pipeline                              # noqa: E402
from bare_hand_teleop.retarget import FEATURE_KEYS, Calibration, LinearCurlRetargeter, RetargetConfig  # noqa: E402
from bare_hand_teleop.safety import GateConfig                              # noqa: E402


def synthetic_pinch():
    p = S.make_hand({"index": (0.55, 0.65, 0.35), "middle": (0.15, 0.1, 0.05),
                     "ring": (0.15, 0.1, 0.05), "little": (0.15, 0.1, 0.05)}, thumb=(0.45, 0.25, 0.2))
    p[4] = p[8] + np.array([0.0, 0.0, 0.006])                   # thumb tip 6 mm from index tip
    return p


def synthetic_stream(fps=30, noise_m=0.002, seed=0):
    """open -> fist -> open -> pinch -> (tracking dropout while hand moves) -> open."""
    rng = np.random.default_rng(seed)
    op, fi, pinch = S.open_hand(), S.fist(), synthetic_pinch()
    segs = [(op, op, 0.5), (op, fi, 1.0), (fi, fi, 0.5), (fi, op, 1.0), (op, pinch, 1.0),
            (pinch, pinch, 0.7), (pinch, fi, 0.6), (fi, op, 1.0), (op, op, 0.7)]
    t, drop = 0.0, (5.9, 6.5)      # hand lost while it closes to a fist
    for a, b, dur in segs:
        n = int(dur * fps)
        for k in range(n):
            s = 0.5 - 0.5 * np.cos(np.pi * k / max(n - 1, 1))
            w = S.blend(a, b, s) + rng.normal(0, noise_m, (21, 3))
            det = not (drop[0] <= t < drop[1])
            yield HandObservation(t=t, detected=det, handedness="Right", score=0.95,
                                  world=w if det else None, image=None)
            t += 1.0 / fps


def jsonl_stream(path):
    with open(path) as fh:
        for line in fh:
            o = json.loads(line)["obs"]
            yield HandObservation(t=o["t"], detected=o["detected"], handedness=o["handedness"], score=o["score"],
                                  world=None if o["world"] is None else np.array(o["world"]),
                                  image=None if o["image"] is None else np.array(o["image"]))


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--jsonl")
    g.add_argument("--synthetic", action="store_true")
    ap.add_argument("--calib")
    ap.add_argument("--retarget-config")
    ap.add_argument("--side", default="Right")
    ap.add_argument("--out", default="runs/replay.csv")
    ap.add_argument("--plot")
    a = ap.parse_args()

    calib = Calibration.load(a.calib) if a.calib else Calibration()
    if a.synthetic and not a.calib:
        # Mimic the in-app o/f/p calibration on the synthetic hand's reference poses.
        from bare_hand_teleop import features as F
        calib.set_open(F.extract(S.open_hand()).scalar_vector())
        calib.set_fist(F.extract(S.fist()).scalar_vector())
        calib.set_pinch(F.extract(synthetic_pinch()).scalar_vector())
    rcfg = RetargetConfig.from_json(a.retarget_config) if a.retarget_config else RetargetConfig()
    pipe = Pipeline(calib, LinearCurlRetargeter(rcfg), GateConfig(expected_handedness=a.side))
    stream = synthetic_stream() if a.synthetic else jsonl_stream(a.jsonl)

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    rows, prev_cmd, prev_t, max_rate = [], None, None, 0.0
    states = {}
    for obs in stream:
        r = pipe.step(obs, obs.t)
        states[r.state] = states.get(r.state, 0) + 1
        q = r.held
        if prev_cmd is not None and r.t > prev_t:
            max_rate = max(max_rate, float(np.max(np.abs(q - prev_cmd)) / (r.t - prev_t)))
        prev_cmd, prev_t = q.copy(), r.t
        row = {"t": round(r.t, 4), "state": r.state, "reason": r.reason, "sent": int(r.command is not None),
               "pinch_m": "" if r.features is None else round(r.features.pinch_m["index"], 4),
               "pinch_blend": round(r.debug.get("pinch_blend", 0.0), 3)}
        for i, k in enumerate(FEATURE_KEYS):
            row["c_" + k] = "" if r.closure is None else round(float(r.closure[i]), 4)
        for i in spec.ACTUATED:
            row["cmd_" + spec.JOINT_NAMES[i]] = round(float(q[i]), 3)
        rows.append(row)
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"frames={len(rows)} states={states} max_cmd_rate={max_rate:.1f} deg/s -> {a.out}")
    for col in ("cmd_index_mcp", "cmd_thumb_cmc"):
        v = [r[col] for r in rows]
        print(f"  {col}: min={min(v):.1f} max={max(v):.1f}")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = np.array([r["t"] for r in rows])
        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for k in ("index_total", "middle_total", "thumb_opp", "pinch_index"):
            ax[0].plot(t, [np.nan if r["c_" + k] == "" else r["c_" + k] for r in rows], label=k)
        ax[0].set_ylabel("human closure")
        ax[0].legend(loc="upper right", fontsize=8)
        for i in spec.ACTUATED:
            n = spec.JOINT_NAMES[i]
            ax[1].plot(t, [r["cmd_" + n] for r in rows], label=n)
        ax[1].set_ylabel("command deg")
        ax[1].legend(loc="upper right", fontsize=7, ncol=2)
        code = {"TRACKING": 2, "REACQUIRING": 1, "LOST": 0}
        ax[2].step(t, [code[r["state"]] for r in rows], where="post")
        ax[2].set_yticks([0, 1, 2], ["LOST", "REACQ", "TRACK"])
        ax[2].set_xlabel("t (s)")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print("plot ->", a.plot)


if __name__ == "__main__":
    main()
