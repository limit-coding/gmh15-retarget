#!/usr/bin/env python3
"""Live single-hand prototype: camera -> MediaPipe -> retarget -> guard -> driver (dry-run by default).

  python scripts/run_camera.py --model models/hand_landmarker.task --side right
  python scripts/run_camera.py --video clip.mp4 --record runs/clip.jsonl

Hardware (real GMH15 moves!) needs ALL of: --driver gmh15-http --hardware --i-have-checked-safety,
then pressing `a` in the window. Without `--hardware`, gmh15-http only reads state.
Keys: q quit | o capture open hand | f capture fist | p capture thumb-index pinch | s save calib
      | a arm hardware | space toggle send-pause (pause = hold, same as tracking loss)
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bare_hand_teleop import overlay                                    # noqa: E402
from bare_hand_teleop.drivers import DriverFault, DryRunDriver, GMH15HttpDriver  # noqa: E402
from bare_hand_teleop.pipeline import Pipeline                          # noqa: E402
from bare_hand_teleop.retarget import Calibration, LinearCurlRetargeter, RetargetConfig  # noqa: E402
from bare_hand_teleop.safety import GateConfig, GuardConfig             # noqa: E402
from bare_hand_teleop.tracking import MediaPipeTracker, select_hand     # noqa: E402


def obs_to_json(o):
    return {"t": o.t, "detected": o.detected, "handedness": o.handedness, "score": o.score,
            "world": None if o.world is None else np.round(o.world, 5).tolist(),
            "image": None if o.image is None else np.round(o.image, 5).tolist()}


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--camera", type=int, default=0)
    src.add_argument("--video")
    ap.add_argument("--model", default="models/hand_landmarker.task")
    ap.add_argument("--side", choices=["right", "left"], default="right", help="operator hand == GMH15 side")
    ap.add_argument("--no-flip", action="store_true",
                    help="do not mirror the image (use when the camera is NOT facing the operator selfie-style)")
    ap.add_argument("--calib", default=None, help="calibration JSON (default config/calib_<side>.json if present)")
    ap.add_argument("--retarget-config", default=None, help="e.g. config/gmh15_right.json")
    ap.add_argument("--record", help="write per-frame JSONL (landmarks + outputs) for offline replay")
    ap.add_argument("--max-rate", type=float, default=90.0, help="deg/s per joint, client side")
    ap.add_argument("--driver", choices=["dry", "gmh15-http"], default="dry")
    ap.add_argument("--url", default="http://127.0.0.1:8765")
    ap.add_argument("--hardware", action="store_true")
    ap.add_argument("--i-have-checked-safety", action="store_true")
    ap.add_argument("--headless", action="store_true", help="no window (video replay / benchmarking)")
    a = ap.parse_args()

    side_label = a.side.capitalize()
    calib_path = a.calib or f"config/calib_{a.side}.json"
    calib = Calibration.load(calib_path) if os.path.exists(calib_path) else Calibration()
    rcfg = RetargetConfig.from_json(a.retarget_config) if a.retarget_config else RetargetConfig()
    retargeter = LinearCurlRetargeter(rcfg)
    pipe = Pipeline(calib, retargeter, GateConfig(expected_handedness=side_label),
                    GuardConfig(max_rate_deg_s=a.max_rate))

    if a.driver == "gmh15-http":
        hw = a.hardware and a.i_have_checked_safety
        if a.hardware and not a.i_have_checked_safety:
            sys.exit("--hardware also requires --i-have-checked-safety (see README safety checklist)")
        driver = GMH15HttpDriver(a.url, a.side, hardware_enabled=hw)
        q0 = driver.connect()
        pipe.guard.lo, pipe.guard.hi = driver.clip_limits(pipe.guard.lo, pipe.guard.hi)
        pipe.reset(q0)
        print("connected; measured", np.round(q0, 1), "limits", np.round(pipe.guard.hi, 1), flush=True)
    else:
        driver = DryRunDriver()

    flip = not a.no_flip
    tracker = MediaPipeTracker(a.model, num_hands=2, mirrored=flip)
    cap = cv2.VideoCapture(a.video if a.video else a.camera)
    if not cap.isOpened():
        sys.exit("cannot open camera/video (macOS: grant camera permission to the terminal app)")
    rec = open(a.record, "w") if a.record else None
    paused, note, t_prev, fps = False, "", None, None
    t_start = time.monotonic()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if a.video:     # use video timestamps so replay timing matches capture
                t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                now = t
            else:
                t = now = time.monotonic()
            if flip:
                frame = cv2.flip(frame, 1)
            obs = select_hand(tracker.process(frame, t - (0 if a.video else t_start)), t, side_label)
            obs.t = t
            if paused:
                obs.detected = False
            res = pipe.step(obs, now)
            driver.submit(res.command, t)
            st = driver.status()
            if st.get("fault") and pipe.guard.state != "LOST":
                note = f"driver fault: {st['fault']}"
            if rec:
                rec.write(json.dumps({"obs": obs_to_json(obs), "valid": res.valid, "reason": res.reason,
                                      "state": res.state,
                                      "closure": None if res.closure is None else np.round(res.closure, 4).tolist(),
                                      "target": None if res.target is None else np.round(res.target, 2).tolist(),
                                      "command": None if res.command is None else np.round(res.command, 2).tolist(),
                                      "driver": {k: st.get(k) for k in ("armed", "fault", "measured", "tactile")}}) + "\n")
            if t_prev is not None and t > t_prev:
                fps = 0.9 * (fps or 1 / (t - t_prev)) + 0.1 / (t - t_prev)
            t_prev = t
            if a.headless:
                continue
            cv2.imshow("bare-hand -> GMH15", overlay.draw(frame, obs, res, pipe.guard.lo, pipe.guard.hi, st, fps,
                                                          note + (" [PAUSED]" if paused else "")))
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                paused = not paused
            if k in (ord("o"), ord("f"), ord("p")):
                snap = pipe.snapshot_raw()
                if snap is None:
                    note = "no hand for calibration"
                else:
                    {ord("o"): calib.set_open, ord("f"): calib.set_fist, ord("p"): calib.set_pinch}[k](snap)
                    note = f"captured {chr(k)}"
            if k == ord("s"):
                os.makedirs(os.path.dirname(calib_path) or ".", exist_ok=True)
                calib.save(calib_path)
                note = f"saved {calib_path}"
            if k == ord("a"):
                try:
                    if not isinstance(driver, GMH15HttpDriver):
                        raise DriverFault("dry-run driver; nothing to arm")
                    q0 = driver.arm()
                    pipe.reset(q0)       # rate-limited approach from the measured pose
                    note = "ARMED"
                except DriverFault as e:
                    note = str(e)
    finally:
        driver.close()
        cap.release()
        tracker.close()
        if rec:
            rec.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
