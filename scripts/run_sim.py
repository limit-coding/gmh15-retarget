#!/usr/bin/env python3
"""Bare hand -> GMH15 in simulation (MuJoCo, kinematic). No hardware is touched.

  .venv/bin/python scripts/run_sim.py                       # Mac camera 0, right hand
  .venv/bin/python scripts/run_sim.py --camera 1 --no-flip  # external camera looking at the hand
  .venv/bin/python scripts/run_sim.py --jsonl runs/live.jsonl   # replay a recording (run_camera --record)
  .venv/bin/python scripts/run_sim.py --path synergy        # compare finger coupling hypotheses

Real hand (start the local Mac service with 启动灵巧手.command first):
  .venv/bin/python scripts/run_sim.py --hardware --i-have-checked-safety
  Starts DISARMED (read-only). Press a to arm, d to disarm, q quits (disarms).
  The sim shows the full retargeting result; the hardware gets the same motion scaled into the
  verified range (bare_hand_teleop/hw_map.py: finger closure -> MCP 0-45, thumb 0-30 each, PIP 0).
  Tracking loss -> hold; server watchdog 0.6 s -> motors off.
  Status page: http://127.0.0.1:8766/ (camera + sim image, sim / sent / measured per joint).

Window: camera + landmarks | GMH15 sim (palm view) | numbers.
Keys: q quit, r reset, space pause,
      o = hold your hand relaxed OPEN and press (captures ~0.7 s), f = same for a FIST.
      Calibration is saved to config/sim_calib_{side}.json and loaded automatically next time.
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)
from bare_hand_teleop import gmh15_spec as spec                     # noqa: E402
from bare_hand_teleop.hand_types import HandObservation             # noqa: E402
from bare_hand_teleop.opt_retarget import OptConfig, load_calib, save_calib  # noqa: E402
from bare_hand_teleop.overlay import CONNECTIONS, STATE_COLOR        # noqa: E402
from bare_hand_teleop.drivers import DriverFault, GMH15HttpDriver     # noqa: E402
from bare_hand_teleop.safety import GateConfig, GuardConfig          # noqa: E402
from bare_hand_teleop.filters import MotionShaper                    # noqa: E402
from bare_hand_teleop.sim_pipeline import SimPipeline                # noqa: E402
from bare_hand_teleop import hw_map                                  # noqa: E402
from bare_hand_teleop.live_web import LiveWeb                        # noqa: E402


class SimView(threading.Thread):
    """MuJoCo rendering in its own thread (GL context must stay in one thread; ~100+ ms/frame)."""

    def __init__(self, side, R, size=(420, 420)):
        super().__init__(daemon=True)
        self.side, self.R, self.size = side, R, size
        self.q, self.markers, self.image = None, [], None
        self.lock, self.stop_evt = threading.Lock(), threading.Event()

    def submit(self, q, markers):
        with self.lock:
            self.q, self.markers = q, markers

    def run(self):
        from bare_hand_teleop.sim import HandSim
        sim = HandSim(self.side, self.size)
        sim.cam.lookat[:] = self.R.T @ np.array([0.0, 0.09, 0.0])
        sim.look_from(self.R.T @ np.array([0.25, -0.35, 1.0]))
        while not self.stop_evt.is_set():
            with self.lock:
                q, mk = self.q, self.markers
                self.q = None
            if q is None:
                time.sleep(0.01)
                continue
            sim.set_q(q)
            img = sim.render(mk)[:, :, ::-1].copy()
            with self.lock:
                self.image = img
        sim.close()


def jsonl_frames(path):
    with open(path) as fh:
        for line in fh:
            o = json.loads(line)["obs"]
            yield None, HandObservation(t=o["t"], detected=o["detected"], handedness=o["handedness"],
                                        score=o["score"], world=None if o["world"] is None else np.array(o["world"]),
                                        image=None if o["image"] is None else np.array(o["image"]))


def panel(h, step, info, fps, side_label, hw=None):
    p = np.zeros((h, 360, 3), np.uint8)
    col = STATE_COLOR.get(step.state, (200, 200, 200))
    y = 22
    def put(txt, c=(220, 220, 220), s=0.48):
        nonlocal y
        cv2.putText(p, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, s, c, 1)
        y += 18
    put(f"{step.state}  {step.reason}", col, 0.58)
    put(f"fps {fps:.0f}   solve {info.get('solve_ms', 0):.1f} ms   alpha {info.get('alpha', 0):.2f}")
    put(info.get("calib", ""), (200, 200, 90), 0.42)
    if hw is not None:
        armed = hw.get("armed")
        txt = "HW ARMED" if armed else ("HW FAULT: " + str(hw.get("fault"))[:30] if hw.get("fault") else "HW disarmed (a = arm)")
        put(txt, (0, 0, 255) if armed or hw.get("fault") else (200, 200, 200), 0.5)
    ch = info.get("chirality")
    if ch is not None:
        put(f"chirality {'OK' if ch > 0 else 'MIRRORED? try --no-flip / flip'} ({ch * 1000:+.0f})",
            (60, 200, 60) if ch > 0 else (40, 40, 230))
    hp, rp = info.get("human_pinch_m", {}), info.get("robot_pinch_m", {})
    put("thumb-tip distance  human -> robot (mm)", (180, 180, 180))
    for f in ("index", "middle", "ring", "pinky"):
        if f in hp:
            put(f"  {f:6s} {hp[f] * 1000:6.1f} -> {rp[f] * 1000:6.1f}", (255, 0, 255) if f == "index" else (220, 220, 220))
    q = step.command if step.command is not None else step.held
    put("command (deg, SDK order)", (180, 180, 180))
    for i in range(spec.N_CMD):
        put(f"  {spec.JOINT_NAMES[i]:10s} {q[i]:6.1f}", col)
    cv2.putText(p, "q quit r reset space pause o open f fist", (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1)
    return p


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--camera", type=int, default=0)
    g.add_argument("--video")
    g.add_argument("--jsonl")
    ap.add_argument("--camera-fps", type=float, default=0, help="Request camera FPS; 0 keeps the camera default")
    ap.add_argument("--preview-hz", type=float, default=0, help="Limit only preview updates; 0 renders every tracked frame")
    ap.add_argument("--duration", type=float, default=0, help="Optional run duration in seconds")
    ap.add_argument("--tracking-width", type=int, default=0, help="Optional smaller recognition image, preserving the camera preview")
    ap.add_argument("--tracking-hands", type=int, choices=(1, 2), default=2)
    ap.add_argument("--latest-frame", action="store_true", help="Capture continuously and process the newest frame")
    ap.add_argument("--sim-size", type=int, default=420, help="Simulation preview render size")
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--no-flip", action="store_true", help="do not mirror the camera image")
    ap.add_argument("--model", default=os.path.join(ROOT, "models/hand_landmarker.task"))
    ap.add_argument("--path", choices=["measured", "mcp_first", "synergy"], default="measured")
    ap.add_argument("--record", help="write JSONL (obs + sim command) for later replay")
    ap.add_argument("--any-hand", action="store_true", help="accept either handedness label")
    ap.add_argument("--calib", help="closure calibration JSON (default config/sim_calib_<side>.json if present)")
    ap.add_argument("--no-calib", action="store_true", help="ignore any calibration file")
    ap.add_argument("--hardware", action="store_true", help="drive the real GMH15 via the teleop server")
    ap.add_argument("--send-hz", type=float, default=10, help="Hardware target frequency, 5-30 Hz")
    ap.add_argument("--i-have-checked-safety", action="store_true")
    ap.add_argument("--joint-mode", action="store_true",
                    help="old hardware path: joint angles scaled into MCP 0-45 / thumb 0-30, PIP 0")
    ap.add_argument("--web-port", type=int, default=8766, help="status page port (0 disables)")
    ap.add_argument("--url", default="http://localhost:8765", help="teleop server (local Mac service by default)")
    a = ap.parse_args()
    if not 0 <= a.camera_fps <= 60 or not 0 <= a.preview_hz <= 60 or a.duration < 0:
        ap.error('Invalid camera/preview frequency or duration')
    if not 5 <= a.send_hz <= 30:
        ap.error('--send-hz must be between 5 and 30')
    calib_path = a.calib or os.path.join(ROOT, f"config/sim_calib_{a.side}.json")
    opt = OptConfig(path=a.path)
    calib_msg = "calib: none (press o / f)"
    if not a.no_calib and os.path.exists(calib_path):
        load_calib(opt, calib_path)
        calib_msg = f"calib: {os.path.basename(calib_path)}"

    side_label = a.side.capitalize()
    flip = not a.no_flip and not a.jsonl          # flip the live image
    mirror = not a.no_flip                         # recordings store un-mirrored world landmarks
    # Responsiveness (2026-11-07): replaying runs/hw_live8.jsonl, x -> command lag went from
    # 310-550 ms (One-Euro 1.5/0.5, shaper 120 deg/s, 600 deg/s^2, tau .12) to 80-220 ms.
    pipe = SimPipeline(a.side, opt=opt,
                       gate=GateConfig(expected_handedness=None if a.any_hand else side_label),
                       mirror_world=mirror, one_euro=(2.0, 1.0),
                       shaper=MotionShaper(v_max=360.0, a_max=3000.0, tau=0.05),
                       guard=GuardConfig(max_rate_deg_s=720.0))
    driver = None
    if a.hardware:
        if not a.i_have_checked_safety:
            sys.exit("--hardware also requires --i-have-checked-safety")
        if a.jsonl:
            sys.exit("--hardware is only for live camera input")
        driver = GMH15HttpDriver(a.url, a.side, hardware_enabled=True, send_hz=a.send_hz,
                                 motor_mode=not a.joint_mode)
        q0 = driver.connect()                     # read-only
        # The sim keeps the full URDF range; hw_map scales each frame into driver.lo/hi.
        print("connected (DISARMED); measured", np.round(q0, 1), flush=True)
        print("hardware path:", "joint angles (MCP 0-45, thumb 0-30)" if a.joint_mode else
              f"motor counts, full measured range; ceilings {driver.motor_hi.astype(int).tolist()}", flush=True)
    web = LiveWeb(a.web_port) if a.web_port else None
    if web:
        print(f"status page: http://127.0.0.1:{a.web_port}/", flush=True)
    view = SimView(a.side, pipe.rt.R, size=(a.sim_size, a.sim_size))
    view.start()

    tracker = None
    raw_labels = []
    frame_timing = {}
    camera_reader = None
    if a.jsonl:
        frames = jsonl_frames(a.jsonl)
    else:
        from bare_hand_teleop.tracking import MediaPipeTracker, select_hand
        tracker = MediaPipeTracker(a.model, num_hands=a.tracking_hands, mirrored=flip)
        cap = cv2.VideoCapture(a.video if a.video else a.camera)
        if not cap.isOpened():
            sys.exit("cannot open camera/video (macOS: allow camera access for the terminal app)")
        if not a.video and a.camera_fps:
            accepted = cap.set(cv2.CAP_PROP_FPS, a.camera_fps)
            print(f'camera: requested {a.camera_fps:g} fps, accepted={accepted}, '
                  f'reported={cap.get(cv2.CAP_PROP_FPS):g} fps; actual FPS is measured from frame timestamps', flush=True)
        capture_started = time.monotonic()
        if a.latest_frame and not a.video:
            from bare_hand_teleop.capture import LatestCamera
            camera_reader = LatestCamera(cap)

        def cam_frames():
            t0 = capture_started
            while True:
                read_start = time.monotonic()
                if camera_reader:
                    ok, frame, captured_at = camera_reader.read()
                else:
                    ok, frame = cap.read()
                    captured_at = time.monotonic()
                read_end = time.monotonic()
                if not ok:
                    return
                t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0 if a.video else captured_at - t0
                if flip:
                    frame = cv2.flip(frame, 1)
                recognition_frame = frame
                if a.tracking_width and frame.shape[1] > a.tracking_width:
                    recognition_frame = cv2.resize(frame, (a.tracking_width,
                        round(frame.shape[0] * a.tracking_width / frame.shape[1])))
                track_start = time.monotonic()
                raw = tracker.process(recognition_frame, t)
                frame_timing.update(capture_ms=(read_end-read_start)*1000,
                                    tracking_ms=(time.monotonic()-track_start)*1000)
                raw_labels[:] = [(o.handedness, round(o.score, 2)) for o in raw]
                obs = select_hand(raw, t, None if a.any_hand else side_label)
                obs.t = t
                yield frame, obs
        frames = cam_frames()

    rec = open(a.record, "w") if a.record else None
    paused, fps, t_prev, info = False, 0.0, None, {}
    s_hist, capture = [], None          # capture = ("o"|"f", frames left)
    next_preview = 0.
    run_started = time.monotonic()
    for frame, obs in frames:
        if a.duration and time.monotonic() - run_started >= a.duration:
            break
        if paused:
            obs = HandObservation(t=obs.t, detected=False)
        t0 = time.perf_counter()
        step = pipe.step(obs, obs.t)
        frame_timing['retarget_ms'] = (time.perf_counter()-t0)*1000
        if step.info:
            info = dict(step.info, solve_ms=(time.perf_counter() - t0) * 1000, calib=calib_msg)
        if capture and step.info and "s_raw" in step.info:
            s_hist.append(step.info["s_raw"])
            if len(s_hist) >= 10:
                v = np.median(s_hist, axis=0)
                if capture == "o":
                    opt.open_s = tuple(v)
                else:
                    opt.fist_s = tuple(v)
                if np.all(np.asarray(opt.fist_s) - np.asarray(opt.open_s) > 0.3):
                    save_calib(opt, calib_path)
                    calib_msg = f"calib saved: open {np.round(opt.open_s, 2)} fist {np.round(opt.fist_s, 2)}"
                else:
                    calib_msg = "calib rejected: open/fist too close, retry"
                    opt.open_s, opt.fist_s = (0.0,) * 4, (1.0,) * 4
                print(calib_msg)
                if web:
                    web.update(msg=calib_msg)
                capture, s_hist = None, []
        q = step.command if step.command is not None else step.held
        hw_cmd = None
        if driver is not None:
            if step.command is not None:
                hw_cmd = (hw_map.hw_from_sim(step.command, driver.lo, driver.hi, a.path) if a.joint_mode
                          else np.array(hw_map.counts_from_sim(step.command), float))
            driver.submit(hw_cmd if driver.armed else None, obs.t)
        tips = pipe.tips_from_cmd(q)
        view.submit(pipe.urdf_q_from_cmd(q), [(tips["thumb"], [1, 0, 1, 1]), (tips["index"], [0, 1, 1, 1])])
        if rec:
            rec.write(json.dumps({"timing": dict(frame_timing), "obs": {"t": obs.t, "detected": obs.detected, "handedness": obs.handedness,
                                          "score": obs.score,
                                          "world": None if obs.world is None else np.round(obs.world, 5).tolist(),
                                          "image": None if obs.image is None else np.round(obs.image, 5).tolist()},
                                  "raw_hands": raw_labels, "state": step.state, "reason": step.reason,
                                  "x": None if step.x is None else np.round(step.x, 4).tolist(),
                                  "command": None if step.command is None else np.round(step.command, 2).tolist(),
                                  "hw_command": None if hw_cmd is None else np.round(hw_cmd, 2).tolist(),
                                  "human_pinch_m": info.get("human_pinch_m"), "robot_pinch_m": info.get("robot_pinch_m")}) + "\n")
        if t_prev is not None and obs.t > t_prev:
            fps = 0.9 * fps + 0.1 / (obs.t - t_prev) if fps else 1 / (obs.t - t_prev)
        t_prev = obs.t

        # Preview refreshes do not determine camera/recording cadence.
        if not a.preview_hz or time.monotonic() >= next_preview:
            next_preview = time.monotonic() + (1 / a.preview_hz if a.preview_hz else 0)
            H = 420
            if frame is None:
                cam = np.zeros((H, 420, 3), np.uint8)
            else:
                cam = cv2.resize(frame, (int(frame.shape[1] * H / frame.shape[0]), H))
            if obs.detected and obs.image is not None:
                hh, ww = cam.shape[:2]
                pts = [(int(x * ww), int(y * hh)) for x, y, _ in obs.image]
                c = STATE_COLOR.get(step.state, (200, 200, 200))
                for i0, i1 in CONNECTIONS:
                    cv2.line(cam, pts[i0], pts[i1], c, 2)
                cv2.line(cam, pts[4], pts[8], (255, 0, 255), 2)
            with view.lock:
                simimg = view.image if view.image is not None else np.zeros((H, H, 3), np.uint8)
            composite = np.hstack([cam, cv2.resize(simimg, (H, H)), panel(H, step, info, fps, side_label, driver.status() if driver else None)])
            cv2.imshow("bare hand -> GMH15 sim", composite)
            if web:
                st = driver.status() if driver else None
                names = ("食指", "中指", "无名指", "小指", "拇指 CMC", "拇指 MCP", "拇指 IP")
                if driver is not None and not a.joint_mode:     # motor counts end to end
                    simc = hw_map.counts_from_sim(q)
                    sent = (st or {}).get("motor_sent") if st and st.get("armed") else None
                    meas = (st or {}).get("motor_positions")
                    rows = [dict(name=n, sim=simc[i], sent=None if not sent else sent[i],
                                 meas=None if not meas else meas[i], hw_max=float(driver.motor_hi[i]),
                                 sim_max=10000.0) for i, n in enumerate(names)]
                    unit = "计数"
                else:
                    sim7 = hw_map.seven(q)
                    sim_max = hw_map.seven(pipe.guard.hi)
                    for k in range(4):             # fingers: closure as an MCP-equivalent 0-80
                        sim7[k] = hw_map.finger_closure(q, a.path)[k] * sim_max[k]
                    sent7 = hw_map.seven(st["server_sent"]) if st and len(st.get("server_sent") or []) == 11 else [None] * 7
                    meas7 = hw_map.seven(st["measured"]) if st and st.get("measured") else [None] * 7
                    hw7 = hw_map.seven(driver.hi) if driver else [None] * 7
                    rows = [dict(name=n, sim=sim7[i], sent=sent7[i], meas=meas7[i], hw_max=hw7[i], sim_max=sim_max[i])
                            for i, n in enumerate(names)]
                    unit = "°"
                ok, jpg = cv2.imencode(".jpg", composite, [cv2.IMWRITE_JPEG_QUALITY, 70])
                web.update(jpeg=jpg.tobytes() if ok else None, rows=rows, unit=unit, state=step.state, fps=float(fps),
                           hw=None if not st else dict(armed=st.get("armed"), fault=st.get("fault"),
                                                       holding=st.get("holding"), releasing=st.get("releasing"),
                                                       hold_remaining_s=st.get("hold_remaining_s")))
        keys = [cv2.waitKey(1 if not a.jsonl else 30) & 0xFF]
        if web:                                   # web buttons act exactly like the window keys
            keys += [ord({"arm": "a", "disarm": "d", "open": "o", "fist": "f", "hold": "h", "release": "u"}[x])
                     for x in web.pop_actions()]
        if ord("q") in keys:
            break
        for k in keys:
            if driver is not None and k == ord("a") and not driver.armed:
                try:
                    q0 = driver.arm()             # server slews from the measured pose (8 deg/s)
                    print("ARMED from", np.round(q0, 1), flush=True)
                    if web:
                        web.update(msg="已使能，真机开始跟随")
                except DriverFault as e:
                    print("arm failed:", e, flush=True)
                    if web:
                        web.update(msg=f"使能失败: {e}")
            if driver is not None and k == ord("h"):
                try:
                    driver.hold()
                    print("HOLD: server keeps the grasp; you can stop teleop or quit", flush=True)
                    if web:
                        web.update(msg="保持握持中：可以放下手 / 关掉程序，点「松开」张开")
                except DriverFault as e:
                    print("hold failed:", e, flush=True)
                    if web:
                        web.update(msg=f"保持失败: {e}")
            if driver is not None and k == ord("u"):
                try:
                    driver.release()
                    print("RELEASE requested", flush=True)
                    if web:
                        web.update(msg="正在松开…（松开后自动失能）")
                except DriverFault as e:
                    print("release failed:", e, flush=True)
                    if web:
                        web.update(msg=f"松开失败: {e}")
            if driver is not None and k == ord("d") and not driver.armed and driver.holding:
                driver._post(dict(action="stop"))       # emergency stop also ends a hold
                driver.holding = False
                print("STOP during hold (motors disabled)", flush=True)
                if web:
                    web.update(msg="已急停（失能），保持结束")
            if driver is not None and k == ord("d") and driver.armed:
                driver.close()
                print("disarmed", flush=True)
                if web:
                    web.update(msg="已停止（失能）")
            if k == ord("r"):
                pipe.reset()
            if k == ord(" "):
                paused = not paused
            if k in (ord("o"), ord("f")):
                capture, s_hist = chr(k), []
                calib_msg = f"capturing {'OPEN' if k == ord('o') else 'FIST'} - hold still"
                info["calib"] = calib_msg
                if web:
                    web.update(msg=("请保持张开不动…" if k == ord("o") else "请保持握拳不动…"))
    view.stop_evt.set()
    if web:
        web.close()
    if driver is not None:
        driver.close()
    if rec:
        rec.close()
    if tracker:
        tracker.close()
    if camera_reader:
        camera_reader.close()
    elif not a.jsonl:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
