"""Output drivers. All share: connect() -> measured q or None, submit(cmd|None), status(), close().

DryRunDriver      : no hardware; keeps the last command, optional JSONL log.
GMH15HttpDriver   : talks to the existing GMH15 teleop server (the one ../hand_client.py uses,
                    127.0.0.1:8765 on the Pi; reach it from another machine with an SSH tunnel).
                    Read-only unless hardware_enabled=True AND arm() is called explicitly.
"""
import json
import secrets
import threading
import time
import urllib.error
import urllib.request

import numpy as np

from . import gmh15_spec as spec


class DryRunDriver:
    name = "dry-run"

    def __init__(self, log_path=None):
        self.last = None
        self.count = 0
        self._fh = open(log_path, "w") if log_path else None

    def connect(self):
        return None

    def submit(self, cmd, t=None):
        if cmd is None:
            return
        self.last = np.asarray(cmd, float).copy()
        self.count += 1
        if self._fh:
            self._fh.write(json.dumps({"t": t, "cmd": [round(x, 2) for x in self.last]}) + "\n")

    def status(self):
        return {"driver": self.name, "sent": self.count, "armed": False}

    def close(self):
        if self._fh:
            self._fh.close()


class DriverFault(RuntimeError):
    pass


class GMH15HttpDriver:
    name = "gmh15-http"

    def __init__(self, base="http://127.0.0.1:8765", side="right", hardware_enabled=False,
                 origin=None, send_hz=20.0, comm_timeout_s=0.5, loop_timeout_s=0.3,
                 state_poll_s=0.2, deadband_deg=0.3, motor_mode=False):
        """motor_mode=True: submit() takes 7 raw motor counts (0-10000, see hw_map.counts_from_sim)
        and the server's raw motor path is used (full measured range, no PIP=0 / 45 deg limits)."""
        self.base, self.side = base.rstrip("/"), side
        self.motor_mode = motor_mode
        self.motor_hi = np.array([10000, 10000, 10000, 10000, 10000, 7200, 7700], float)
        self.origin = origin or self.base   # server requires Origin == the address it is reached at
        self.hardware_enabled = hardware_enabled
        self.send_period = 1.0 / send_hz
        self.comm_timeout_s, self.loop_timeout_s = comm_timeout_s, loop_timeout_s
        self.state_poll_s, self.deadband = state_poll_s, deadband_deg
        self.owner = "bare-hand-" + secrets.token_hex(8)
        self.token = None
        self.server_limits = None
        self.lo, self.hi = spec.range_arrays()
        self.seq = 0
        self.armed = False
        self.fault = None
        self.feedback = {}
        self._pending = None
        self._last_sent = None
        self._loop_tick = time.monotonic()
        self._last_ok = time.monotonic()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._worker = None
        self.holding = False
        self._status_poll = 0.0

    # ---------------- HTTP ----------------
    def _get(self, path):
        return json.load(urllib.request.urlopen(self.base + path, timeout=1.0))

    def _post(self, body):
        body = dict(body, side=self.side, owner=self.owner)
        r = urllib.request.Request(self.base + "/api/teleop", data=json.dumps(body).encode(), method="POST",
                                   headers={"Content-Type": "application/json", "Origin": self.origin,
                                            "X-Teleop-Token": self.token or ""})
        for _ in range(10):     # sampling pauses briefly right after init/arm: retry
            try:
                out = json.loads(urllib.request.urlopen(r, timeout=1.0).read())
            except urllib.error.HTTPError as e:
                out = json.loads(e.read())
            if "新鲜" not in str(out.get("error", "")):
                return out
            time.sleep(0.1)
        return out

    def _state(self):
        d = self._get("/api/state")
        return d["hands"][self.side], d["teleop"][self.side]

    # ---------------- lifecycle ----------------
    def connect(self):
        """Read-only: fetch config/limits and current measured position. Never moves the hand."""
        cfg = self._get("/api/teleop/config")
        self.token = cfg.get("token")
        server_hz = cfg.get("max_send_hz")
        if isinstance(server_hz, (int, float)) and np.isfinite(server_hz) and server_hz > 0:
            self.send_period = max(self.send_period, 1.0 / server_hz)
        if cfg.get("motor_hi") and len(cfg["motor_hi"]) == 7:
            self.motor_hi = np.minimum(self.motor_hi, np.array(cfg["motor_hi"], float))
        lim = cfg.get("limits_degrees")
        if lim and len(lim) == spec.N_CMD:
            self.server_limits = [tuple(map(float, x)) for x in lim]
            slo = np.array([a for a, _ in self.server_limits])
            shi = np.array([b for _, b in self.server_limits])
            self.lo, self.hi = np.maximum(self.lo, slo), np.minimum(self.hi, shi)
        h, t = self._state()
        self.feedback = self._summarize(h, t)
        return spec.measured_cmd_from_feedback(h["joints_degrees"])

    def clip_limits(self, lo, hi):
        """Intersect client limits with those fetched from the server."""
        return np.maximum(lo, self.lo), np.minimum(hi, self.hi)

    def arm(self):
        """Explicit, operator-triggered. Requires hardware_enabled, successful init, no faults."""
        if not self.hardware_enabled:
            raise DriverFault("hardware disabled (start with --hardware --i-have-checked-safety)")
        h, t = self._state()
        faults = h.get("device_diagnostics", {}).get("faults")
        if faults:
            raise DriverFault(f"device faults present: {faults}")
        if t.get("last_init_result") != 1:
            raise DriverFault("hand not initialised on server (last_init_result != 1); init via the teleop UI first")
        if t.get("holding") or t.get("releasing"):
            raise DriverFault("hand is holding/releasing an object; release it first")
        if t.get("active") and t.get("owner") != self.owner:
            raise DriverFault(f"side already controlled by {t.get('owner')}")
        with self._lock:            # never replay a target left over from a previous session
            self._pending = None
            self._last_sent = None
        self.fault = None
        r = self._post(dict(action="arm", control_enabled=True))
        if not r.get("accepted"):
            raise DriverFault(f"arm rejected: {r}")
        self._loop_tick = self._last_ok = time.monotonic()
        self._stop.clear()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        for _ in range(30):
            time.sleep(0.1)
            with self._lock:
                self._loop_tick = time.monotonic()   # the app loop is blocked in arm(); not a stall
            h, t = self._state()
            if t.get("active") and t.get("owner") == self.owner:
                self.armed = True
                self.fault = None
                return spec.measured_cmd_from_feedback(h["joints_degrees"])
        self.close()
        raise DriverFault(f"arm not confirmed: {t.get('error')} {t.get('reason')}")

    def hold(self):
        """Hand the frozen grasp to the server (it keeps holding without us), then detach
        WITHOUT sending stop, so quitting this program does not drop the object."""
        if not self.armed:
            raise DriverFault("not armed")
        r = self._post(dict(action="hold", control_enabled=True))
        if not r.get("accepted"):
            raise DriverFault(f"hold rejected: {r}")
        for _ in range(20):
            time.sleep(0.1)
            with self._lock:
                self._loop_tick = time.monotonic()
            h, t = self._state()
            if t.get("holding"):
                break
        else:
            raise DriverFault("hold not confirmed by server")
        self._stop.set()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        self._worker = None          # close() must not send stop now
        self.armed = False
        self.holding = True
        self.feedback = self._summarize(h, t)

    def release(self):
        """Ask the server to open the hand slowly and then disable (works from any client)."""
        r = self._post(dict(action="release", control_enabled=True))
        if not r.get("accepted"):
            raise DriverFault(f"release rejected: {r}")
        self.holding = False

    def submit(self, cmd, t=None):
        """Called every frame from the main loop. cmd=None means hold (no new target)."""
        with self._lock:
            self._loop_tick = time.monotonic()
            if cmd is not None:
                if self.motor_mode:
                    self._pending = np.round(np.clip(np.asarray(cmd, float), 0, self.motor_hi))
                else:
                    c = np.clip(np.asarray(cmd, float), self.lo, self.hi)
                    c[list(spec.PASSIVE)] = 0.0
                    self._pending = c

    def _trip(self, why):
        self.fault = why
        self.armed = False
        self._stop.set()
        try:
            self._post(dict(action="stop"))
        except Exception:
            pass

    def _run(self):
        last_poll = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                stalled = now - self._loop_tick > self.loop_timeout_s
                pending = None if self._pending is None else self._pending.copy()
            if stalled:
                self._trip("app_loop_stalled")
                break
            try:
                self._post(dict(action="heartbeat", control_enabled=True))
                deadband = 15 if self.motor_mode else self.deadband      # counts vs degrees
                # Targets only after the server confirmed this session (arm() sets armed).
                if self.armed and pending is not None and (self._last_sent is None or
                                            np.max(np.abs(pending - self._last_sent)) > deadband):
                    self.seq += 1
                    if self.motor_mode:
                        r = self._post(dict(action="motor_target", control_enabled=True, sequence=self.seq,
                                            motor_counts=[int(x) for x in pending]))
                    else:
                        r = self._post(dict(action="target", control_enabled=True, sequence=self.seq,
                                            angles_degrees=[float(x) for x in pending]))
                    if not r.get("accepted"):
                        self._trip(f"target_rejected: {r}")
                        break
                    self._last_sent = pending
                if now - last_poll > self.state_poll_s:
                    last_poll = now
                    h, t = self._state()
                    self.feedback = self._summarize(h, t)
                    # The server activates asynchronously after "arm"; arm() waits for that.
                    # Only a session that was confirmed active can be "dropped".
                    if self.armed and (not t.get("active") or t.get("owner") != self.owner):
                        self._trip(f"control_dropped: {t.get('error')} / {t.get('reason')}")
                        break
                    if self.feedback["faults"]:
                        self._trip(f"device_fault: {self.feedback['faults']}")
                        break
                self._last_ok = time.monotonic()
            except Exception as e:  # network / JSON errors
                if time.monotonic() - self._last_ok > self.comm_timeout_s:
                    self._trip(f"comm_timeout: {type(e).__name__}")
                    break
            # Count HTTP time inside the period instead of adding it to every tick.
            self._stop.wait(max(0, self.send_period - (time.monotonic() - now)))

    @staticmethod
    def _summarize(h, t):
        return {
            "measured": [round(x, 1) for x in spec.measured_cmd_from_feedback(h["joints_degrees"])],
            "server_sent": [round(x, 1) for x in (t.get("sent_degrees") or [])],
            "motor_positions": h.get("motor_positions"),
            "motor_sent": t.get("motor_sent"),
            "motor_current": h.get("motor_current"),
            "motor_enabled": h.get("motor_enabled"),
            "faults": h.get("device_diagnostics", {}).get("faults") or [],
            "tactile": {f["name"]: round(sum(f["force"]), 2) for f in h.get("fingers", [])},
            "active": t.get("active"), "reason": t.get("reason"),
            "holding": t.get("holding"), "releasing": t.get("releasing"),
            "hold_remaining_s": t.get("hold_remaining_s"),
        }

    def status(self):
        # While not armed (e.g. the server is holding), refresh the feedback ourselves.
        if not self.armed and self.token and time.monotonic() - self._status_poll > 0.3:
            self._status_poll = time.monotonic()
            try:
                h, t = self._state()
                self.feedback = self._summarize(h, t)
                self.holding = bool(t.get("holding"))
            except Exception:
                pass
        return {"driver": self.name, "armed": self.armed, "fault": self.fault,
                "send_hz": round(1.0 / self.send_period, 1),
                "hardware_enabled": self.hardware_enabled, **self.feedback}

    def close(self):
        self._stop.set()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        if self.token and self._worker is not None:   # only if we armed; never stop someone else's session
            try:
                self._post(dict(action="stop"))
            except Exception:
                pass
        self.armed = False
