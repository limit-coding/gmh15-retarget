"""GMH15HttpDriver against a fake teleop server (same endpoints as ../hand_client.py). No hardware."""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bare_hand_teleop.drivers import DriverFault, GMH15HttpDriver  # noqa: E402


class Fake:
    def __init__(self):
        self.posts = []
        self.active = False
        self.owner = None
        self.faults = []
        self.init_result = 1
        self.limits = [[0, 90]] * 11
        self.sent = [0.0] * 11
        self.arm_delay = 0.0     # real server activates asynchronously after "arm"


def make_server(fake):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path == "/api/teleop/config":
                return self._json({"token": "tok", "limits_degrees": fake.limits})
            if self.path == "/api/state":
                j15 = [0.0] * 15
                return self._json({"hands": {"right": {"joints_degrees": j15, "motor_positions": [0] * 7,
                                                       "motor_enabled": fake.active,
                                                       "device_diagnostics": {"faults": fake.faults},
                                                       "fingers": []}},
                                   "teleop": {"right": {"active": fake.active, "owner": fake.owner,
                                                        "last_init_result": fake.init_result,
                                                        "holding": getattr(fake, "holding", False),
                                                        "sent_degrees": fake.sent}}})
            self._json({}, 404)

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            assert self.headers["X-Teleop-Token"] == "tok"
            fake.posts.append(body)
            act = body["action"]
            if act == "arm":
                def activate(owner=body["owner"]):
                    if not any(p["action"] == "stop" for p in fake.posts):
                        fake.active, fake.owner = True, owner
                if fake.arm_delay:
                    threading.Timer(fake.arm_delay, activate).start()
                else:
                    activate()
            elif act == "stop":
                fake.active = False
            elif act == "target":
                if any(body["angles_degrees"][i] != 0 for i in (1, 3, 5, 7)):
                    return self._json({"error": "PIP must be 0"}, 400)
                fake.sent = body["angles_degrees"]
            elif act == "motor_target":
                fake.motor = body["motor_counts"]
            elif act == "hold":
                fake.holding = True
            elif act == "release":
                fake.holding = False
                fake.active = False
            self._json({"accepted": True, "request_id": "x"})

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


@pytest.fixture
def env():
    fake = Fake()
    srv = make_server(fake)
    yield fake, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def actions(fake):
    return [p["action"] for p in fake.posts]


def test_connect_is_read_only_and_arm_refused_without_flag(env):
    fake, url = env
    d = GMH15HttpDriver(url, hardware_enabled=False)
    q = d.connect()
    assert q.shape == (11,)
    with pytest.raises(DriverFault):
        d.arm()
    d.submit(np.full(11, 10.0))
    d.close()
    assert fake.posts == []          # nothing was ever POSTed


def test_arm_refused_if_not_initialised_or_faulted(env):
    fake, url = env
    d = GMH15HttpDriver(url, hardware_enabled=True)
    d.connect()
    fake.init_result = None
    with pytest.raises(DriverFault):
        d.arm()
    fake.init_result, fake.faults = 1, ["overcurrent"]
    with pytest.raises(DriverFault):
        d.arm()
    assert "arm" not in actions(fake)


def test_targets_clipped_passive_zero_and_stop_on_close(env):
    fake, url = env
    d = GMH15HttpDriver(url, hardware_enabled=True)
    d.connect()
    d.arm()
    cmd = np.full(11, 200.0)
    for _ in range(10):
        d.submit(cmd)
        time.sleep(0.03)
    d.close()
    tg = [p for p in fake.posts if p["action"] == "target"]
    assert tg, actions(fake)
    a = np.array(tg[-1]["angles_degrees"])
    assert np.all(a[[1, 3, 5, 7]] == 0)
    assert a[0] == 45.0 and a[8] == 30.0      # verified-range clip, tighter than server's 90
    assert actions(fake)[-1] == "stop"


def test_main_loop_stall_trips_stop(env):
    fake, url = env
    d = GMH15HttpDriver(url, hardware_enabled=True, loop_timeout_s=0.2)
    d.connect()
    d.arm()
    d.submit(np.zeros(11))
    time.sleep(0.6)                           # no submit() calls -> app considered stalled
    assert d.fault == "app_loop_stalled" and not d.armed
    assert "stop" in actions(fake) and not fake.active
    d.close()


def test_control_dropped_by_server_trips(env):
    fake, url = env
    d = GMH15HttpDriver(url, hardware_enabled=True)
    d.connect()
    d.arm()
    fake.active = False                       # e.g. server watchdog / another client
    for _ in range(20):
        d.submit(np.zeros(11))
        time.sleep(0.05)
    assert d.fault and d.fault.startswith("control_dropped")
    d.close()


def test_comm_timeout_trips():
    d = GMH15HttpDriver("http://127.0.0.1:9", hardware_enabled=True, comm_timeout_s=0.2)
    d.token = "tok"
    d._loop_tick = d._last_ok = time.monotonic()
    t = threading.Thread(target=d._run, daemon=True)
    t.start()
    for _ in range(20):
        d.submit(np.zeros(11))
        time.sleep(0.05)
    t.join(timeout=2)
    assert d.fault and d.fault.startswith("comm_timeout")


def test_arm_waits_for_asynchronous_server_activation(env):
    fake, url = env
    fake.arm_delay = 0.4
    d = GMH15HttpDriver(url, "right", hardware_enabled=True, send_hz=50)
    d.connect()
    stop_flag = threading.Event()

    def app_loop():                      # the real main loop keeps calling submit()
        while not stop_flag.is_set():
            d.submit(None)
            time.sleep(0.02)
    threading.Thread(target=app_loop, daemon=True).start()
    try:
        d.arm()
        assert d.armed and fake.active and "stop" not in actions(fake)
    finally:
        stop_flag.set()
        d.close()


def test_motor_mode_sends_clipped_counts(env):
    fake, url = env
    d = GMH15HttpDriver(url, "right", hardware_enabled=True, send_hz=50, motor_mode=True)
    d.connect()
    stop_flag = threading.Event()

    def app_loop():
        while not stop_flag.is_set():
            d.submit([12000, 5000, 0, 0, 0, 9000, 9000] if d.armed else None)
            time.sleep(0.02)
    threading.Thread(target=app_loop, daemon=True).start()
    try:
        d.arm()
        time.sleep(0.3)
        assert fake.motor == [10000, 5000, 0, 0, 0, 7200, 7700]
        assert "target" not in actions(fake)
    finally:
        stop_flag.set()
        d.close()


def test_rearm_after_server_stop_does_not_replay_old_target(env):
    """Regression: a pending target from the previous session was sent before the server
    activated the new one -> rejected -> driver tripped -> could never re-arm."""
    fake, url = env
    fake.arm_delay = 0.3
    d = GMH15HttpDriver(url, "right", hardware_enabled=True, send_hz=50, motor_mode=True)
    d.connect()
    stop_flag = threading.Event()

    def app_loop():
        while not stop_flag.is_set():
            d.submit([3000] * 7 if d.armed else None)
            time.sleep(0.02)
    threading.Thread(target=app_loop, daemon=True).start()
    try:
        d.arm()
        time.sleep(0.2)
        fake.active = False                      # someone else pressed stop
        time.sleep(0.4)
        assert not d.armed and d.fault
        fake.posts.clear()
        d.arm()                                  # must succeed again
        assert d.armed and fake.active and "stop" not in actions(fake)
        assert actions(fake).index("arm") < (actions(fake).index("motor_target")
                                             if "motor_target" in actions(fake) else 10**6)
    finally:
        stop_flag.set()
        d.close()


def test_hold_detaches_without_stop_and_release_works(env):
    fake, url = env
    d = GMH15HttpDriver(url, "right", hardware_enabled=True, send_hz=50, motor_mode=True)
    d.connect()
    stop_flag = threading.Event()

    def app_loop():
        while not stop_flag.is_set():
            d.submit([3000] * 7 if d.armed else None)
            time.sleep(0.02)
    threading.Thread(target=app_loop, daemon=True).start()
    try:
        d.arm()
        time.sleep(0.2)
        d.hold()
        assert d.holding and not d.armed and fake.active and fake.holding
        time.sleep(0.5)                       # app keeps running: no heartbeats/targets needed
        d.close()                             # quitting the gesture program
        assert "stop" not in actions(fake) and fake.active and fake.holding
        with pytest.raises(DriverFault):
            d.arm()                           # cannot take over a held object
        d.release()
        assert not fake.holding
    finally:
        stop_flag.set()
