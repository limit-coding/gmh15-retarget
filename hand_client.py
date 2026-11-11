"""Thin client for the GMH15 teleop server (127.0.0.1:8765) with a background heartbeat.
Joint order (11): 0 idx MCP, 1 idx PIP*, 2 mid MCP, 3 mid PIP*, 4 ring MCP, 5 ring PIP*,
6 little MCP, 7 little PIP*, 8 thumb CMC, 9 thumb MCP, 10 thumb IP   (*passive, must be 0)."""
import json, secrets, threading, time, urllib.request, urllib.error

B = "http://127.0.0.1:8765"
FINGER_MCP = {"index": 0, "middle": 2, "ring": 4, "little": 6}
FINGER_J15 = {"index": 0, "middle": 3, "ring": 6, "little": 9}  # feedback index in 15-joint list
THUMB = (8, 9, 10)
THUMB_J15 = (12, 13, 14)
TACTILE_NAME = {"index": "食指", "middle": "中指", "ring": "无名指", "little": "小指", "thumb": "拇指"}


class Hand:
    def __init__(self, side="right"):
        self.side = side
        self.owner = "claude-" + secrets.token_hex(10)
        self.token = json.load(urllib.request.urlopen(B + "/api/teleop/config"))["token"]
        self.seq = 0
        self._hb = None
        self._hb_stop = threading.Event()

    def post(self, body):
        body = dict(body, side=self.side, owner=self.owner)
        r = urllib.request.Request(B + "/api/teleop", data=json.dumps(body).encode(), method="POST",
                                   headers={"Content-Type": "application/json", "Origin": "http://127.0.0.1:8765",
                                            "X-Teleop-Token": self.token})
        for _ in range(15):     # sampling pauses ~0.3 s right after init/arm: retry briefly
            try:
                out = json.loads(urllib.request.urlopen(r, timeout=2).read())
            except urllib.error.HTTPError as e:
                out = json.loads(e.read())
            if "新鲜" not in str(out.get("error", "")):
                return out
            time.sleep(0.1)
        return out

    def state(self):
        d = json.load(urllib.request.urlopen(B + "/api/state", timeout=2))
        return d["hands"][self.side], d["teleop"][self.side]

    def _wait_request(self, rid, done, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.1)
            h, t = self.state()
            if t.get("last_request_id") == rid and done(t):
                return h, t
        raise RuntimeError(f"timeout waiting for request; teleop={t}")

    def initialize(self):
        r = self.post(dict(action="initialize", confirm_initialization=True))
        if not r.get("accepted"):
            raise RuntimeError(f"init rejected: {r}")
        h, t = self._wait_request(r["request_id"], lambda t: not t.get("initializing"), 20)
        if t.get("last_init_result") != 1:
            raise RuntimeError(f"init failed: {t.get('last_init_result')} {t.get('error')}")
        return t["initialization_diagnostics"]

    def _heartbeat(self):
        while not self._hb_stop.is_set():
            try:
                self.post(dict(action="heartbeat", control_enabled=True))
            except Exception:
                pass
            self._hb_stop.wait(0.1)

    def arm(self):
        r = self.post(dict(action="arm", control_enabled=True))
        if not r.get("accepted"):
            raise RuntimeError(f"arm rejected: {r}")
        self._hb_stop.clear()
        self._hb = threading.Thread(target=self._heartbeat, daemon=True)
        self._hb.start()
        for _ in range(40):
            time.sleep(0.1)
            h, t = self.state()
            if t.get("active") and t.get("owner") == self.owner:
                return
            if t.get("last_request_id") == r["request_id"] and t.get("error"):
                break
        self.stop()
        raise RuntimeError(f"arm not confirmed: {t.get('error')} {t.get('reason')}")

    def target(self, angles):
        self.seq += 1
        a = [float(x) for x in angles]
        for i in (1, 3, 5, 7):
            a[i] = 0.0
        r = self.post(dict(action="target", control_enabled=True, sequence=self.seq, angles_degrees=a))
        if not r.get("accepted"):
            raise RuntimeError(f"target rejected: {r}")

    def check_active(self):
        h, t = self.state()
        if not t.get("active"):
            raise RuntimeError(f"control dropped: {t.get('error')} / {t.get('reason')}")
        return h, t

    def move(self, angles, timeout=20.0, tol=1.5):
        """Blocking move; returns when commanded slew reaches target (and measured within tol)."""
        self.target(angles)
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.1)
            h, t = self.check_active()
            if max(abs(a - b) for a, b in zip(t["sent_degrees"], angles if len(angles) == 11 else [])) < 0.05:
                if max(abs(a - b) for a, b in zip(measured11(h), angles)) < tol:
                    return h
        return h

    def stop(self):
        self._hb_stop.set()
        r = self.post(dict(action="stop"))
        time.sleep(1.0)
        return r


def measured11(h):
    j = h["joints_degrees"]
    return [j[0], 0, j[3], 0, j[6], 0, j[9], 0, j[12], j[13], j[14]]


def tactile(h):
    """Total normal force per finger (N): sum of the 3-element force vector per finger as reported."""
    out = {}
    for f in h.get("fingers", []):
        for k, name in TACTILE_NAME.items():
            if f["name"] == name:
                out[k] = f["force"]
    return out
