"""GMH15 right hand automated test via local teleop API (127.0.0.1:8765).
Arms, moves each finger moderately, does a soft grip, returns open, then stops (disables).
Always sends 'stop' on exit."""
import json, secrets, time, urllib.request, sys

B = "http://127.0.0.1:8765"
EDITABLE = None
OWNER = "claude-autotest-" + secrets.token_hex(8)
cfg = json.load(urllib.request.urlopen(B + "/api/teleop/config"))
TOKEN = cfg["token"]
NAMES = cfg["joint_names"]
seq = 0


def post(body):
    body = dict(body, side="right", owner=OWNER)
    r = urllib.request.Request(B + "/api/teleop", data=json.dumps(body).encode(), method="POST",
                               headers={"Content-Type": "application/json", "Origin": "http://127.0.0.1:8765",
                                        "X-Teleop-Token": TOKEN})
    try:
        return json.loads(urllib.request.urlopen(r, timeout=2).read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


def state():
    d = json.load(urllib.request.urlopen(B + "/api/state", timeout=2))
    return d["hands"]["right"], d["teleop"]["right"]


def hold(target, seconds, label):
    """Send target + heartbeats at 10 Hz for `seconds`; log tracking."""
    global seq
    seq += 1
    r = post(dict(action="target", control_enabled=True, sequence=seq, angles_degrees=target))
    if not r.get("accepted"):
        raise RuntimeError(f"target rejected: {r}")
    t0 = time.time()
    while time.time() - t0 < seconds:
        post(dict(action="heartbeat", control_enabled=True))
        time.sleep(0.1)
        h, t = state()
        if VERBOSE: print("  tick", [round(x,1) for x in t.get("sent_degrees") or []][:2], [round(h["joints_degrees"][i],1) for i in (0,1,2)], h["motor_positions"], flush=True)
        if not t.get("active"):
            raise RuntimeError(f"control dropped: reason={t.get('reason')} error={t.get('error')}")
    h, t = state()
    sent = t["sent_degrees"]
    err = max(abs(a - b) for a, b in zip(sent, target))
    force = {f["name"]: round(sum(f["force"]), 2) for f in h.get("fingers", [])}
    print(json.dumps(dict(step=label, target=[round(x, 1) for x in target], sent=[round(x, 1) for x in sent],
                          reached=err < 1.0, joints15=[round(x, 1) for x in h["joints_degrees"]],
                          motors=h["motor_positions"], faults=h["device_diagnostics"]["faults"],
                          tactile_N=force), ensure_ascii=False), flush=True)
    return err


VERBOSE = "--verbose" in sys.argv


def main():
    h, t = state()
    print("pre:", json.dumps(dict(enabled=h["motor_enabled"], faults=h["device_diagnostics"]["faults"],
                                   last_init=t.get("last_init_result"), active=t.get("active")), ensure_ascii=False))
    if "--init" in sys.argv:
        r = post(dict(action="initialize", confirm_initialization=True)); print("init:", r, flush=True)
        for _ in range(100):
            time.sleep(0.2); h, t = state()
            if t.get("last_request_id") == r.get("request_id") and not t.get("initializing"): break
        print("init result:", t.get("last_init_result"), t.get("initialization_diagnostics"), flush=True)
        if t.get("last_init_result") != 1: raise RuntimeError("init failed")
    bad = [0.0] * 11; bad[1] = 20.0
    r = post(dict(action="preview", angles_degrees=bad))
    print("PIP preview rejected:", r, flush=True)
    if r.get("accepted"): raise RuntimeError("server accepted nonzero PIP")
    r = post(dict(action="arm", control_enabled=True))
    print("arm:", r)
    for _ in range(30):
        post(dict(action="heartbeat", control_enabled=True))
        time.sleep(0.1)
        h, t = state()
        if t.get("active"):
            break
        if t.get("error") and t.get("last_request_id") == r.get("request_id") and not t.get("active"):
            raise RuntimeError(f"arm failed: {t}")
    else:
        raise RuntimeError(f"arm not confirmed: {t}")
    print("armed; enabled=", h["motor_enabled"], flush=True)

    zero = [0.0] * 11
    if "--joint" in sys.argv:
        j = int(sys.argv[sys.argv.index("--joint") + 1]); tgt = zero[:]; tgt[j] = 15.0
        hold(tgt, 3.5, f"joint {j} {NAMES[j]} 15"); hold(zero, 3.5, "back")
        return
    if "--probe" in sys.argv:
        tgt = zero[:]; tgt[0] = 20.0
        hold(tgt, 4.0, "probe index MCP 20"); hold(zero, 4.0, "probe back")
        return
    A = 30.0  # moderate test angle, well under limits
    # 7 actuated joints: 4 finger MCPs (0,2,4,6) + thumb CMC/MCP/IP (8,9,10).
    # Finger PIPs (1,3,5,7) have no independent motor/feedback -> keep at 0.
    tests = [
        ("index MCP 30", {0: A}),
        ("middle MCP 30", {2: A}),
        ("ring MCP 30", {4: A}),
        ("little MCP 30", {6: A}),
        ("thumb CMC 30", {8: A}),
        ("thumb MCP 30", {9: A}),
        ("thumb IP 30", {10: A}),
    ]
    move_time = A / 8.0 + 1.5  # slew 8 deg/s
    for label, js in tests:
        tgt = zero[:]
        for i, v in js.items():
            tgt[i] = v
        hold(tgt, move_time, label)
        hold(zero, move_time, label + " -> back")
    # soft half-fist: four fingers 45, thumb CMC 30
    fist = [45.0, 0, 45.0, 0, 45.0, 0, 45.0, 0, 30.0, 20.0, 20.0]
    hold(fist, 45 / 8.0 + 1.5, "half fist")
    hold(fist, 1.0, "half fist hold")
    hold(zero, 45 / 8.0 + 1.5, "open")


if __name__ == "__main__":
    try:
        main()
        print("RESULT: PASS", flush=True)
    except Exception as e:
        print("RESULT: FAIL", e, flush=True)
    finally:
        print("stop:", post(dict(action="stop")), flush=True)
        time.sleep(1.5)
        h, t = state()
        print("post:", json.dumps(dict(enabled=h["motor_enabled"], active=t.get("active"), reason=t.get("reason"),
                                        error=t.get("error"), faults=h["device_diagnostics"]["faults"],
                                        joints15=[round(x, 1) for x in h["joints_degrees"]]), ensure_ascii=False))
