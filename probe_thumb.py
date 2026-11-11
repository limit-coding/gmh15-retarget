import sys, time, urllib.request
sys.path.insert(0, "/tmp")
from hand_client import Hand, measured11
h = Hand()
print("init", h.initialize(), flush=True)
def snap(tag):
    for view in ("oblique", "front", "side"):
        urllib.request.urlopen(f"http://127.0.0.1:8765/media/sim/right.jpg?view={view}").read()
        time.sleep(1.2)  # renderer ~3fps, view switch applies next frame
        open(f"/tmp/thumb_{tag}_{view}.jpg", "wb").write(urllib.request.urlopen(f"http://127.0.0.1:8765/media/sim/right.jpg?view={view}").read())
    open(f"/tmp/thumb_{tag}_cam.jpg", "wb").write(urllib.request.urlopen("http://127.0.0.1:8765/media/camera/right.jpg").read())
try:
    h.arm(); print("armed", flush=True)
    z = [0.0] * 11
    for cmc in (0, 60, 100):
        p = z[:]; p[8] = cmc
        st = h.move(p, timeout=20)
        print("cmc", cmc, "meas", [round(x, 1) for x in measured11(st)], flush=True)
        snap(f"cmc{cmc}")
    h.move(z, timeout=20)
finally:
    print("stop", h.stop(), flush=True)
    s, t = h.state(); print("enabled", s["motor_enabled"], "faults", s["device_diagnostics"]["faults"])
