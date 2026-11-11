"""Search the vendor URDF for thumb-index pinch poses (model-only; verify on hardware)."""
import sys, os
import numpy as np
from scipy.optimize import minimize
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bare_hand_teleop.gmh15_model import GMH15Model

urdf = sys.argv[1] if len(sys.argv) > 1 else "vendor/yubi-pi_gmh15-sdk/models/right/meshes/DexH15_right_urdf.urdf"
m = GMH15Model(urdf)
rng = np.random.default_rng(0)
z = np.zeros(11)
q0 = m.q_from_cmd(z)
print("open thumb-index tip distance %.1f mm" % (1000 * np.linalg.norm(m.tip("thumb", q0) - m.tip("index", q0))))

def cmd(x, pip_free):
    c = np.zeros(11); c[0] = x[0]; c[8], c[9], c[10] = x[1], x[2], x[3]
    if pip_free: c[1] = x[4]
    return c

def dist(x, pip_free):
    q = m.q_from_cmd(cmd(x, pip_free))
    return np.linalg.norm(m.tip("thumb", q) - m.tip("index", q))

def search(bnds, pip_free, n=150):
    best = None
    for _ in range(n):
        x0 = np.array([rng.uniform(a, b) for a, b in bnds])
        r = minimize(dist, x0, args=(pip_free,), bounds=bnds, method="L-BFGS-B")
        if best is None or r.fun < best.fun: best = r
    return best

names = ["idx_mcp", "th_cmc", "th_mcp", "th_ip", "idx_pip"]
for label, bnds, pf in [("full URDF range, PIP=0", [(0, 80), (0, 109.6), (0, 87), (0, 120)], False),
                        ("full URDF range, PIP free", [(0, 80), (0, 109.6), (0, 87), (0, 120), (0, 80)], True),
                        ("verified range (fingers<=45, thumb<=30), PIP=0", [(0, 45), (0, 30), (0, 30), (0, 30)], False)]:
    b = search(bnds, pf)
    print(f"{label}: min pad-centroid distance {b.fun*1000:.1f} mm at", dict(zip(names, np.round(b.x, 1))))
