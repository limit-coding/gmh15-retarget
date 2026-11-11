"""Offline (no device) probe of GMH15 SDK kinematics: how 11 joint angles map to 7 motors."""
import json, sys
from pxdex.dh15 import DexH15Kinematic, HandType
kin = DexH15Kinematic()
hw = sys.argv[1] if len(sys.argv) > 1 else ""
out = {"hw": hw}
def ik(q, side=HandType.RIGHT_HAND):
    r, m = kin.inverseKinematicParse(side, [float(x) for x in q], hw)
    return r, list(m)
z = [0.0] * 11
rows = []
for j1 in (0, 20, 40, 60, 80):
    for j2 in (0, 20, 40, 60, 80):
        q = z[:]; q[0] = j1; q[1] = j2
        rows.append([j1, j2] + list(ik(q)))
out["index_j1_j2"] = rows
th = []
for i in (8, 9, 10):
    for v in (0, 30, 60, 87):
        q = z[:]; q[i] = v
        th.append([i, v] + list(ik(q)))
out["thumb"] = th
r, n = kin.calculateNormalizeAngle([40.0, 20.0] + [0.0] * 9, hw)
out["normalize_40_20"] = [r, list(n)]
print(json.dumps(out))
