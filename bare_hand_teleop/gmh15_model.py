"""Minimal numpy forward kinematics for the vendor GMH15 URDF (DexH15_{left,right}_urdf).

Source: vendor archive "3D drawing URDF.rar" (identical to ~/gmh15-sdk/models/*/meshes on the Pi).
15 revolute joints, no <mimic>: the URDF does NOT encode the motor/tendon coupling. In the SDK,
each finger's joint_0 and joint_1 both feed one motor (see tools/ik_coupling_probe.py), so a
finger posture is one point on a mechanism-defined curve in (joint_0, joint_1). Which curve is
not known yet; callers pass joint_1 explicitly.

Fingertip point = centroid of the fingertip tactile-pad mesh (…_tactile_link_0.STL).
"""
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, a):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def _tf(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def read_stl(path):
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="ignore").splitlines()
             if ln.strip().startswith("vertex")]
        return np.array(v)
    n = struct.unpack("<I", data[80:84])[0]
    arr = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return arr["v"].reshape(-1, 3).astype(float)


class GMH15Model:
    def __init__(self, urdf_path):
        self.root_dir = os.path.dirname(urdf_path)
        r = ET.parse(urdf_path).getroot()
        self.joints = {}
        self.child_joint = {}
        for j in r.iter("joint"):
            o = j.find("origin")
            xyz = np.array(list(map(float, (o.get("xyz") or "0 0 0").split()))) if o is not None else np.zeros(3)
            rpy = np.array(list(map(float, (o.get("rpy") or "0 0 0").split()))) if o is not None else np.zeros(3)
            ax = j.find("axis")
            lim = j.find("limit")
            jd = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                      child=j.find("child").get("link"), T0=_tf(_rpy(*rpy), xyz),
                      axis=np.array(list(map(float, ax.get("xyz").split()))) if ax is not None else None,
                      limit=(float(lim.get("lower")), float(lim.get("upper"))) if lim is not None and j.get("type") == "revolute" else None)
            self.joints[jd["name"]] = jd
            self.child_joint[jd["child"]] = jd
        self.meshes = {}
        for link in r.iter("link"):
            m = link.find("visual/geometry/mesh")
            if m is not None:
                self.meshes[link.get("name")] = os.path.basename(m.get("filename"))
        self.side = "right" if any(n.startswith("right_") for n in self.joints) else "left"
        self.revolute = [n for n, j in self.joints.items() if j["type"] == "revolute"]
        self._tip_local = {}

    def link_tf(self, link, q):
        """q: {joint_name: radians}. Returns 4x4 transform palm->link."""
        T = np.eye(4)
        chain = []
        while link in self.child_joint:
            j = self.child_joint[link]
            chain.append(j)
            link = j["parent"]
        for j in reversed(chain):
            T = T @ j["T0"]
            if j["type"] == "revolute":
                T = T @ _tf(_axis_angle(j["axis"], q.get(j["name"], 0.0)), np.zeros(3))
        return T

    def tip_local(self, finger):
        link = f"{self.side}_{finger}_tactile_link_0"
        if link not in self._tip_local:
            v = read_stl(os.path.join(self.root_dir, self.meshes.get(link, link + ".STL")))
            self._tip_local[link] = v.mean(axis=0)
        return self._tip_local[link]

    def tip(self, finger, q):
        link = f"{self.side}_{finger}_tactile_link_0"
        T = self.link_tf(link, q)
        return (T @ np.r_[self.tip_local(finger), 1.0])[:3]

    def q_from_cmd(self, cmd_deg, dip_deg=None):
        """11-entry teleop command (deg) -> URDF joint dict (rad). Finger joint_2 (DIP) is not in the
        command; default 0 (unknown coupling)."""
        s = self.side
        q = {}
        for k, f in enumerate(("index", "middle", "ring", "pinky")):
            q[f"{s}_{f}_joint_0"] = np.deg2rad(cmd_deg[2 * k])
            q[f"{s}_{f}_joint_1"] = np.deg2rad(cmd_deg[2 * k + 1])
            q[f"{s}_{f}_joint_2"] = np.deg2rad(0.0 if dip_deg is None else dip_deg[k])
        for k, i in enumerate((8, 9, 10)):
            q[f"{s}_thumb_joint_{k}"] = np.deg2rad(cmd_deg[i])
        return q
