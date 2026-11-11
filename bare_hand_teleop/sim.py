"""Kinematic MuJoCo view of the GMH15 (no dynamics, no actuators: set qpos -> mj_forward -> render).

Model: vendor URDF converted on the Pi by prepare_mujoco_models.py (models/{side}/hand.xml).
That file hard-codes the container mesh dir (/opt/models/...); we override it here.
"""
import os

import mujoco
import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), "..")
MODEL_DIR = os.path.join(ROOT, "vendor/yubi-pi_gmh15-sdk/models")


class HandSim:
    def __init__(self, side="right", size=(480, 480), model_dir=MODEL_DIR):
        d = os.path.join(model_dir, side)
        spec = mujoco.MjSpec.from_file(os.path.join(d, "hand.xml"))
        spec.meshdir = os.path.join(d, "meshes") + os.sep
        spec.worldbody.add_light(pos=[0, 0, 1.0], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.side = side
        self.qadr = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, j): self.m.jnt_qposadr[j]
                     for j in range(self.m.njnt)}
        self.renderer = mujoco.Renderer(self.m, size[1], size[0])
        self.cam = mujoco.MjvCamera()
        mujoco.mj_forward(self.m, self.d)
        self.cam.lookat[:] = self.d.xpos[1:].mean(axis=0) if self.m.nbody > 1 else 0
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 0.42, 150.0, -25.0
        self.markers = []

    def look_from(self, direction, distance=0.38):
        """Place the free camera on `direction` (model frame) from the hand centre."""
        d = np.asarray(direction, float)
        d = d / np.linalg.norm(d)
        f = -d
        self.cam.azimuth = float(np.degrees(np.arctan2(f[1], f[0])))
        self.cam.elevation = float(np.degrees(np.arcsin(np.clip(f[2], -1, 1))))
        self.cam.distance = distance

    def set_q(self, q):
        """q: {urdf_joint_name: radians}."""
        for name, v in q.items():
            if name in self.qadr:
                self.d.qpos[self.qadr[name]] = v
        mujoco.mj_forward(self.m, self.d)

    def render(self, markers=()):
        """markers: iterable of (xyz in model frame, rgba). Returns RGB uint8 image."""
        self.renderer.update_scene(self.d, self.cam)
        scn = self.renderer.scene
        for xyz, rgba in markers:
            if scn.ngeom >= scn.maxgeom:
                break
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, [0.004, 0, 0],
                                np.asarray(xyz, float), np.eye(3).flatten(), np.asarray(rgba, np.float32))
            scn.ngeom += 1
        return self.renderer.render()

    def close(self):
        self.renderer.close()
