import os, sys
import numpy as np
import pytest
ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)
from bare_hand_teleop.gmh15_model import GMH15Model  # noqa: E402

URDF = os.path.join(ROOT, "vendor/yubi-pi_gmh15-sdk/models/right/meshes/DexH15_right_urdf.urdf")
pytestmark = pytest.mark.skipif(not os.path.exists(URDF), reason="vendor URDF not present")


def test_urdf_structure():
    m = GMH15Model(URDF)
    assert len(m.revolute) == 15 and m.side == "right"
    assert np.rad2deg(m.joints["right_thumb_joint_0"]["limit"][1]) == pytest.approx(109.6, abs=0.1)


def test_flexing_index_moves_tip_toward_thumb():
    m = GMH15Model(URDF)
    z = np.zeros(11)
    d0 = np.linalg.norm(m.tip("thumb", m.q_from_cmd(z)) - m.tip("index", m.q_from_cmd(z)))
    c = z.copy(); c[0] = 60; c[8] = 80; c[9] = 50
    d1 = np.linalg.norm(m.tip("thumb", m.q_from_cmd(c)) - m.tip("index", m.q_from_cmd(c)))
    assert d1 < d0 * 0.5
