import os, sys, time
import numpy as np
import pytest
ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "scripts"))
URDF = os.path.join(ROOT, "vendor/yubi-pi_gmh15-sdk/models/right/meshes/DexH15_right_urdf.urdf")
pytestmark = pytest.mark.skipif(not os.path.exists(URDF), reason="vendor URDF not present")

from bare_hand_teleop import synthetic as S  # noqa: E402
from bare_hand_teleop.gmh15_model import GMH15Model  # noqa: E402
from bare_hand_teleop.hand_types import HandObservation  # noqa: E402
from bare_hand_teleop.opt_retarget import GMH15VectorRetargeter  # noqa: E402
from bare_hand_teleop.sim_pipeline import SimPipeline, chirality_score  # noqa: E402
from replay import synthetic_pinch  # noqa: E402


def right(p):
    p = p.copy(); p[:, 0] *= -1; return p


@pytest.fixture(scope="module")
def rt():
    return GMH15VectorRetargeter(GMH15Model(URDF))


def solve(rt, w, n=5):
    rt.reset()
    for _ in range(n):
        x = rt.solve(w)
    return x


def test_open_fist_closure(rt):
    assert np.all(solve(rt, S.open_hand(mirror=True))[:4] < 0.15)
    assert np.all(solve(rt, S.fist(mirror=True))[:4] > 0.85)


def test_pinch_brings_robot_tips_together(rt):
    solve(rt, S.open_hand(mirror=True)); d_open = rt.last["robot_pinch_m"]["index"]
    x = solve(rt, right(synthetic_pinch()))
    d_pinch = rt.last["robot_pinch_m"]["index"]
    assert d_pinch < 0.015 and d_pinch < 0.2 * d_open
    assert np.all(x[1:4] < 0.3)          # other fingers stay open during a thumb-index pinch


def test_individual_finger_follows(rt):
    w = S.make_hand({"index": (0.05, 0.05, 0.03), "middle": (1.4, 1.5, 1.0), "ring": (0.05, 0.05, 0.03),
                     "little": (0.05, 0.05, 0.03)}, mirror=True)
    x = solve(rt, w)
    assert x[1] > 0.7 and x[0] < 0.2 and x[2] < 0.25 and x[3] < 0.25


def test_output_within_urdf_limits(rt):
    x = solve(rt, S.fist(mirror=True))
    c = rt.to_cmd11(x)
    assert np.all(c[:8] >= 0) and np.all(c[:8] <= 81.6)
    assert 0 <= c[8] <= 109.6 and 0 <= c[9] <= 87 and 0 <= c[10] <= 120


def test_solve_is_fast_when_warm(rt):
    rt.reset(); op, fi = S.open_hand(mirror=True), S.fist(mirror=True)
    rt.solve(op); ts = []
    for k in range(30):
        t = time.perf_counter(); rt.solve(S.blend(op, fi, k / 29)); ts.append(time.perf_counter() - t)
    assert np.median(ts) < 0.02


def test_chirality_score_sign():
    assert chirality_score(S.fist(mirror=True)) > 0
    assert chirality_score(S.fist(mirror=False)) < 0


def test_sim_pipeline_holds_on_loss():
    p = SimPipeline("right", URDF)
    op = S.open_hand(mirror=True); fi = S.fist(mirror=True)
    for k in range(10):
        p.step(HandObservation(t=k * .033, detected=True, handedness="Right", score=.9, world=op), k * .033)
    for k in range(10, 40):
        p.step(HandObservation(t=k * .033, detected=True, handedness="Right", score=.9, world=fi), k * .033)
    held = p.guard.last.copy()
    r = p.step(HandObservation(t=1.4, detected=False), 1.4)
    assert r.command is None and r.state == "LOST" and np.allclose(p.guard.last, held)
    assert held[0] > 40      # closed toward fist before the loss


def test_calibration_maps_open_to_zero_and_keeps_pinch():
    from bare_hand_teleop.opt_retarget import OptConfig
    m = GMH15Model(URDF)
    w_open = S.blend(S.open_hand(mirror=True), S.fist(mirror=True), 0.25)    # "relaxed" half-open hand
    raw = GMH15VectorRetargeter(m)
    s_open = solve(raw, w_open)[:4]
    assert np.all(s_open > 0.05)
    cal = GMH15VectorRetargeter(m, OptConfig(open_s=tuple(s_open), fist_s=(1, 1, 1, 1)))
    assert np.all(solve(cal, w_open)[:4] < 0.03)
    assert np.all(solve(cal, S.fist(mirror=True))[:4] > 0.85)
    solve(raw, right(synthetic_pinch())); solve(cal, right(synthetic_pinch()))
    assert abs(cal.last["robot_pinch_m"]["index"] - raw.last["robot_pinch_m"]["index"]) < 0.003


def test_calib_roundtrip(tmp_path):
    from bare_hand_teleop.opt_retarget import OptConfig, load_calib, save_calib
    p = tmp_path / "c.json"
    save_calib(OptConfig(open_s=(0.1, 0.2, 0.3, 0.4)), p)
    c = load_calib(OptConfig(), p)
    assert c.open_s == (0.1, 0.2, 0.3, 0.4) and c.fist_s == (1, 1, 1, 1)


def test_thumb_hysteresis_keeps_basin(rt):
    # a high switch threshold of 0 means "never switch": the thumb must stay where the warm start is
    from bare_hand_teleop.opt_retarget import OptConfig
    r = GMH15VectorRetargeter(rt.m, OptConfig(thumb_switch_ratio=0.0))
    solve(r, S.open_hand(mirror=True))
    assert r.last["thumb_switched"] is False
