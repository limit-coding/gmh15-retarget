import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bare_hand_teleop import features as F, gmh15_spec as spec, synthetic as S  # noqa: E402
from bare_hand_teleop.hand_types import HandObservation  # noqa: E402
from bare_hand_teleop.pipeline import Pipeline  # noqa: E402
from bare_hand_teleop.retarget import FEATURE_KEYS, Calibration, LinearCurlRetargeter, RetargetConfig  # noqa: E402
from bare_hand_teleop.safety import GateConfig, GuardConfig, TargetGuard, TrackingGate  # noqa: E402


def calibrated():
    c = Calibration()
    c.set_open(F.extract(S.open_hand()).scalar_vector())
    c.set_fist(F.extract(S.fist()).scalar_vector())
    return c


# ---------------- features ----------------
def test_known_angles_recovered():
    p = S.make_hand({"index": (0.4, 0.7, 0.3)})
    f = F.extract(p)
    assert f.finger_mcp["index"] == pytest.approx(0.4, abs=1e-6)
    assert f.finger_pip["index"] == pytest.approx(0.7, abs=1e-6)
    assert f.finger_dip["index"] == pytest.approx(0.3, abs=1e-6)


def test_mirror_invariant():
    a = F.extract(S.fist()).scalar_vector()
    b = F.extract(S.fist(mirror=True)).scalar_vector()
    for k in a:
        assert a[k] == pytest.approx(b[k], abs=1e-9)


def test_abduction_not_read_as_flexion():
    p = S.open_hand()
    # rotate index finger chain in the palm plane by 15 deg about its MCP
    th = np.deg2rad(15)
    R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    for i in (6, 7, 8):
        p[i] = p[5] + R @ (p[i] - p[5])
    assert F.extract(p).finger_mcp["index"] < 0.06


def test_pinch_distance():
    p = S.open_hand()
    p[4] = p[8] + [0, 0, 0.01]
    f = F.extract(p)
    assert f.pinch_m["index"] == pytest.approx(0.01, abs=1e-9)


# ---------------- retarget ----------------
def test_open_and_fist_map_to_range_ends():
    c, r = calibrated(), LinearCurlRetargeter()
    q_open = r(c.normalize(F.extract(S.open_hand()).scalar_vector()))
    q_fist = r(c.normalize(F.extract(S.fist()).scalar_vector()))
    for i in spec.FINGER_CMD.values():
        assert q_open[i] == pytest.approx(r.lo[i], abs=1e-6)
        assert q_fist[i] == pytest.approx(r.hi[i], abs=1e-6)
    assert np.all(q_fist[list(spec.PASSIVE)] == 0)


def test_output_within_verified_range_and_passive_zero():
    r = LinearCurlRetargeter()
    for closure in (np.zeros(len(FEATURE_KEYS)), np.ones(len(FEATURE_KEYS))):
        q = r(closure)
        assert np.all(q >= r.lo) and np.all(q <= r.hi)
        assert np.all(q[list(spec.PASSIVE)] == 0)


def test_pinch_blend_reaches_pose():
    pose = {0: 20.0, 8: 25.0, 9: 10.0, 10: 12.0}
    r = LinearCurlRetargeter(RetargetConfig(pinch_pose_deg=pose))
    c = np.zeros(len(FEATURE_KEYS))
    c[FEATURE_KEYS.index("pinch_index")] = 1.0
    q = r(c)
    for i, v in pose.items():
        assert q[i] == pytest.approx(v)
    c[FEATURE_KEYS.index("pinch_index")] = 0.2       # far apart -> no blend
    assert r(c)[0] == pytest.approx(0.0)


# ---------------- gate ----------------
def test_gate_reasons():
    g = TrackingGate(GateConfig(expected_handedness="Right"))
    assert g.check(HandObservation(t=0, detected=False), 0)[1] == "no_hand"
    assert g.check(HandObservation(t=0, detected=True, handedness="Left", score=1), 0)[1].startswith("wrong_hand")
    assert g.check(HandObservation(t=0, detected=True, handedness="Right", score=0.2), 0)[1].startswith("low_score")
    assert g.check(HandObservation(t=0, detected=True, handedness="Right", score=1), 1.0)[1].startswith("stale")


def test_gate_jump_is_one_frame():
    g = TrackingGate()
    o = HandObservation(t=0, detected=True, handedness="Right", score=1)
    assert g.check(o, 0, np.zeros(8))[0]
    assert g.check(o, 0, np.ones(8))[1] == "jump"
    assert g.check(o, 0, np.ones(8))[0]          # persisted -> accepted
    one = np.zeros(8)
    one[0] = 1.0                                 # single noisy feature is not a glitch
    g.check(o, 0, np.zeros(8))
    assert g.check(o, 0, one)[0]


# ---------------- guard ----------------
def guard():
    lo, hi = spec.range_arrays()
    return TargetGuard(lo, hi, GuardConfig(max_rate_deg_s=90, reacquire_frames=3, resume_ramp_s=0.0))


def test_rate_limit_and_clamp():
    g, tgt = guard(), np.full(11, 100.0)
    outs = [g.update(k * 0.033, tgt, True) for k in range(40)]
    sent = [o for o in outs if o is not None]
    steps = np.diff(np.array(sent), axis=0)
    assert np.max(np.abs(steps)) <= 90 * 0.033 + 1e-9
    assert np.allclose(sent[-1][list(spec.ACTUATED)], g.hi[list(spec.ACTUATED)])
    assert np.all(sent[-1][list(spec.PASSIVE)] == 0)


def test_hold_on_loss_and_reacquire():
    g = guard()
    for k in range(10):
        g.update(k * 0.03, np.full(11, 20.0), True)
    held = g.last.copy()
    assert g.update(0.4, None, False, "no_hand") is None and g.state == "LOST"
    assert np.allclose(g.last, held)
    # need reacquire_frames valid frames before anything is sent again
    assert g.update(0.43, np.full(11, 40.0), True) is None
    assert g.update(0.46, np.full(11, 40.0), True) is None
    out = g.update(0.49, np.full(11, 40.0), True)
    assert out is not None and np.max(np.abs(out - held)) <= 90 * 0.03 + 1e-9


def test_long_gap_does_not_allow_big_step():
    g = guard()
    for k in range(5):
        g.update(k * 0.03, np.zeros(11), True)
    out = g.update(10.0, np.full(11, 45.0), True)   # 10 s later
    assert np.max(out) <= 90 * g.cfg.max_dt_s + 1e-9


def test_soft_resume_slower():
    lo, hi = spec.range_arrays()
    g = TargetGuard(lo, hi, GuardConfig(reacquire_frames=1, resume_ramp_s=1.0, resume_min_scale=0.25))
    g.update(0.0, np.zeros(11), True)
    out = g.update(0.03, np.full(11, 45.0), True)
    assert np.max(out) <= 90 * (0.25 + 0.75 * 0.03) * 0.03 + 1e-9   # ~0.74 deg vs 2.7 at full rate


def test_soft_jump_keeps_tracking():
    p = Pipeline(calibrated(), gate=GateConfig(expected_handedness="Right"),
                 guard=GuardConfig(reacquire_frames=2))
    mk = lambda t, w: HandObservation(t=t, detected=True, handedness="Right", score=0.9, world=w)
    for k in range(5):
        p.step(mk(k * 0.03, S.open_hand()), k * 0.03)
    assert p.guard.state == "TRACKING"
    r = p.step(mk(0.15, S.fist()), 0.15)            # all features jump at once
    assert r.reason == "jump" and r.command is None and p.guard.state == "TRACKING"
    r = p.step(mk(0.18, S.fist()), 0.18)
    assert r.valid and r.command is not None
