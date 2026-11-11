"""Mac controller: position initialization runs the vendor fault-recovery step first."""
import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mac_driver'))
spec = importlib.util.spec_from_file_location('mac_test_teleop_control', ROOT / 'mac_driver/pi_snapshot/test_teleop_control.py')
vendor_tests = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vendor_tests)
from teleop_control import AuditedHandProxy, Controller  # noqa: E402


class LatchedHand(vendor_tests.FakeHand):
    """Models H15R49G00039: one motor's zero state stays latched until clearFaultCode."""
    def __init__(self):
        super().__init__()
        self.latched = True

    def clearFaultCode(self, a):
        self.calls.append(('clear',))
        self.latched = False
        return True

    def initMotorPosition(self, a):
        self.calls.append(('initialize',))
        self.enabled = True
        return 0 if self.latched else self.init_result


def make(hand):
    return Controller(hand, 1, vendor_tests.FakeKin(), 'right', 'E', 'POSITION', list, lambda v: ('speed', v))


def test_clear_runs_before_init_and_motor_is_disabled_after():
    hand = LatchedHand()
    ctrl = make(hand)
    ctrl.initialize(vendor_tests.OWNER)
    assert ctrl.last_init_result == 1
    assert [c[0] for c in hand.calls] == ['clear', 'initialize', 'disable']
    assert ctrl.initialization_diagnostics['clear_fault'] is True
    assert not hand.enabled and not ctrl.active and not ctrl.initializing


def test_failed_init_still_disables():
    hand = LatchedHand()
    hand.init_result = 0
    ctrl = make(hand)
    ctrl.initialize(vendor_tests.OWNER)
    assert ctrl.last_init_result == 0 and ctrl.error
    assert hand.calls[-1] == ('disable',) and not hand.enabled


def test_audited_proxy_allows_and_counts_clear():
    hand = LatchedHand()
    proxy = AuditedHandProxy(hand)
    assert proxy.clearFaultCode(1) is True
    assert proxy.write_counts == {'clearFaultCode': 1}


class MotorHand(LatchedHand):
    def __init__(self):
        super().__init__()
        self.latched = False
        self.motor_writes = []

    def setMotorTargetPosition(self, a, v):
        self.calls.append(('seed', v))
        if self.enabled:
            self.motor_writes.append(list(v))
        return 1 if self.enabled else 0


def armed(now):
    import time as _t
    hand = MotorHand()
    clock = {'t': now}
    ctrl = Controller(hand, 1, vendor_tests.FakeKin(), 'right', 'E', 'POSITION', list, lambda v: ('speed', v),
                      clock=lambda: clock['t'])
    frame = dict(connected=True, joint_error=None, timestamp=_t.time(), joints_degrees=[0.] * 15,
                 motor_positions=[100] * 7)
    ctrl.last_init_result = 1
    ctrl.arm(vendor_tests.OWNER, frame)
    return hand, ctrl, frame, clock


def test_raw_motor_mode_slews_and_validates():
    import pytest
    import teleop_control as tc
    import time as _t
    hand, ctrl, frame, clock = armed(10.)
    with pytest.raises(ValueError):
        ctrl.set_motor_target(vendor_tests.OWNER, 1, [float('nan')] * 7)
    with pytest.raises(ValueError):
        ctrl.set_motor_target('someone-else-0000001', 1, [0] * 7)
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [20000] * 7)          # clamped to the ceilings
    assert ctrl.motor_target == list(tc.MOTOR_HI)
    ctrl.set_motor_target(vendor_tests.OWNER, 2, [9000] + [100] * 6)
    for _ in range(3):
        clock['t'] += .11
        ctrl.last_heartbeat = clock['t']
        frame['timestamp'] = _t.time()
        frame['motor_positions'] = list(ctrl.motor_sent)       # motors follow perfectly
        ctrl.tick(frame)
    step = tc.MOTOR_SLEW_CPS * .11
    assert ctrl.active
    assert abs(hand.motor_writes[-1][0] - (100 + 3 * step)) <= 3        # slew-limited, not a jump
    assert hand.motor_writes[-1][1:] == [100] * 6


def test_raw_motor_mode_blocked_motor_keeps_bounded_lead():
    """Changed 2026-11-07: a blocked motor (object contact) no longer trips the error stop; the
    command stays within CONTACT_LEAD of the measured position and force is capped by
    GRIP_CURRENT (over-current guard still applies above CURRENT_STOP)."""
    import time as _t
    import teleop_control as tc
    hand, ctrl, frame, clock = armed(10.)
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [10000] + [100] * 6)
    for _ in range(40):                                        # motor blocked at 100
        clock['t'] += .15
        ctrl.last_heartbeat = clock['t']
        frame['timestamp'] = _t.time()
        ctrl.tick(frame)
    assert ctrl.active
    assert hand.motor_writes[-1][0] == 100 + tc.CONTACT_LEAD



def test_over_current_stops_in_both_modes():
    import time as _t
    import teleop_control as tc
    for raw in (True, False):
        hand, ctrl, frame, clock = armed(10.)
        if raw:
            ctrl.set_motor_target(vendor_tests.OWNER, 1, [100] * 7)
        frame['motor_current'] = [0] * 5 + [tc.CURRENT_STOP + 50, 0]
        for _ in range(tc.CURRENT_STOP_FRAMES + 1):
            clock['t'] += .11
            ctrl.last_heartbeat = clock['t']
            frame['timestamp'] = _t.time()
            ctrl.tick(frame)
        assert not ctrl.active and not hand.enabled and '[6]' in ctrl.error


def test_single_current_spike_does_not_stop():
    import time as _t
    import teleop_control as tc
    hand, ctrl, frame, clock = armed(10.)
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [100] * 7)
    for k in range(4):
        frame['motor_current'] = [tc.CURRENT_STOP + 50 if k == 1 else 0] + [0] * 6
        clock['t'] += .11
        ctrl.last_heartbeat = clock['t']
        frame['timestamp'] = _t.time()
        ctrl.tick(frame)
    assert ctrl.active


# ---------------- grasp / hold (2026-11-07) ----------------
class GripHand(MotorHand):
    def __init__(self):
        super().__init__()
        self.max_current = 1000

    def getMotorMaxCurrentConfig(self, a):
        return self.max_current

    def setMotorMaxCurrent(self, a, v):
        self.calls.append(('current', v))
        self.max_current = v
        return True


def armed_grip():
    import time as _t
    hand = GripHand()
    clock = {'t': 10.}
    ctrl = Controller(hand, 1, vendor_tests.FakeKin(), 'right', 'E', 'POSITION', list, lambda v: ('speed', v),
                      clock=lambda: clock['t'], make_currents=lambda v: ('cur', v))
    frame = dict(connected=True, joint_error=None, timestamp=_t.time(), joints_degrees=[0.] * 15,
                 motor_positions=[0] * 7, motor_current=[0] * 7)
    ctrl.last_init_result = 1
    ctrl.arm(vendor_tests.OWNER, frame)
    return hand, ctrl, frame, clock


def step(ctrl, frame, clock, n=1, heartbeat=True, follow=None):
    import time as _t
    for _ in range(n):
        clock['t'] += .11
        if heartbeat:
            ctrl.last_heartbeat = clock['t']
        frame['timestamp'] = _t.time()
        if follow and ctrl.motor_sent:
            frame['motor_positions'] = follow(list(ctrl.motor_sent))
        ctrl.tick(frame)


def test_grip_current_cap_set_on_arm_and_restored_on_stop():
    import teleop_control as tc
    hand, ctrl, frame, clock = armed_grip()
    assert hand.max_current == ('cur', tc.GRIP_CURRENT)
    ctrl.stop('x')
    assert hand.max_current == 1000


def test_object_contact_squeezes_without_tripping():
    import teleop_control as tc
    hand, ctrl, frame, clock = armed_grip()
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [10000] * 5 + [7200, 7700])
    # fingers close freely until 4000, then the bottle blocks them
    step(ctrl, frame, clock, n=60, follow=lambda sent: [min(v, 4000) for v in sent])
    assert ctrl.active, ctrl.error
    assert max(ctrl.motor_sent) <= 4000 + tc.CONTACT_LEAD        # bounded squeeze, no runaway


def test_hold_survives_without_heartbeats_and_ignores_late_targets():
    hand, ctrl, frame, clock = armed_grip()
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [3000] * 7)
    step(ctrl, frame, clock, n=10, follow=lambda sent: sent)
    ctrl.hold(vendor_tests.OWNER)
    frozen = list(ctrl.motor_target)
    step(ctrl, frame, clock, n=50, heartbeat=False, follow=lambda sent: sent)   # client gone ~5 s
    assert ctrl.active and ctrl.holding
    ctrl.set_motor_target(vendor_tests.OWNER, 99, [0] * 7)                    # late teleop target
    step(ctrl, frame, clock, n=5, heartbeat=False, follow=lambda sent: sent)
    assert ctrl.motor_target == frozen and ctrl.active


def test_release_opens_then_disables():
    hand, ctrl, frame, clock = armed_grip()
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [3000] * 7)
    step(ctrl, frame, clock, n=10, follow=lambda sent: sent)
    ctrl.hold(vendor_tests.OWNER)
    ctrl.release()
    step(ctrl, frame, clock, n=40, heartbeat=False, follow=lambda sent: sent)
    assert not ctrl.active and not hand.enabled and not ctrl.holding
    assert hand.motor_writes[-1] == [0] * 7                                    # opened before disabling


def test_stop_still_works_during_hold():
    hand, ctrl, frame, clock = armed_grip()
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [3000] * 7)
    step(ctrl, frame, clock, n=5, follow=lambda sent: sent)
    ctrl.hold(vendor_tests.OWNER)
    ctrl.stop('急停')
    assert not ctrl.active and not hand.enabled and not ctrl.holding


def test_send_deadline_survives_acquisition_jitter(monkeypatch):
    """100 ms samples alternate 98/102 ms: resetting last_send used to halve rate."""
    import time as _t
    import teleop_control as tc
    monkeypatch.setattr(tc, 'MAX_RATE_HZ', 10.)
    hand, ctrl, frame, clock = armed(10.)
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [1000] * 7)
    for n in range(40):
        clock['t'] = 10. + (n+1)*.1 + (-.002 if n % 2 == 0 else .002)
        ctrl.last_heartbeat = clock['t']
        frame['timestamp'] = _t.time()
        frame['motor_positions'] = list(ctrl.motor_sent)
        ctrl.tick(frame)
    assert ctrl.active and ctrl.command_count >= 38


def test_30hz_keeps_current_stop_and_slew(monkeypatch):
    import time as _t
    import teleop_control as tc
    monkeypatch.setattr(tc, 'MAX_RATE_HZ', 30.)
    hand, ctrl, frame, clock = armed(10.)
    ctrl.set_motor_target(vendor_tests.OWNER, 1, [5000] * 7)
    clock['t'] += 1/30 + .001
    ctrl.last_heartbeat = clock['t']
    frame['timestamp'] = _t.time()
    ctrl.tick(frame)
    assert ctrl.active and ctrl.command_count == 1
    assert max(ctrl.motor_sent) <= 100 + tc.MOTOR_SLEW_CPS * (1/30 + .001) + 1
    frame['motor_current'] = [tc.CURRENT_STOP + 1] + [0]*6
    for _ in range(2):
        clock['t'] += 1/60
        ctrl.last_heartbeat = clock['t']
        frame['timestamp'] = _t.time()
        ctrl.tick(frame)
    assert not ctrl.active and not hand.enabled
