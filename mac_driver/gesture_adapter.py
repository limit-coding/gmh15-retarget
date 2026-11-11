"""Reuse official gesture steps with this hand's measured motor travel limits."""
import math

MOTOR_HI = (10000, 10000, 10000, 10000, 10000, 7200, 7700)


class BoundedGestureHand:
    def __init__(self, hand, kin, hand_type, hardware_version, make_positions, log):
        self.hand, self.kin = hand, kin
        self.hand_type, self.hw = hand_type, hardware_version
        self.make_positions, self.log = make_positions, log

    def __getattr__(self, name):
        return getattr(self.hand, name)

    def setJointPositionsAngle(self, address, values):
        if len(values) != 11 or any(not math.isfinite(v) for v in values):
            raise ValueError('官方手势关节目标无效')
        result, real = self.kin.calculateRealAngle(values, self.hw)
        if result != 1 or len(real) != 11 or any(not math.isfinite(v) for v in real):
            raise RuntimeError('官方手势角度转换失败')
        result, motors = self.kin.inverseKinematicParse(self.hand_type, real, self.hw)
        if result != 1 or len(motors) != 7 or any(not math.isfinite(v) for v in motors):
            raise RuntimeError('官方手势逆解失败')
        bounded = [min(max(round(v), 0), hi) for v, hi in zip(motors, MOTOR_HI)]
        self.log('gesture_target', normalized=list(values), original_motors=list(motors),
                 motor_target=bounded, limited=bounded != list(motors))
        return self.hand.setMotorTargetPosition(address, self.make_positions(bounded))
