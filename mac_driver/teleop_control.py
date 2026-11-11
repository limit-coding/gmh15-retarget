"""GMH15 bounded position teleoperation; initialization requires a separate UI click.

Real degrees -> official calculateNormalizeAngle -> setJointPositionsAngle.
One acquisition process owns each serial port and this controller.
"""
import math
import os
import time

EDITABLE = (0, 1, 3, 4, 6, 7, 9, 10, 12, 13, 14)
LIMITS = ((0., 80.),) * 8 + ((0., 109.6), (0., 87.), (0., 120.1))
# GMH15: 7 motors = 4 finger MCPs + thumb CMC/MCP/IP. Finger PIPs (11-index 1,3,5,7)
# have no independent motor or feedback; SDK FK always reports 0. Commands must keep them at 0.
PASSIVE = (1, 3, 5, 7)
JOINT_NAMES = ('食指 MCP', '食指 PIP', '中指 MCP', '中指 PIP', '无名指 MCP',
               '无名指 PIP', '小指 MCP', '小指 PIP', '拇指 CMC', '拇指 MCP', '拇指 IP')
WATCHDOG_S = .6
MAX_RATE_HZ = float(os.environ.get('GMH15_CONTROL_HZ', 10))
if not math.isfinite(MAX_RATE_HZ) or not 5 <= MAX_RATE_HZ <= 30:
    raise ValueError('GMH15_CONTROL_HZ must be between 5 and 30')
# Teleop speed. Pi first-test values were 8 deg/s, motor speed 100/1000, 10 deg error; raised
# 2026-11-07 for hand following (operator request). Override per run with environment variables.
SLEW_DEG_S = float(os.environ.get('GMH15_SLEW_DEG_S', 60.))
MOTOR_SPEED = int(os.environ.get('GMH15_MOTOR_SPEED', 1000))           # 0-1000 SDK max velocity
MAX_TRACKING_ERROR_DEG = float(os.environ.get('GMH15_MAX_TRACKING_ERROR_DEG', 25.))
# Raw motor mode (diagnostics page /motors): SDK motor position counts 0-10000 per motor.
MOTOR_MAX_COUNT = 10000
# Per-motor ceilings from the 2026-11-07 sweep: thumb MCP / IP hit mechanical end stops at
# ~7500 / ~8000 counts (current > 500); fingers and thumb CMC run the full 0-10000.
MOTOR_HI = (10000, 10000, 10000, 10000, 10000, 7200, 7700)
MOTOR_SLEW_CPS = float(os.environ.get('GMH15_MOTOR_SLEW_CPS', 8000.))      # counts per second
MOTOR_MAX_ERROR = int(os.environ.get('GMH15_MOTOR_MAX_ERROR', 2500))        # stall -> stop
# Over-current guard (SDK current limit is 1000; free motion peaked at ~250 in the sweep).
# Any motor above this for CURRENT_STOP_FRAMES consecutive frames -> disable (both modes).
CURRENT_STOP = int(os.environ.get('GMH15_CURRENT_STOP', 600))
CURRENT_STOP_FRAMES = 2
# Grasping (2026-11-07). While enabled the SDK motor current limit is lowered to GRIP_CURRENT,
# which caps finger force; free motion peaked at ~270. Raw-mode targets may lead the measured
# position by at most CONTACT_LEAD counts, so a finger blocked by an object keeps a bounded
# squeeze instead of tripping the tracking-error stop.
GRIP_CURRENT = int(os.environ.get('GMH15_GRIP_CURRENT', 450))
CONTACT_LEAD = int(os.environ.get('GMH15_CONTACT_LEAD', 1500))
# Hold: the server keeps the frozen grasp without client heartbeats until "release" / stop.
HOLD_MAX_S = float(os.environ.get('GMH15_HOLD_MAX_S', 1800.))
RELEASE_DONE_COUNTS = 400


class ReadOnlyHandProxy:
    """SDK allowlist used for the motion-locked deployment. No motor writes pass."""
    READ_CALLS = frozenset(('openUsbDevice','disconnectUsb','getDeviceSN','getHandType',
        'getHardwareVersion','isMotorEnabled','getFingerTactileData','getPressPositionForce',
        'getMotorPosition','getJointMagneticEncoder','getFirmwareVersion','getFaultCode','checkHandStatus','getMotorControlMode',
        'getMotorMaxVelocityConfig','getMotorMaxCurrentConfig','getMotorActualCurrent'))

    def __init__(self, hand):
        self._hand = hand
        self.blocked_write_attempts = 0

    def __getattr__(self, name):
        if name not in self.READ_CALLS:
            self.blocked_write_attempts += 1
            raise RuntimeError('本次部署锁定运动，禁止SDK接口: '+name)
        return getattr(self._hand,name)


class AuditedHandProxy:
    READ_CALLS=ReadOnlyHandProxy.READ_CALLS | frozenset(('checkHandStatus','getFaultCode','getMotorMaxVelocityConfig'))
    WRITE_CALLS=frozenset(('setMotorControlMode','setMotorMaxVelocity','setMotorMaxCurrent','setMotorTargetPosition',
                         'enableMotor','disableMotor','setJointPositionsAngle','initMotorPosition','clearFaultCode'))
    def __init__(self,hand):self._hand=hand;self.write_counts={}
    def __getattr__(self,name):
        if name in self.READ_CALLS:return getattr(self._hand,name)
        if name not in self.WRITE_CALLS:raise RuntimeError('Unsupported SDK interface: '+name)
        def call(*args,**kwargs):
            self.write_counts[name]=self.write_counts.get(name,0)+1
            return getattr(self._hand,name)(*args,**kwargs)
        return call


def angles11(values):
    if not isinstance(values, (list, tuple)) or len(values) != 11:
        raise ValueError('目标必须是11个真实角度（度），不能传15个反馈角或归一化值')
    result = []
    for i, (value, (low, high)) in enumerate(zip(values, LIMITS)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('目标必须为有限数值')
        if not low <= value <= high:
            raise ValueError(f'{JOINT_NAMES[i]}超出范围 {low}–{high}°')
        result.append(float(value))
    return result


def command11(values):
    result = angles11(values)
    for i in PASSIVE:
        if result[i] != 0.0:
            raise ValueError(f'{JOINT_NAMES[i]}无独立电机，目标必须为0°')
    return result


def measured11(frame):
    if (not frame.get('connected') or frame.get('joint_error') or
            time.time() - frame.get('timestamp', 0) > .5 or len(frame.get('joints_degrees', [])) != 15):
        raise ValueError('没有新鲜且有效的15关节反馈')
    result = angles11([frame['joints_degrees'][i] for i in EDITABLE])
    for i in PASSIVE:
        result[i] = 0.0
    return result


def motor_counts(values):
    if not isinstance(values, (list, tuple)) or len(values) != 7:
        raise ValueError('电机目标必须是7个计数值')
    out = []
    for v in values:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError('电机目标必须为有限数值')
        out.append(int(round(v)))
    # Clamp (not reject) to each motor's measured ceiling so a slider at 10000 stays safe.
    return [min(max(v, 0), hi) for v, hi in zip(out, MOTOR_HI)]


class Controller:
    def __init__(self, hand, address, kin, hand_type, hardware_version, position_mode,
                 make_positions, make_speeds, clock=time.monotonic, make_currents=None):
        self.hand, self.addr, self.kin = hand, address, kin
        self.hand_type, self.hw, self.position_mode = hand_type, hardware_version, position_mode
        self.make_positions, self.make_speeds, self.clock = make_positions, make_speeds, clock
        self.make_currents = make_currents
        self.old_current = None
        self.holding = False
        self.releasing = False
        self.hold_started = None
        self.active = False
        self.owner = None
        self.last_heartbeat = 0
        self.last_send = 0
        self.next_send = 0
        self.last_sequence = -1
        self.target = None
        self.sent = None
        self.old_speed = None
        self.touched_enable = False
        self.error = None
        self.reason = '默认只预览；尚未使能'
        self.command_count = 0
        self.initializing = False
        self.initialization_count = 0
        self.last_init_result = None
        self.initialization_diagnostics = None
        self.over_current_frames = 0
        self.motor_target = None      # raw mode: 7 counts; None = joint-angle mode
        self.motor_sent = None
        self.resync = False

    def status(self):
        return dict(active=self.active, owner=self.owner, error=self.error, reason=self.reason,
                    target_degrees=self.target, sent_degrees=self.sent, command_count=self.command_count,
                    last_sequence=self.last_sequence, watchdog_ms=int(WATCHDOG_S * 1000),
                    slew_deg_s=SLEW_DEG_S, max_send_hz=MAX_RATE_HZ,initializing=self.initializing,
                    initialization_count=self.initialization_count,last_init_result=self.last_init_result,
                    initialization_diagnostics=self.initialization_diagnostics,
                    motor_target=self.motor_target, motor_sent=self.motor_sent,
                    motor_slew_cps=MOTOR_SLEW_CPS, motor_max_error=MOTOR_MAX_ERROR,
                    motor_hi=list(MOTOR_HI), current_stop=CURRENT_STOP,
                    holding=self.holding, releasing=self.releasing, grip_current=GRIP_CURRENT,
                    hold_remaining_s=None if not self.holding else
                    max(0, round(HOLD_MAX_S - (self.clock() - self.hold_started))))

    def initialize(self,owner):
        if self.active or self.initializing or self.hand.isMotorEnabled(self.addr):
            raise ValueError('请先停止并确认电机未使能')
        if self.hand.checkHandStatus(self.addr)!=1 or self.hand.getFaultCode(self.addr):
            raise ValueError('初始化前设备或故障检查未通过')
        self.owner=owner;self.initializing=True;self.error=None;self.last_init_result=None
        started=self.clock()
        self.initialization_diagnostics={}
        try:
            self.initialization_diagnostics['mode_before']=str(self.hand.getMotorControlMode(self.addr))
            # Do not change velocity config before the firmware's own initialization routine.
            # Vendor fault-recovery flow first: a motor can hold a latched zero state (reg 0x2120 = 1)
            # that getFaultCode does not report; initMotorPosition then times out (~5 s, returns 0)
            # until clearFaultCode is sent. Verified on H15R49G00039 motor 1, 2026-11-07.
            self.touched_enable=True
            self.initialization_diagnostics['clear_fault']=bool(self.hand.clearFaultCode(self.addr))
            self.initialization_count+=1
            self.last_init_result=int(self.hand.initMotorPosition(self.addr))
            if self.last_init_result!=1:raise RuntimeError('位置初始化失败（SDK返回'+str(self.last_init_result)+'）')
            if self.hand.getFaultCode(self.addr):raise RuntimeError('初始化后检测到设备故障')
            self.reason='初始化完成；等待新鲜关节反馈'
        except Exception as exc:
            self.error=str(exc);self.reason='初始化失败；已请求失能'
        finally:
            try:
                self.initialization_diagnostics.update(duration_ms=round((self.clock()-started)*1000),
                    faults_after=[str(getattr(f,'error_code',f)) for f in self.hand.getFaultCode(self.addr)])
            except Exception:
                self.initialization_diagnostics['diagnostic_read_failed']=True
            finally:
                reason=self.reason
                self.stop(reason)
                self.initializing=False

    def arm(self, owner, frame):
        if self.active:
            raise ValueError('已有控制会话，先停止')
        if self.last_init_result!=1:
            raise ValueError('位置初始化尚未成功，禁止使能控制')
        if not isinstance(owner, str) or not 16 <= len(owner) <= 96:
            raise ValueError('无效的浏览器会话')
        target = measured11(frame)
        if self.hand.isMotorEnabled(self.addr):
            raise ValueError('电机已由其他程序使能，拒绝接管')
        if self.hand.checkHandStatus(self.addr) != 1 or self.hand.getFaultCode(self.addr):
            raise ValueError('设备状态或故障检查未通过')
        motors = frame.get('motor_positions', [])
        if len(motors) != 7 or any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 10000 for v in motors):
            raise ValueError('当前7路电机位置无效')
        self.error = None
        self.owner = owner
        self.target = target[:]
        self.sent = target[:]
        self.motor_target = None
        self.motor_sent = list(motors)
        self.resync = False
        self.over_current_frames = 0
        self.last_sequence = -1
        try:
            self.old_speed = self.hand.getMotorMaxVelocityConfig(self.addr)
            if not self.hand.setMotorControlMode(self.addr, self.position_mode):
                raise RuntimeError('设置位置模式失败')
            if not self.hand.setMotorMaxVelocity(self.addr, self.make_speeds(MOTOR_SPEED)):
                raise RuntimeError('设置低速上限失败')
            if self.make_currents is not None:      # grip force cap
                self.old_current = self.hand.getMotorMaxCurrentConfig(self.addr)
                if not self.hand.setMotorMaxCurrent(self.addr, self.make_currents(GRIP_CURRENT)):
                    raise RuntimeError('设置握力电流上限失败')
            self.touched_enable = True
            if not self.hand.enableMotor(self.addr) or not self.hand.isMotorEnabled(self.addr):
                raise RuntimeError('使能未确认')
            # SDK rejects target writes while disabled. Enable at the configured
            # low velocity, then immediately hold the measured motor positions.
            if self.hand.setMotorTargetPosition(self.addr, self.make_positions(motors)) != 1:
                raise RuntimeError('同步当前电机目标失败，已请求失能')
            self.active = True
            self.last_heartbeat = self.last_send = self.clock()
            self.next_send = self.last_send + 1. / MAX_RATE_HZ
            self.reason = '实机已启用；拖拽直接控制，点击停止可失能'
        except Exception:
            self.stop('启用失败')
            raise

    def heartbeat(self, owner):
        if self.active and owner == self.owner:
            self.last_heartbeat = self.clock()

    def set_target(self, owner, sequence, target):
        if not self.active or owner != self.owner:
            raise ValueError('当前浏览器没有控制权')
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= self.last_sequence:
            raise ValueError('目标序号过期')
        if self.holding or self.releasing:      # never let a late client target break a hold
            return
        self.target = command11(target)
        self.last_sequence = sequence
        if self.motor_target is not None:      # leaving raw mode: restart slew from measured angles
            self.motor_target = None
            self.resync = True

    def set_motor_target(self, owner, sequence, counts):
        if not self.active or owner != self.owner:
            raise ValueError('当前浏览器没有控制权')
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= self.last_sequence:
            raise ValueError('目标序号过期')
        counts = motor_counts(counts)
        if self.holding or self.releasing:      # ignore late targets from the teleop client
            return
        self.motor_target = counts
        self.last_sequence = sequence

    def hold(self, owner):
        """Freeze the current grasp; the server keeps it without client heartbeats."""
        if not self.active or owner != self.owner:
            raise ValueError('当前浏览器没有控制权')
        if self.motor_target is None:
            raise ValueError('保持握持只支持电机位置模式')
        self.motor_target = list(self.motor_sent)
        self.holding, self.releasing = True, False
        self.hold_started = self.clock()
        self.reason = '保持握持中（不需要遥操作）；点「松开」张开手'

    def release(self):
        """Open slowly to zero, then disable. Any client may request it (like stop)."""
        if not self.active:
            return
        if self.motor_sent is None:
            raise ValueError('没有电机目标可松开')
        self.holding, self.releasing = False, True
        self.motor_target = [0] * 7
        self.reason = '正在松开…'

    def stop(self, reason='用户停止'):
        cleanup = []
        # The owner serial process performs cleanup; HTTP and renderer never open the port.
        if self.touched_enable:
            try:
                if not self.hand.disableMotor(self.addr) or self.hand.isMotorEnabled(self.addr):
                    cleanup.append('失能未确认，请检查实物')
            except Exception as exc:
                cleanup.append('失能失败: ' + str(exc))
        if self.old_speed is not None:
            try:
                if not self.hand.setMotorMaxVelocity(self.addr, self.old_speed):
                    cleanup.append('原速度配置恢复失败')
            except Exception as exc:
                cleanup.append('速度恢复失败: ' + str(exc))
        if self.old_current is not None:
            try:
                if not self.hand.setMotorMaxCurrent(self.addr, self.old_current):
                    cleanup.append('原电流上限恢复失败')
            except Exception as exc:
                cleanup.append('电流上限恢复失败: ' + str(exc))
        self.old_current = None
        self.holding = self.releasing = False
        self.hold_started = None
        self.active = False
        self.owner = None
        self.old_speed = None
        self.touched_enable = False
        self.reason = reason
        if cleanup:
            self.error = '; '.join(cleanup)

    def _tick_motors(self, frame, now, elapsed):
        actual = frame.get('motor_positions') or []
        if (not frame.get('connected') or time.time() - frame.get('timestamp', 0) > .5 or len(actual) != 7):
            raise RuntimeError('没有新鲜有效的电机位置反馈')
        if self.motor_sent is None:
            self.motor_sent = list(actual)
        step = MOTOR_SLEW_CPS * min(elapsed, .15)
        candidate = [int(round(old + max(-step, min(step, goal - old))))
                     for old, goal in zip(self.motor_sent, self.motor_target)]
        # Contact handling: never command further than CONTACT_LEAD past the measured position.
        # A finger stopped by an object then squeezes with a bounded error (force capped by
        # GRIP_CURRENT) instead of accumulating error and tripping.
        candidate = [min(max(c, a - CONTACT_LEAD), a + CONTACT_LEAD) for c, a in zip(candidate, actual)]
        if max(abs(a - b) for a, b in zip(candidate, actual)) > MOTOR_MAX_ERROR:
            raise RuntimeError(f'电机目标与反馈相差超过{MOTOR_MAX_ERROR}计数（可能到位卡住），停止控制')
        if self.hand.setMotorTargetPosition(self.addr, self.make_positions(candidate)) != 1:
            raise RuntimeError('SDK电机目标发送失败')
        self.motor_sent = candidate
        self.last_send = now
        self.command_count += 1
        if self.releasing and max(candidate) == 0 and max(actual) < RELEASE_DONE_COUNTS:
            self.stop('已松开并失能')

    def tick(self, frame):
        if not self.active:
            return
        now = self.clock()
        try:
            if not (self.holding or self.releasing) and now - self.last_heartbeat > WATCHDOG_S:
                self.stop('心跳超时，已请求失能')
                return
            if self.holding and now - self.hold_started > HOLD_MAX_S:
                self.release()
                self.reason = f'保持超过 {HOLD_MAX_S:g} 秒，自动松开'
            if not self.hand.isMotorEnabled(self.addr) or self.hand.getFaultCode(self.addr):
                raise RuntimeError('电机使能丢失或检测到故障')
            current = frame.get('motor_current') or []
            over = [i + 1 for i, c in enumerate(current) if abs(c) > CURRENT_STOP]
            self.over_current_frames = self.over_current_frames + 1 if over else 0
            if self.over_current_frames >= CURRENT_STOP_FRAMES:
                raise RuntimeError(f'电机 {over} 电流超过 {CURRENT_STOP}（顶住或卡住），已停止')
            elapsed = now - self.last_send
            # A few milliseconds of sampling jitter must not drop a whole cycle.
            if now + min(.005, .1 / MAX_RATE_HZ) < self.next_send:
                return
            if self.motor_target is not None:      # raw mode does not need joint FK
                self._tick_motors(frame, now, elapsed)
                self._schedule_send(now)
                return
            actual = measured11(frame)
            if self.resync:
                self.sent, self.target, self.resync = actual[:], actual[:], False
            step = SLEW_DEG_S * min(elapsed, .15)
            candidate = [old + max(-step, min(step, goal-old)) for old, goal in zip(self.sent, self.target)]
            if max(abs(a-b) for a, b in zip(candidate, actual)) > MAX_TRACKING_ERROR_DEG:
                raise RuntimeError(f'目标与反馈偏差超过{MAX_TRACKING_ERROR_DEG:g}°，停止控制')
            ret, normalized = self.kin.calculateNormalizeAngle(candidate, self.hw)
            if ret != 1 or len(normalized) != 11:
                raise RuntimeError('官方真实角度→归一化转换失败')
            for i, value in enumerate(normalized):
                low, high = (-500, 500) if i == 8 else (0, 1000)
                if not math.isfinite(value) or not low <= value <= high:
                    raise RuntimeError('SDK归一化目标越界')
            if self.hand.setJointPositionsAngle(self.addr, normalized) != 1:
                raise RuntimeError('SDK关节目标发送失败')
            self.sent = candidate
            self.last_send = now
            self.command_count += 1
            self._schedule_send(now)
        except Exception as exc:
            self.error = str(exc)
            self.stop('控制错误，已请求失能')

    def _schedule_send(self, now):
        # Preserve cadence across SDK jitter instead of skipping alternate ticks.
        period = 1. / MAX_RATE_HZ
        self.next_send += max(1, math.floor((now-self.next_send)/period)+1) * period
