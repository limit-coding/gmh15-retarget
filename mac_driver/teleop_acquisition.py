"""Single serial owner per hand: feedback sampling and explicitly armed commands."""
import queue
import time
from pxdex.dh15 import (DexH15Control, DexH15Kinematic, ControlMode,
                         Dex15MotorPosition, Dex15MotorSpeed, Dex15MotorCurrent)
from tactile_server import FINGERS
from teleop_control import Controller,ReadOnlyHandProxy,AuditedHandProxy,MAX_RATE_HZ,MOTOR_SPEED,GRIP_CURRENT


def hardware_worker(side, port, expected_sn, states, statuses, targets, heartbeats, commands, stop_event, motion_locked=True, motor_only=False):
    hand = DexH15Control()
    hand=ReadOnlyHandProxy(hand) if motion_locked else AuditedHandProxy(hand)
    connected = False
    controller = None
    sequence = 0
    started = time.monotonic()
    joint_retry_after=0
    last_request_id=None
    try:
        connected, addr = hand.openUsbDevice(port)
        if not connected:
            raise RuntimeError('USB连接失败')
        sn = hand.getDeviceSN(addr)
        hand_type = hand.getHandType(addr)
        if sn != expected_sn or str(hand_type) != f'HandType.{side.upper()}_HAND':
            raise RuntimeError('设备身份不匹配: ' + sn)
        hw = str(hand.getHardwareVersion(addr).mainboard_hardware_version)
        fw=hand.getFirmwareVersion(addr)
        firmware_version=str(getattr(fw,'unified_version',fw))
        device_diagnostics={};next_diagnostics=0
        kin = DexH15Kinematic()
        def make_positions(values):
            result = Dex15MotorPosition()
            for i, value in enumerate(values, 1):
                setattr(result, f'motor{i}_pos', int(value))
            return result
        def make_speeds(value):
            result = Dex15MotorSpeed()
            for i in range(1, 8):
                setattr(result, f'motor{i}_speed', value)
            return result
        def make_currents(value):
            result = Dex15MotorCurrent()
            for i in range(1, 8):
                setattr(result, f'motor{i}_current', value)
            return result
        controller = Controller(hand, addr, kin, hand_type, hw, ControlMode.POSITION_CONTROL_MODE,
                                make_positions, make_speeds, make_currents=make_currents)
        sample_hz = max(30., MAX_RATE_HZ) if motor_only and MAX_RATE_HZ > 10 else 10.
        next_sample = time.monotonic()
        while True:
            begin = time.monotonic()
            # Priority stop is independent of the bounded target/arm queue.
            if stop_event.is_set():
                stop_event.clear()
                controller.stop('用户停止或页面离开')
                while True:
                    try: commands.get_nowait()
                    except queue.Empty: break
            heartbeat = heartbeats.get(side, {})
            if (controller.active and heartbeat.get('owner') == controller.owner and
                    begin - heartbeat.get('time', 0) <= .3):
                controller.heartbeat(heartbeat['owner'])
            if (controller.active and not (controller.holding or controller.releasing)
                    and begin-controller.last_heartbeat > .6):
                controller.stop('心跳超时，已请求失能')
            enabled = bool(hand.isMotorEnabled(addr))
            if begin>=next_diagnostics:
                device_diagnostics={'firmware_version':firmware_version,
                    'hand_status':hand.checkHandStatus(addr),'control_mode':str(hand.getMotorControlMode(addr)),
                    'faults':[{'motor':getattr(f,'motor_ID',None),'finger':getattr(f,'finger_name',None),
                               'error':str(f.error_code)} for f in hand.getFaultCode(addr)]}
                if motor_only and not controller.active:
                    speed = hand.getMotorMaxVelocityConfig(addr)
                    current_limit = hand.getMotorMaxCurrentConfig(addr)
                    device_diagnostics.update(
                        max_velocity=[getattr(speed,f'motor{i}_speed') for i in range(1,8)],
                        max_current=[getattr(current_limit,f'motor{i}_current') for i in range(1,8)],
                        config_source='SDK read while disarmed')
                elif motor_only:
                    # Configuration reads are not part of motion feedback. Avoid
                    # querying them while moving; show the values applied by arm().
                    device_diagnostics.update(max_velocity=[MOTOR_SPEED]*7,
                        max_current=[GRIP_CURRENT]*7, config_source='session applied configuration')
                next_diagnostics=begin+1
            if enabled and not controller.active:
                raise RuntimeError('检测到未由本会话启用的电机，拒绝接管')
            fingers = []
            for enum, key, name, count in (() if motor_only else FINGERS):
                force, points, temp = hand.getFingerTactileData(addr, enum)
                if len(points) != count: raise RuntimeError('触觉点数异常')
                fingers.append(dict(key=key, name=name,
                    force=[round(float(v)*.1, 3) for v in (force.x, force.y, force.z)],
                    points=[[round(float(v)*.1, 3) for v in (p.x, p.y, p.z)] for p in points],
                    temperature=round(float(temp.temperature), 1)))
            contacts = [] if motor_only else hand.getPressPositionForce(addr)
            positions = hand.getMotorPosition(addr)
            motors = [getattr(positions, f'motor{i}_pos') for i in range(1, 8)]
            magnetic = list(hand.getJointMagneticEncoder(addr))
            ret,degrees=0,[]
            if time.monotonic()>=joint_retry_after:
                ret, degrees = kin.forwardKinematicParse(hand_type, motors, magnetic, hw, True)
                joint_retry_after=0 if ret==1 else time.monotonic()+2
            valid = ret == 1 and len(degrees) == 15
            sequence += 1
            frame = dict(connected=True, error=None, sn=sn, hand=side.upper()+'_HAND',
                motor_only=motor_only,
                motor_enabled=enabled, timestamp=time.time(), sequence=sequence,
                read_ms=round((time.monotonic()-begin)*1000, 1),
                sample_hz=round(sequence/(time.monotonic()-started), 1), unit='N',
                configured_sample_hz=sample_hz,
                calibrated_this_session=False, fingers=fingers,
                contacts=[dict(model=p.model_name, xyz=[p.x,p.y,p.z], force=round(float(p.force_value)*.1,3), slot=i)
                          for i,p in enumerate(contacts) if p.force_value > 0],
                contact_slots=len(contacts), coordinate_source='SDK getPressPositionForce',
                coordinate_unit='SDK native (physical unit not documented)', full_taxel_coordinate_map_available=False,
                joints_degrees=list(degrees) if valid else [], motor_positions=motors,
                magnetic_encoders=magnetic, hardware_version=hw,
                joint_error=None if valid else ('官方编码器正解失败（7路电机反馈均为0，未执行初始化）' if all(v==0 for v in motors) else '官方编码器正解失败'),
                joint_source='SDK forwardKinematicParse: measured motors + encoders')
            frame['device_diagnostics']=device_diagnostics
            if motor_only:            # every frame: raw motor page shows load / end stops live
                cur_ret, cur = hand.getMotorActualCurrent(addr)
                frame['motor_current'] = [getattr(cur, f'motor{i}_current') for i in range(1, 8)] if cur_ret == 1 else None
            # Drain a bounded batch; target requests remain separate from display preview targets.
            for _ in range(16):
                try: command = commands.get_nowait()
                except queue.Empty: break
                try:
                    if motion_locked:raise ValueError('实机控制已锁定')
                    if time.monotonic()-command.get('queued_at',0)>1:raise ValueError('操作请求已过期')
                    if command['action']=='initialize':
                        if command.get('confirm_initialization') is not True:raise ValueError('缺少初始化确认')
                        statuses[side]={**controller.status(),'initializing':True,'owner':command['owner'],
                                        'reason':'正在执行位置初始化（可能运动）','last_request_id':last_request_id}
                        controller.initialize(command['owner']);joint_retry_after=0
                    elif command['action'] == 'arm':
                        hb = heartbeats.get(side, {})
                        if hb.get('owner') != command['owner'] or time.monotonic()-hb.get('time',0) > .4:
                            raise ValueError('启用请求已过期')
                        controller.arm(command['owner'], frame)
                        if stop_event.is_set():controller.stop('启用期间收到停止请求')
                        targets[side] = dict(angles_degrees=controller.target, timestamp=time.time())
                    elif command['action'] == 'target':
                        controller.set_target(command['owner'], command['sequence'], command['angles_degrees'])
                    elif command['action'] == 'motor_target':
                        controller.set_motor_target(command['owner'], command['sequence'], command['motor_counts'])
                    elif command['action'] == 'hold':
                        controller.hold(command['owner'])
                    elif command['action'] == 'release':
                        controller.release()
                except Exception as exc:
                    if controller.active and command.get('owner') == controller.owner:
                        controller.error = str(exc); controller.stop('无效控制目标，已请求失能')
                    elif not controller.active:
                        controller.error = str(exc)
                finally:
                    # Only arm/initialize carry request ids; other commands must not clear it,
                    # or the server's pending-arm reservation can never be matched and released.
                    if command.get('request_id'):last_request_id=command['request_id']
            controller.tick(frame)
            status=controller.status()
            status.update(motion_locked=motion_locked,sdk_write_calls=0 if motion_locked else sum(hand.write_counts.values()),
                          sdk_write_counts={} if motion_locked else dict(hand.write_counts),last_request_id=last_request_id,
                          blocked_sdk_write_attempts=hand.blocked_write_attempts if motion_locked else None)
            if motion_locked and not status['error']:
                status['reason']='真实设备在线；运动锁定，仅预览目标' if valid else '真实触觉与设备在线；关节角未就绪，运动锁定'
            statuses[side] = status
            frame['motor_enabled'] = bool(hand.isMotorEnabled(addr))
            states[side] = frame
            next_sample += 1. / sample_hz
            remaining = next_sample - time.monotonic()
            if remaining < 0:  # skip missed periods after initialization or SDK timeouts
                next_sample = time.monotonic()
            else:
                time.sleep(remaining)
    except Exception as exc:
        states[side] = {**states.get(side, {}), 'connected':False, 'error':str(exc)}
        print(side, 'sampling:', str(exc), flush=True)
    finally:
        if controller:
            controller.stop('采集进程退出'); statuses[side] = controller.status()
        if connected: hand.disconnectUsb()
