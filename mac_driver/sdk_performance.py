"""Empty-hand SDK comparison. Original vendor files are mounted read-only.

The position example's preparation function, values, commands and holds are reused;
only its Modbus connection is replaced by the vendor USB connection for this Mac.
"""
import argparse
import json
from pathlib import Path
import runpy
import signal
import sys
import time

from pxdex.dh15 import DexH15Control, DexH15Kinematic, Dex15MotorPosition, Dex15MotorCurrent


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--allow-motion', action='store_true')
    ap.add_argument('--hz', type=float, default=10)
    ap.add_argument('--official-only', action='store_true')
    ap.add_argument('--out', default='sdk')
    ap.add_argument('--usb-example', action='store_true', help='Run original open_usb_device.py with supervised cleanup')
    ap.add_argument('--disable-only', action='store_true', help='Recovery: disable this known hand without moving it')
    ap.add_argument('--gesture', type=int, choices=range(1, 9), help='Official gesture_demo.py gesture ID')
    a = ap.parse_args()
    if not a.allow_motion:
        ap.error('Explicit --allow-motion required: initializes and moves the empty hand')
    def interrupt(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    sys.path.insert(0, '/opt/examples')
    import position_control_demo as demo
    hand = DexH15Control()
    ok, addr = hand.openUsbDevice('/tmp/gmh15-serial')
    if not ok:
        raise RuntimeError('USB connection failed')
    rows = []
    old_current = None
    old_speed = None
    owned = False
    def log(event, **fields):
        row = dict(event=event, t=time.monotonic(), **fields)
        rows.append(row)
        if event != 'sample':
            print(json.dumps(row), flush=True)
    def timed(name, *args):
        start = time.monotonic()
        result = getattr(hand, name)(addr, *args)
        elapsed = (time.monotonic() - start) * 1000
        return result, elapsed
    def sample(phase):
        p, pm = timed('getMotorPosition')
        (ok, current), cm = timed('getMotorActualCurrent')
        _, em = timed('isMotorEnabled')
        faults, fm = timed('getFaultCode')
        _, mm = timed('getJointMagneticEncoder')
        currents = [getattr(current, f'motor{i}_current') for i in range(1, 8)] if ok == 1 else []
        log('sample', phase=phase, positions=[getattr(p, f'motor{i}_pos') for i in range(1, 8)],
            currents=currents, ms=dict(position=pm, current=cm, enabled=em, faults=fm, magnetic=mm))
        if ok != 1 or max(map(abs, currents)) > 600 or faults:
            raise RuntimeError(f'Current/fault check failed: {currents}, {faults}')
    def observe(seconds, phase):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            start = time.monotonic()
            sample(phase)
            time.sleep(max(0, .02 - (time.monotonic()-start)))
    try:
        sn = hand.getDeviceSN(addr)
        if sn != 'H15R49G00039' or (hand.isMotorEnabled(addr) and not a.disable_only):
            raise RuntimeError('Identity mismatch or another motor session active')
        owned = True
        log('connected', sn=sn)
        if a.disable_only:
            return
        if a.usb_example:
            # Execute the original file; reuse our checked connection and defer
            # its disconnect until the finally block confirms motor disable.
            import pxdex.dh15 as sdk
            class SupervisedUSB:
                def __getattr__(self, name):
                    return getattr(hand, name)
                def openUsbDevice(self, port):
                    return True, addr
                def disconnectUsb(self):
                    return True
            original = sdk.DexH15Control
            argv = sys.argv
            try:
                hand.clearFaultCode(addr)
                sdk.DexH15Control = lambda: SupervisedUSB()
                sys.argv = ['open_usb_device.py', '--port', '/tmp/gmh15-serial']
                runpy.run_path('/opt/examples/open_usb_device.py', run_name='__main__')
                log('official_usb_example_completed')
            finally:
                sdk.DexH15Control = original
                sys.argv = argv
            return
        if not a.gesture:
            observe(3, 'idle')
        # Same documented fault-recovery preflight used by the working Mac driver.
        if not hand.clearFaultCode(addr):
            raise RuntimeError('clearFaultCode failed')
        old_current = hand.getMotorMaxCurrentConfig(addr)
        if a.gesture:
            old_speed = hand.getMotorMaxVelocityConfig(addr)
        cap = Dex15MotorCurrent()
        for i in range(1, 8):
            setattr(cap, f'motor{i}_current', 450)
        if not hand.setMotorMaxCurrent(addr, cap):
            raise RuntimeError('Current cap failed')
        if a.gesture:
            import gesture_demo as gesture
            prepare = gesture.prepare_position_mode
        else:
            prepare = demo.prepare_position_mode
        if not prepare(hand, addr):
            raise RuntimeError('Official position-mode preparation failed')
        if not hand.setMotorMaxCurrent(addr, cap):
            raise RuntimeError('Current cap failed after init')
        if a.gesture:
            from types import SimpleNamespace
            from gesture_adapter import BoundedGestureHand
            def gesture_sleep(seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    sample('gesture')
                    time.sleep(max(0, min(.1, deadline-time.monotonic())))
            gesture.time = SimpleNamespace(sleep=gesture_sleep)
            hw = str(hand.getHardwareVersion(addr).mainboard_hardware_version)
            proxy = BoundedGestureHand(hand, DexH15Kinematic(), hand.getHandType(addr), hw,
                                       demo.motor_position, log)
            log('gesture_start', gesture=a.gesture)
            if not gesture.execute_gesture(proxy, addr, a.gesture):
                raise RuntimeError('官方手势执行失败')
            log('gesture_completed', gesture=a.gesture)
            return
        log('official_position_demo_start', usb_adapter=True, protective_current_cap=450)
        for label, values, duration in [('official_out', (2000,2000,2000,2000,800,1200,1200), 1),
                                        ('official_return', (0,)*7, .5)]:
            result, ms = timed('setMotorTargetPosition', demo.motor_position(values))
            log('official_target', phase=label, values=values, result=result, ms=ms)
            if result != 1:
                raise RuntimeError('Official target rejected')
            # Preserve the official example's quiet holds (no extra SDK queries).
            time.sleep(duration)
            sample(label)
        # Identical smooth input at several requested rates, directly through SDK.
        import math
        for hz in (() if a.official_only else (a.hz,)):
            start = time.monotonic()
            deadline = start
            while (elapsed := time.monotonic()-start) < 8:
                x = 1000*(1-math.cos(2*math.pi*elapsed/4))
                values = [round(x)]*4 + [round(x*.4),round(x*.6),round(x*.6)]
                result, ms = timed('setMotorTargetPosition', demo.motor_position(values))
                log('target', hz=hz, values=values, result=result, ms=ms)
                if result != 1:
                    raise RuntimeError('SDK target rejected')
                sample(f'direct_{hz}')
                deadline += 1/hz
                time.sleep(max(0, deadline-time.monotonic()))
                deadline = max(deadline, time.monotonic()-1/hz)
        hand.setMotorTargetPosition(addr, Dex15MotorPosition())
        observe(1, 'return')
    finally:
        # Stop requests must not interrupt the disable/restore handshake itself.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if owned:
                hand.disableMotor(addr)
                if old_speed is not None:
                    if not hand.setMotorMaxVelocity(addr, old_speed):
                        raise RuntimeError('原速度配置恢复失败')
                if old_current is not None:
                    hand.setMotorMaxCurrent(addr, old_current)
                confirmed = not hand.isMotorEnabled(addr)
                log('disabled', confirmed=confirmed)
                if not confirmed:
                    raise RuntimeError('电机失能未确认，请检查实物')
        finally:
            hand.disconnectUsb()
            Path('/out/' + a.out + '.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('例程已停止。', flush=True)
