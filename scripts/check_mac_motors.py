#!/usr/bin/env python3
"""Operator-requested, empty-hand basic motor check: 7 separate small moves, then disable."""
import json
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from hand_client import Hand, measured11


def main():
    out = ROOT / 'runs/mac_driver/motor_check.jsonl'
    hand = Hand('right')
    records = []
    touched = False
    def log(event, **fields):
        row = dict(time=time.time(), event=event, **fields)
        records.append(row)
        with out.open('a') as file:
            file.write(json.dumps(row, ensure_ascii=False) + '\n')
        print(json.dumps(row, ensure_ascii=False), flush=True)
    def check(h, t, active=False):
        if h.get('sn') != 'H15R49G00039' or not h.get('connected') or h.get('age_ms', 9999) > 400:
            raise RuntimeError('Identity/connection/freshness check failed')
        if h.get('joint_error') or len(h.get('joints_degrees', [])) != 15:
            raise RuntimeError('Joint feedback unavailable')
        if h.get('device_diagnostics', {}).get('faults') or h.get('device_diagnostics', {}).get('hand_status') != 1:
            raise RuntimeError('Device fault/status check failed')
        if active and (not t.get('active') or t.get('owner') != hand.owner or not h.get('motor_enabled')):
            raise RuntimeError(f'Motor control lost: {t}')
    def move(target, label, timeout=7):
        hand.target(target)
        deadline = time.monotonic() + timeout
        good = 0
        while time.monotonic() < deadline:
            time.sleep(.12)
            h, t = hand.state()
            check(h, t, active=True)
            actual = measured11(h)
            sent = t.get('sent_degrees') or []
            settled = len(sent) == 11 and max(abs(a-b) for a,b in zip(sent,target)) < .15
            error = max(abs(a-b) for a,b in zip(actual,target))
            good = good+1 if settled and error < 2.5 else 0
            if good >= 3:
                log('position', label=label, target=target, actual=actual, motors=h['motor_positions'],
                    faults=h['device_diagnostics']['faults'], max_error_deg=error)
                return h
        raise RuntimeError(f'{label}: position did not settle; actual={actual}, target={target}')
    def interrupt(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    try:
        h, t = hand.state()
        check(h, t)
        if t.get('active') or h.get('motor_enabled'):
            raise RuntimeError('Another motor session is active; refusing takeover')
        log('precheck', sn=h['sn'], motor_enabled=h['motor_enabled'], faults=h['device_diagnostics']['faults'])
        touched = True
        diagnostics = hand.initialize()
        log('initialized', diagnostics=diagnostics)
        # Initialization may change the physical zero. Wait for a new frame before arming.
        time.sleep(.5)
        h, t = hand.state()
        check(h, t)
        baseline = measured11(h)
        if any(not 0 <= baseline[i] <= (45 if i < 8 else 30) for i in (0,2,4,6,8,9,10)):
            raise RuntimeError(f'Unexpected initialized position: {baseline}')
        hand.arm()
        time.sleep(.3)
        h, t = hand.state()
        check(h, t, active=True)
        log('armed', baseline=baseline, motors=h['motor_positions'])
        joints = [(1,0,'index'),(2,2,'middle'),(3,4,'ring'),(4,6,'little'),
                  (5,8,'thumb CMC'),(6,9,'thumb MCP'),(7,10,'thumb IP')]
        only = [a for a in sys.argv[1:]]  # e.g. ring little "thumb CMC"
        for motor, cmd_index, name in [j for j in joints if not only or j[2] in only]:
            before, _ = hand.state()
            target = baseline[:]
            target[cmd_index] += 10.
            after = move(target, name + ' +10deg')
            displacement = measured11(after)[cmd_index] - measured11(before)[cmd_index]
            counts = after['motor_positions'][motor-1] - before['motor_positions'][motor-1]
            if displacement < 6 or abs(counts) < 20:
                raise RuntimeError(f'{name}: insufficient measured motor/angle movement')
            returned = move(baseline, name + ' return')
            log('motor_pass', motor=motor, joint=name, measured_travel_deg=displacement, motor_travel_counts=counts,
                return_error_deg=abs(measured11(returned)[cmd_index] - baseline[cmd_index]))
        log('all_motors_pass' if not only else 'selected_motors_pass', count=7 if not only else len(only), joints=only or None)
    finally:
        if touched:
            try:
                hand.stop()
            finally:
                h, t = hand.state()
                disabled = not h.get('motor_enabled') and not t.get('active')
                log('stopped', motor_enabled=h.get('motor_enabled'), active=t.get('active'),
                    faults=h.get('device_diagnostics', {}).get('faults'), disabled_confirmed=disabled)
                if not disabled:
                    raise RuntimeError('Motor disable not confirmed')


if __name__ == '__main__':
    main()
