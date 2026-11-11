#!/usr/bin/env python3
"""Measure each GMH15 motor's usable range in raw counts (0-10000) and how the joints follow.

Steps one motor at a time (others held at their start position), waits for the motor to reach each
step, and records position, current, 15 joint angles and 8 magnetic encoders. A step whose current
exceeds --current-limit (default 500 of the 1000 SDK limit) is treated as the mechanical end:
the motor backs off immediately and the previous step is reported as its max. Always disables at exit.

  .venv/bin/python scripts/motor_range_probe.py               # all 7 motors
  .venv/bin/python scripts/motor_range_probe.py 2 3           # motors 2 and 3 only
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from hand_client import Hand  # noqa: E402

NAMES = {1: 'index', 2: 'middle', 3: 'ring', 4: 'little', 5: 'thumb CMC', 6: 'thumb MCP', 7: 'thumb IP'}
JOINTS = {1: (0, 1, 2), 2: (3, 4, 5), 3: (6, 7, 8), 4: (9, 10, 11), 5: (12,), 6: (13,), 7: (14,)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('motors', nargs='*', type=int, default=list(range(1, 8)))
    ap.add_argument('--step', type=int, default=250)
    ap.add_argument('--current-limit', type=int, default=500)
    ap.add_argument('--out', default=str(ROOT / 'runs/motor_range.jsonl'))
    a = ap.parse_args()
    hand = Hand('right')
    seq = [int(time.time() * 1000)]
    out = open(a.out, 'a')

    def log(**row):
        row['time'] = time.time()
        out.write(json.dumps(row, ensure_ascii=False) + '\n')
        out.flush()

    def send(counts):
        for _ in range(15):          # feedback briefly stale while the server runs arm/diagnostics
            seq[0] += 1
            r = hand.post(dict(action='motor_target', control_enabled=True, sequence=seq[0], motor_counts=counts))
            if r.get('accepted'):
                return
            if '新鲜' not in str(r.get('error')):
                break
            time.sleep(0.1)
        raise RuntimeError(f'motor_target rejected: {r}')

    def settle(m, goal, timeout=3.0):
        """Wait until motor m reaches goal; return (state, peak |current|)."""
        t0, peak = time.time(), 0
        while True:
            time.sleep(0.1)
            h, t = hand.check_active()
            cur = (h.get('motor_current') or [0] * 7)[m - 1]
            peak = max(peak, abs(cur))
            if peak > a.current_limit or abs(h['motor_positions'][m - 1] - goal) <= 60 or time.time() - t0 > timeout:
                return h, peak

    h, t = hand.state()
    if t.get('last_init_result') != 1:
        sys.exit('Position initialization required first (page button or hand_client.initialize()).')
    hand.arm()
    summary = {}
    try:
        h, _ = hand.check_active()
        base = list(h['motor_positions'])
        for m in a.motors:
            target = base[:]
            best = None
            for goal in range(a.step, 10001, a.step):
                target[m - 1] = goal
                send(target)
                h, peak = settle(m, goal)
                pos = h['motor_positions'][m - 1]
                joints = [round(h['joints_degrees'][i], 1) for i in JOINTS[m]] if h.get('joints_degrees') else None
                log(motor=m, name=NAMES[m], goal=goal, pos=pos, peak_current=peak, joints=joints,
                    encoders=h.get('magnetic_encoders'))
                print(f'{NAMES[m]:10s} goal {goal:5d} pos {pos:5d} current {peak:4d} joints {joints}', flush=True)
                if peak > a.current_limit:
                    print(f'  -> mechanical end (current {peak} > {a.current_limit}); backing off', flush=True)
                    break
                best = dict(pos=pos, joints=joints, current=peak)
            summary[NAMES[m]] = best
            target[m - 1] = base[m - 1]
            send(target)
            settle(m, base[m - 1], timeout=5.0)
        print('SUMMARY ' + json.dumps(summary, ensure_ascii=False), flush=True)
        log(summary=summary)
    finally:
        hand.stop()
        h, t = hand.state()
        print('disabled:', not h['motor_enabled'] and not t['active'], flush=True)


if __name__ == '__main__':
    main()
