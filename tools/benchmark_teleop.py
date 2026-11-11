"""Small empty-hand sine benchmark through the existing guarded HTTP controller."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bare_hand_teleop.drivers import GMH15HttpDriver
from hand_client import Hand


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=10)
    ap.add_argument('--seconds', type=float, default=12)
    ap.add_argument('--legacy-driver', help='Optional backed-up drivers.py for a like-for-like baseline')
    a = ap.parse_args()
    driver_class = GMH15HttpDriver
    if a.legacy_driver:
        import importlib.util
        spec = importlib.util.spec_from_file_location('bare_hand_teleop.legacy_drivers', a.legacy_driver)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        driver_class = module.GMH15HttpDriver
    h = Hand()
    state, status = h.state()
    if state.get('motor_enabled') or status.get('active'):
        raise RuntimeError('Another control session is active')
    if status.get('last_init_result') != 1:
        print('initialize:', h.initialize(), flush=True)
        time.sleep(.5)
    d = driver_class(hardware_enabled=True, motor_mode=True, send_hz=a.hz)
    d.connect()
    rows = []
    try:
        d.arm()
        start = time.monotonic()
        next_poll = 0
        while (elapsed := time.monotonic() - start) < a.seconds + 2:
            x = 1000 * (1 - np.cos(2 * np.pi * elapsed / 4)) if elapsed < a.seconds else 0
            target = [x] * 4 + [x * .4, x * .6, x * .6]
            d.submit(target)
            if elapsed >= next_poll:
                st, ctl = h.state()
                rows.append(dict(t=elapsed, host_monotonic=time.monotonic(), target=target, hand=st, control=ctl))
                next_poll = elapsed + .1
                if not d.armed or ctl.get('error'):
                    raise RuntimeError(f'Control fault: {d.fault}, {ctl}')
            time.sleep(max(0, .01 - (time.monotonic()-start-elapsed)))
    finally:
        d.close()
        for _ in range(30):
            time.sleep(.1)
            st, ctl = h.state()
            if not st.get('motor_enabled') and not ctl.get('active'):
                break
        Path(a.out).write_text(''.join(json.dumps(r) + '\n' for r in rows))
        if st.get('motor_enabled') or ctl.get('active'):
            raise RuntimeError('Motor disable not confirmed')
        print('disabled confirmed; saved', len(rows), 'rows', flush=True)
    frames = {r['hand']['sequence']: r for r in rows if r['t'] > 1}
    times = np.array([r['hand']['timestamp'] for r in frames.values()])
    sends = {}
    for r in rows:
        if r['t'] > 1:
            sends.setdefault(r['control']['command_count'], r['t'])
    intervals = np.diff(list(sends.values())) * 1000
    valid = [r for r in rows if r['t'] > 1]
    summary = dict(feedback_hz=(valid[-1]['hand']['sequence']-valid[0]['hand']['sequence'])/(times[-1]-times[0]),
                   command_hz=(valid[-1]['control']['command_count']-valid[0]['control']['command_count'])/(valid[-1]['t']-valid[0]['t']),
                   interval_note='HTTP-observed intervals only; use USB trace for exact command intervals',
                   command_interval_ms_p50_p95_max=np.percentile(intervals, [50,95,100]).tolist(),
                   read_ms_p50_p95=np.percentile([r['hand']['read_ms'] for r in frames.values()], [50,95]).tolist(),
                   peak_current=max(abs(c) for r in rows for c in r['hand'].get('motor_current', [])),
                   faults=[r['control']['error'] for r in rows if r['control'].get('error')])
    Path(a.out + '.summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
