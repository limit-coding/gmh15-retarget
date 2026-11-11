"""Read-only GMH15 tactile monitor. No calibration or motor control interfaces."""
import json
import threading
import multiprocessing
import time
from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from pxdex.dh15 import DexH15Control, DexH15Kinematic, FingerType

states = {}
media_frames = {}
media_metadata = {}
views = {}
lock = threading.Lock()
FINGERS = [(FingerType.INDEX_FINGER, 'index', '食指', 50),
           (FingerType.MIDDLE_FINGER, 'middle', '中指', 50),
           (FingerType.RING_FINGER, 'ring', '无名指', 50),
           (FingerType.PINKY_FINGER, 'pinky', '小指', 50),
           (FingerType.THUMB_FINGER, 'thumb', '拇指', 127)]

def sample(side, port, expected_sn):
    hand = DexH15Control()
    connected = False
    sequence = 0
    started = time.monotonic()
    try:
        connected, addr = hand.openUsbDevice(port)
        if not connected:
            raise RuntimeError('USB 连接失败')
        sn = hand.getDeviceSN(addr)
        if sn != expected_sn or str(hand.getHandType(addr)) != f'HandType.{side.upper()}_HAND':
            raise RuntimeError('设备身份不匹配: ' + sn)
        hardware = hand.getHardwareVersion(addr)
        hardware_version = str(hardware.mainboard_hardware_version)
        kinematics = DexH15Kinematic()
        hand_type = hand.getHandType(addr)
        joint_retry_after = 0
        while True:
            begin = time.monotonic()
            if hand.isMotorEnabled(addr):
                raise RuntimeError('检测到电机已使能，触觉监视已停止；请先检查控制程序')
            fingers = []
            for enum, key, name, count in FINGERS:
                force, points, temp = hand.getFingerTactileData(addr, enum)
                if len(points) != count:
                    raise RuntimeError(f'{name}测点数量异常: {len(points)} / {count}')
                fingers.append({'key': key, 'name': name,
                                'force': [round(float(v) * .1, 3) for v in (force.x, force.y, force.z)],
                                'points': [[round(float(v) * .1, 3) for v in (p.x, p.y, p.z)] for p in points],
                                'temperature': round(float(temp.temperature), 1)})
            contacts = hand.getPressPositionForce(addr)
            motor_state = hand.getMotorPosition(addr)
            motors = [getattr(motor_state, f'motor{i}_pos') for i in range(1,8)]
            magnetic = list(hand.getJointMagneticEncoder(addr))
            degrees = []
            angle_ret = 0
            if time.monotonic() >= joint_retry_after:
                angle_ret, degrees = kinematics.forwardKinematicParse(hand_type, motors, magnetic, hardware_version, True)
                joint_retry_after = 0 if angle_ret == 1 else time.monotonic() + 2
            joint_error = None if angle_ret == 1 and len(degrees) == 15 else '官方编码器正解失败'
            if joint_error:
                degrees = []
            press = [{'model': p.model_name, 'xyz': [p.x, p.y, p.z],
                      'force': round(float(p.force_value) * .1, 3), 'slot': i}
                     for i, p in enumerate(contacts) if p.force_value > 0]
            sequence += 1
            frame = {'connected': True, 'error': None, 'sn': sn, 'hand': side.upper() + '_HAND',
                     'motor_enabled': False, 'timestamp': time.time(), 'sequence': sequence,
                     'read_ms': round((time.monotonic() - begin) * 1000, 1),
                     'sample_hz': round(sequence / (time.monotonic() - started), 1),
                     'unit': 'N', 'calibrated_this_session': False, 'fingers': fingers,
                     'contacts': press, 'contact_slots': len(contacts),
                     'coordinate_source': 'SDK getPressPositionForce',
                     'coordinate_unit': 'SDK native (physical unit not documented)',
                     'full_taxel_coordinate_map_available': False,
                     'joints_degrees': list(degrees), 'motor_positions': motors, 'magnetic_encoders': magnetic,
                     'joint_source': 'SDK forwardKinematicParse: measured 7 motor positions + 8 magnetic encoders',
                     'hardware_version': hardware_version, 'joint_error': joint_error}
            with lock:
                states[side] = frame
            time.sleep(max(0, .1 - (time.monotonic() - begin)))
    except Exception as exc:
        with lock:
            states[side] = {**states[side], 'connected': False, 'error': str(exc)}
        print('Tactile sampling stopped:', exc, flush=True)
    finally:
        if connected:
            hand.disconnectUsb()

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed=urlparse(self.path)
        if parsed.path == '/api/state':
            with lock:
                hands = {k: dict(v) for k,v in states.items()}
            for h in hands.values():
                h['age_ms'] = round((time.time() - h['timestamp']) * 1000) if h['timestamp'] else None
            media={k:dict(v) for k,v in media_metadata.items()}
            for m in media.values():m['age_ms']=round((time.time()-m['timestamp'])*1000)
            frame = {'hands': hands, 'media':media,'server_time': time.time()}
            body = json.dumps(frame, ensure_ascii=False, allow_nan=False).encode()
            content_type = 'application/json; charset=utf-8'
        elif parsed.path.startswith('/media/'):
            parts=parsed.path.split('/')
            if len(parts)!=4 or parts[2] not in ('camera','sim') or parts[3] not in ('left.jpg','right.jpg'):
                self.send_error(404);return
            kind=parts[2];side=parts[3].split('.')[0]
            view=parse_qs(parsed.query).get('view',['oblique'])[0]
            if kind=='sim' and view in ('front','back','oblique','side'):views[side]=view
            key=kind+'-'+side
            meta=media_metadata.get(key,{})
            if not meta.get('ok') or time.time()-meta.get('timestamp',0)>3:
                self.send_error(503,'Media not ready');return
            body=media_frames.get(key,b'')
            content_type='image/jpeg'
        elif parsed.path in ('/', '/index.html'):
            body = Path('/opt/tactile/index.html').read_bytes()
            content_type = 'text/html; charset=utf-8'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass

if __name__ == '__main__':
    manager = multiprocessing.Manager()
    states = manager.dict()
    media_frames=manager.dict()
    media_metadata=manager.dict()
    views=manager.dict()
    lock = manager.Lock()
    for side, port, sn in [('left', '/dev/ttyACM0', 'H15L49G00062'),
                           ('right', '/dev/ttyACM1', 'H15R49G00039')]:
        states[side] = {'connected': False, 'error': '正在连接设备', 'timestamp': 0, 'sequence': 0}
        multiprocessing.Process(target=sample, args=(side, port, sn), daemon=True).start()
    from live_media import camera_worker,render_worker
    for side,device in [('left','/dev/video0'),('right','/dev/video2')]:
        multiprocessing.Process(target=camera_worker,args=(side,device,media_frames,media_metadata),daemon=True).start()
        multiprocessing.Process(target=render_worker,args=(side,states,media_frames,media_metadata,views),daemon=True).start()
    ThreadingHTTPServer(('0.0.0.0', 8765), Handler).serve_forever()
