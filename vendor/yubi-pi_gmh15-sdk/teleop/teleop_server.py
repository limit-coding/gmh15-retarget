"""Local browser UI plus opt-in hardware commands. --demo never imports the SDK."""
import argparse
import copy
import io
import json
import multiprocessing as mp
import queue
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs
from teleop_control import angles11, command11, EDITABLE, LIMITS, JOINT_NAMES, PASSIVE

B=Path(__file__).resolve().parent


def demo_worker(states, statuses):
    fixture=json.loads((B/'interface-current-sample.json').read_text(encoding='utf-8'))['hands']
    sequence=0
    while True:
        sequence+=1
        for side in ('left','right'):
            state=copy.deepcopy(fixture[side]);state.update(timestamp=time.time(),sequence=sequence,demo=True,
                motor_enabled=False,connected=True,sample_hz=10.,error=None)
            states[side]=state
            statuses[side]=dict(active=False,owner=None,error=None,reason='离线演示；实机使能不可用',command_count=0)
        time.sleep(.1)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--demo',action='store_true')
    ap.add_argument('--port',type=int,default=8765)
    ap.add_argument('--bind',default='127.0.0.1')
    ap.add_argument('--models',default='/opt/models')
    ap.add_argument('--no-render',action='store_true',help='HTTP/unit testing only')
    ap.add_argument('--allow-motion',action='store_true',help='Explicit operator opt-in; default hardware SDK is read-only')
    ap.add_argument('--hands',nargs='+',choices=('left','right'),default=['left','right'])
    args=ap.parse_args()
    manager=mp.Manager()
    states=manager.dict();statuses=manager.dict();targets=manager.dict()
    frames=manager.dict();metadata=manager.dict();views=manager.dict();heartbeats=manager.dict()
    commands={s:mp.Queue(maxsize=16) for s in ('left','right')}
    stops={s:mp.Event() for s in ('left','right')}
    csrf=secrets.token_urlsafe(32)
    arm_lock=threading.Lock()
    reservation={'side':None,'request_id':None}
    processes=[]
    def start(fn,*params):
        p=mp.Process(target=fn,args=params,daemon=True);p.start();processes.append(p)
    if args.demo:
        start(demo_worker,states,statuses)
        from PIL import Image,ImageDraw
        im=Image.new('RGB',(640,480),'#17232d');draw=ImageDraw.Draw(im)
        draw.text((120,220),'OFFLINE DEMO / NO LIVE CAMERA',fill='#7edec8')
        buf=io.BytesIO();im.save(buf,format='JPEG')
        for side in ('left','right'):
            frames['camera-'+side]=buf.getvalue()
            metadata['camera-'+side]=dict(ok=True,timestamp=time.time(),width=640,height=480,fps=0,demo=True)
    else:
        from teleop_acquisition import hardware_worker
        from live_media import camera_worker,render_worker
        for side,port,sn,cam in [('left','/dev/ttyACM0','H15L49G00062','/dev/video0'),
                                 ('right','/dev/ttyACM1','H15R49G00039','/dev/video2')]:
            if side not in args.hands:continue
            states[side]=dict(connected=False,timestamp=0,error='正在连接设备')
            start(hardware_worker,side,port,sn,states,statuses,targets,heartbeats,commands[side],stops[side],not args.allow_motion)
            start(camera_worker,side,cam,frames,metadata)
            if not args.no_render:start(render_worker,side,states,frames,metadata,views)
    if not args.no_render:
        from teleop_media import target_worker
        for side in args.hands:start(target_worker,side,states,targets,frames,metadata,views,args.models)
        if args.demo:
            for side in ('left','right'):start(target_worker,side,states,targets,frames,metadata,views,args.models,'sim')

    class Handler(BaseHTTPRequestHandler):
        def send(self,body,status=200,kind='application/json; charset=utf-8'):
            self.send_response(status);self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Length',str(len(body)));self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def result(self,value,status=200):self.send(json.dumps(value,ensure_ascii=False,allow_nan=False).encode(),status)
        def host_ok(self):
            host=urlparse('http://'+self.headers.get('Host','')).hostname
            return host in ('localhost','127.0.0.1','::1')
        def do_GET(self):
            if not self.host_ok():return self.result({'error':'Local host required'},403)
            parsed=urlparse(self.path)
            if parsed.path=='/api/teleop/config':
                return self.result(dict(token=csrf,demo=args.demo,hands=args.hands,motion_locked=not args.allow_motion,interaction='buttons',supports_initialize=True,limits_degrees=LIMITS,joint_names=JOINT_NAMES,
                    adjustable_indices_15=EDITABLE,passive_indices=PASSIVE,watchdog_ms=600,max_send_hz=10,slew_deg_s=8))
            if parsed.path=='/api/state':
                hands={k:dict(v) for k,v in states.items()}
                media={k:dict(v) for k,v in metadata.items()}
                for h in hands.values():h['age_ms']=round((time.time()-h.get('timestamp',0))*1000)
                for m in media.values():m['age_ms']=round((time.time()-m.get('timestamp',0))*1000)
                return self.result(dict(hands=hands,media=media,teleop=dict(statuses),demo=args.demo,server_time=time.time()))
            if parsed.path.startswith('/media/'):
                parts=parsed.path.split('/')
                if len(parts)!=4 or parts[2] not in ('sim','camera','target') or parts[3] not in ('left.jpg','right.jpg'):
                    return self.result({'error':'Not found'},404)
                side=parts[3].split('.')[0];kind=parts[2]
                view=parse_qs(parsed.query).get('view',['oblique'])[0]
                if view in ('front','back','oblique','side'):views[side]=view
                key=kind+'-'+side
                meta=metadata.get(key,{})
                if not meta.get('ok') or (not args.demo and time.time()-meta.get('timestamp',0)>3):
                    return self.result({'error':meta.get('error','Media not ready')},503)
                return self.send(frames.get(key,b''),kind='image/jpeg')
            assets={'/teleop.js':('teleop_buttons.js','application/javascript; charset=utf-8'),
                    '/teleop.css':('teleop.css','text/css; charset=utf-8')}
            if parsed.path in assets:
                file,kind=assets[parsed.path];return self.send((B/file).read_bytes(),kind=kind)
            if parsed.path in ('/','/index.html'):
                html=(B/'tactile-dual.html').read_text(encoding='utf-8')
                html=html.replace('</head>','<link rel="stylesheet" href="/teleop.css"></head>')
                html=html.replace('</body>','<script src="/teleop.js"></script></body>')
                html=html.replace('未标定、未使能电机；暂停仅冻结画面。双手采集占用各自串口，执行动作前需停止采集容器。',
                    '触觉未重新标定。点击启用后才发送硬件目标；停止按钮、失焦、断线或暂停画面会停止遥操作。')
                html=html.replace("h?.motor_enabled===false?'未使能':'未知'", "h?.motor_enabled===true?'已使能':h?.motor_enabled===false?'未使能':'未知'")
                html=html.replace("valid?'在线 · 只读'", "valid?(h.demo?'样本 · 非实机':h.motor_enabled?'在线 · 电机已使能':'在线 · 未使能')")
                html=html.replace("(all?'双手实时连接':'部分设备离线')", "(frame.demo?'离线样本演示':all?'双手实时连接':'部分设备离线')")
                if args.demo:
                    html=html.replace('掌内相机 · 真机','相机 · 离线占位').replace('参考零位 · 无有效关节','离线样本姿态')
                if len(args.hands)==1:
                    label='右手' if args.hands[0]=='right' else '左手'
                    html=html.replace("sides=['left','right']",'sides='+json.dumps(args.hands))
                    html=html.replace('GMH15 双手','GMH15 '+label).replace('双手实时连接',label+'实时连接')
                    html=html.replace('</head>','<style>.hands{grid-template-columns:minmax(0,1fr)}main{max-width:1000px}</style></head>')
                return self.send(html.encode(),kind='text/html; charset=utf-8')
            return self.result({'error':'Not found'},404)
        def do_POST(self):
            if urlparse(self.path).path!='/api/teleop':return self.result({'error':'Not found'},404)
            if (not self.host_ok() or self.headers.get('Origin')!='http://'+self.headers.get('Host','') or
                    not secrets.compare_digest(self.headers.get('X-Teleop-Token',''),csrf)):
                return self.result({'error':'Origin/token rejected'},403)
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=8192:raise ValueError('请求长度异常')
                request=json.loads(self.rfile.read(length))
                action=request.get('action');side=request.get('side');owner=request.get('owner')
                if side not in args.hands or not isinstance(owner,str) or not 16<=len(owner)<=96:
                    raise ValueError('手型或会话参数无效')
                status=statuses.get(side,{})
                if action in ('stop','release'):
                    stops[side].set();heartbeats[side]={}
                    with arm_lock:
                        if reservation['side']==side:reservation['side']=None
                    return self.result({'accepted':True,'note':'已请求串口所有者失能；请看状态确认'})
                if action=='preview':
                    target=command11(request.get('angles_degrees'))
                    if status.get('active') and status.get('owner')!=owner:raise ValueError('另一会话正在控制')
                    targets[side]=dict(angles_degrees=target,timestamp=time.time())
                    return self.result({'accepted':True,'preview_only':True})
                if action not in ('initialize','arm','heartbeat','target'):raise ValueError('未知动作')
                if args.demo:return self.result({'error':'离线演示不连接设备，不能启用实机'},409)
                if not args.allow_motion:return self.result({'error':'实机运动已由服务端锁定，本次只读取反馈和预览目标'},423)
                if action=='initialize':
                    if request.get('confirm_initialization') is not True:raise ValueError('必须明确点击位置初始化')
                elif request.get('control_enabled') is not True:raise ValueError('必须明确点击启用控制')
                if action in ('heartbeat','target') and (not status.get('active') or status.get('owner')!=owner):
                    raise ValueError('没有当前手控制权')
                if action in ('arm','initialize') and any(s.get('active') or s.get('initializing') for s in statuses.values()):
                    raise ValueError('请先停止当前实机手；本入口一次只控制一只手')
                if action in ('arm','initialize'):
                    with arm_lock:
                        old=statuses.get(reservation['side'],{}) if reservation['side'] else {}
                        if old.get('last_request_id')==reservation['request_id'] and not old.get('active') and not old.get('initializing'):
                            reservation['side']=None
                        if reservation['side'] is not None:raise ValueError('已有待确认的启用请求；请先点击停止')
                        reservation['side']=side
                        request['request_id']=secrets.token_hex(12);reservation['request_id']=request['request_id']
                if action=='target':
                    request['angles_degrees']=command11(request.get('angles_degrees'))
                    sequence=request.get('sequence')
                    if isinstance(sequence,bool) or not isinstance(sequence,int) or not 0<=sequence<=2**53-1:
                        raise ValueError('目标序号无效')
                if action in ('arm','heartbeat'):heartbeats[side]=dict(owner=owner,time=time.monotonic())
                request['queued_at']=time.monotonic()
                if action!='heartbeat':commands[side].put_nowait(request)
                return self.result({'accepted':True,'hardware_confirmed':False,'request_id':request.get('request_id')})
            except queue.Full:return self.result({'error':'命令队列已满'},429)
            except (ValueError,TypeError,json.JSONDecodeError) as exc:return self.result({'error':str(exc)},400)
        def log_message(self,*_):pass
    server=ThreadingHTTPServer((args.bind,args.port),Handler)
    mode='OFFLINE DEMO' if args.demo else ('HARDWARE / DISARMED' if args.allow_motion else 'HARDWARE / MOTION LOCKED')
    print(f'GMH15 {mode}: http://127.0.0.1:{args.port}/',flush=True)
    import signal
    def terminate(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        for event in stops.values():event.set()
        time.sleep(2.)
        for p in processes:p.terminate();p.join(timeout=1)
        server.server_close()

if __name__=='__main__':main()
