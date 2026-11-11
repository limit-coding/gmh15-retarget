"""Measure capture, hand recognition and retarget speed without any motor connection."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from bare_hand_teleop.tracking import MediaPipeTracker,select_hand
from bare_hand_teleop.opt_retarget import OptConfig,load_calib
from bare_hand_teleop.sim_pipeline import SimPipeline
from bare_hand_teleop.safety import GateConfig,GuardConfig
from bare_hand_teleop.filters import MotionShaper


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--camera',type=int,default=1)
    ap.add_argument('--seconds',type=float,default=8)
    ap.add_argument('--out',default=str(ROOT/'runs/camera-fps-20261107.json'))
    a=ap.parse_args()
    cap=cv2.VideoCapture(a.camera)
    tracker=None
    results={}
    try:
        if not cap.isOpened():
            raise RuntimeError('摄像头未打开，请检查相机权限或占用情况')
        results['camera']=dict(index=a.camera,backend=cap.getBackendName(),
            width=cap.get(cv2.CAP_PROP_FRAME_WIDTH),height=cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            reported_fps=cap.get(cv2.CAP_PROP_FPS))
        print(json.dumps(results['camera']),flush=True)
        for _ in range(10):cap.read()
        for mode in ('capture','recognition','retarget'):
            if mode=='recognition':
                tracker=MediaPipeTracker(ROOT/'models/hand_landmarker.task',num_hands=2,mirrored=True)
            if mode=='retarget':
                opt=OptConfig(path='measured')
                load_calib(opt,ROOT/'config/sim_calib_right.json')
                pipe=SimPipeline('right',opt=opt,gate=GateConfig(expected_handedness='Right'),
                    mirror_world=True,one_euro=(2.,1.),
                    shaper=MotionShaper(v_max=360.,a_max=3000.,tau=.05),
                    guard=GuardConfig(max_rate_deg_s=720.))
            print('开始测量：'+mode,flush=True)
            rows=[];start=time.monotonic()
            while time.monotonic()-start<a.seconds:
                begin=time.monotonic();ok,frame=cap.read();after_read=time.monotonic()
                if not ok:raise RuntimeError('摄像头读取失败')
                detected=False;tracking=solve=0
                if tracker:
                    observations=tracker.process(cv2.flip(frame,1),after_read)
                    tracking=time.monotonic()-after_read
                    detected=bool(observations)
                    if mode=='retarget':
                        obs=select_hand(observations,after_read,'Right')
                        solve_start=time.monotonic();pipe.step(obs,after_read)
                        solve=time.monotonic()-solve_start
                rows.append(dict(t=after_read,read_ms=(after_read-begin)*1000,
                    tracking_ms=tracking*1000,solve_ms=solve*1000,detected=detected))
            elapsed=time.monotonic()-start
            steady=rows[5:]
            results[mode]=dict(frames=len(rows),seconds=elapsed,fps=len(rows)/elapsed,
                detected_frames=sum(r['detected'] for r in rows),
                read_ms_p50_p95=np.percentile([r['read_ms'] for r in steady],[50,95]).tolist(),
                tracking_ms_p50_p95=np.percentile([r['tracking_ms'] for r in steady],[50,95]).tolist(),
                solve_ms_p50_p95=np.percentile([r['solve_ms'] for r in steady],[50,95]).tolist())
            print(json.dumps({mode:results[mode]}),flush=True)
    finally:
        cap.release()
        if tracker:tracker.close()
        Path(a.out).write_text(json.dumps(results,indent=2))
    print('完成；未连接电机。结果：'+a.out,flush=True)


if __name__=='__main__':
    main()
