"""Camera and MuJoCo display workers. This module never opens a robot serial port."""
import time
import math

def camera_worker(side, device, frames, metadata):
    import cv2
    cap=None
    count=0
    started=time.monotonic()
    try:
        cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        cap.set(cv2.CAP_PROP_FPS,10)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        if not cap.isOpened():raise RuntimeError('Camera open failed: '+device)
        while True:
            begin=time.monotonic()
            ok,image=cap.read()
            if not ok:raise RuntimeError('Camera capture failed')
            ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])
            if not ok:raise RuntimeError('JPEG encode failed')
            frames['camera-'+side]=jpeg.tobytes();count+=1
            metadata['camera-'+side]={'ok':True,'timestamp':time.time(),'sequence':count,
                'fps':round(count/(time.monotonic()-started),1),'width':image.shape[1],'height':image.shape[0]}
            time.sleep(max(0,.1-(time.monotonic()-begin)))
    except Exception as exc:
        metadata['camera-'+side]={'ok':False,'error':str(exc),'timestamp':time.time()}
        print(side,'camera:',exc,flush=True)
    finally:
        if cap is not None:cap.release()

def render_worker(side, states, frames, metadata, views):
    import cv2
    import mujoco
    import numpy as np
    renderer=None
    try:
        model=mujoco.MjModel.from_xml_path(f'/opt/models/{side}/hand.xml')
        if model.nq!=15 or model.nu!=0:raise RuntimeError('Unexpected model joint/actuator count')
        data=mujoco.MjData(model)
        renderer=mujoco.Renderer(model,height=420,width=560)
        camera=mujoco.MjvCamera()
        camera.lookat[:]=model.stat.center
        camera.distance=model.stat.extent*1.1
        camera.elevation=-15
        mapping=[f'{side}_{f}_joint_{j}' for f in ('index','middle','ring','pinky','thumb') for j in range(3)]
        addresses=[int(model.joint(name).qposadr[0]) for name in mapping]
        count=0;started=time.monotonic()
        while True:
            begin=time.monotonic()
            state=states.get(side,{})
            angles=state.get('joints_degrees',[])
            valid=(state.get('connected') and not state.get('joint_error') and len(angles)==15
                   and time.time()-state.get('timestamp',0)<1.5)
            data.qpos[:]=0
            outside=[]
            if valid:
                for name,adr,angle in zip(mapping,addresses,angles):
                    data.qpos[adr]=math.radians(angle)
                    joint=model.joint(name)
                    low,high=joint.range
                    if data.qpos[adr]<low-.02 or data.qpos[adr]>high+.02:outside.append(name)
            view=views.get(side,'oblique')
            camera.azimuth={'front':90,'back':270,'oblique':135,'side':0}.get(view,135)
            mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=camera)
            image=renderer.render()
            image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            image=cv2.rotate(image,cv2.ROTATE_180)
            label='LIVE / 15 SDK joints' if valid else 'REFERENCE POSE / joint data unavailable'
            cv2.putText(image,label,(12,24),cv2.FONT_HERSHEY_SIMPLEX,.52,(90,225,190) if valid else (80,180,255),1,cv2.LINE_AA)
            ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,83])
            if not ok:raise RuntimeError('MuJoCo JPEG encode failed')
            frames['sim-'+side]=jpeg.tobytes();count+=1
            metadata['sim-'+side]={'ok':True,'timestamp':time.time(),'sequence':count,
                'fps':round(count/(time.monotonic()-started),1),'live_pose':bool(valid),
                'source_sequence':state.get('sequence'),'source_timestamp':state.get('timestamp'),
                'joint_count':15,'actuator_count':0,'out_of_model_range':outside,
                'mujoco_version':mujoco.__version__,'view':view,
                'display_rotation_degrees':180,
                'source_urdf':f'DexH15_{side}_urdf_P','joint_mapping':mapping}
            time.sleep(max(0,.25-(time.monotonic()-begin)))
    except Exception as exc:
        metadata['sim-'+side]={'ok':False,'error':str(exc),'timestamp':time.time()}
        print(side,'render:',exc,flush=True)
    finally:
        if renderer is not None:renderer.close()
