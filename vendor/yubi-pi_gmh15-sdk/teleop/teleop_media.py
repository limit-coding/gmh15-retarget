"""MuJoCo target preview with projected draggable joint markers. No serial access."""
import math
import time
from pathlib import Path
from teleop_control import EDITABLE, JOINT_NAMES


def target_worker(side, states, targets, frames, metadata, views, model_root='/opt/models', kind='target'):
    import numpy as np
    import mujoco
    from PIL import Image, ImageDraw
    try:
        model = mujoco.MjModel.from_xml_path(str(Path(model_root)/side/'hand.xml'))
        assert model.nq == 15 and model.nu == 0
        data = mujoco.MjData(model)
        names = [f'{side}_{finger}_joint_{j}' for finger in ('index','middle','ring','pinky','thumb') for j in range(3)]
        ids = [int(model.joint(name).id) for name in names]
        addresses = [int(model.jnt_qposadr[i]) for i in ids]
        camera = mujoco.MjvCamera()
        camera.lookat[:] = model.stat.center
        camera.distance = model.stat.extent * 1.1
        camera.elevation = -15
        count = 0;started=time.monotonic()
        with mujoco.Renderer(model, height=420, width=560) as renderer:
            while True:
                begin = time.monotonic()
                state = states.get(side, {})
                actual = state.get('joints_degrees', [])
                live = bool(state.get('connected') and not state.get('joint_error') and len(actual)==15
                            and time.time()-state.get('timestamp',0)<1.5)
                pose = list(actual) if live else [0.]*15
                target = targets.get(side, {}).get('angles_degrees') if kind=='target' else None
                if target and len(target)==11:
                    for index, value in zip(EDITABLE, target): pose[index] = value
                for adr, value in zip(addresses, pose): data.qpos[adr] = math.radians(value)
                mujoco.mj_forward(model, data)
                camera.azimuth = {'front':90,'back':270,'oblique':135,'side':0}.get(views.get(side,'oblique'),135)
                renderer.update_scene(data, camera=camera)
                rotation = Image.Transpose.ROTATE_180 if hasattr(Image,'Transpose') else Image.ROTATE_180
                im = Image.fromarray(renderer.render()).transpose(rotation)
                cams = renderer.scene.camera
                position = (np.array(cams[0].pos)+np.array(cams[1].pos))*.5
                forward = np.array(cams[0].forward); up = np.array(cams[0].up)
                right = np.cross(forward, up)
                tangent = math.tan(math.radians(float(model.vis.global_.fovy))*.5)
                markers=[]; draw=ImageDraw.Draw(im)
                for index, pose_index in enumerate(EDITABLE if kind=='target' else ()):
                    vector = data.xanchor[ids[pose_index]]-position
                    depth = float(vector@forward)
                    if depth <= 0: continue
                    x=560-(1+float(vector@right)/(depth*tangent*560/420))*280
                    y=420-(1-float(vector@up)/(depth*tangent))*210
                    if not (0<=x<=560 and 0<=y<=420):continue
                    markers.append(dict(index=index,x=round(x,1),y=round(y,1),label=JOINT_NAMES[index]))
                    draw.ellipse((x-8,y-8,x+8,y+8),fill='#165e68',outline='#73f5df',width=2)
                    draw.text((x-4,y-6),str(index+1),fill='white')
                draw.text((10,10),'TARGET PREVIEW / drag joint markers' if kind=='target' else 'SAVED FEEDBACK SAMPLE / OFFLINE',fill='#2af0c0')
                import io
                buf=io.BytesIO();im.save(buf,format='JPEG',quality=86)
                frames[kind+'-'+side]=buf.getvalue();count+=1
                metadata[kind+'-'+side]=dict(ok=True,timestamp=time.time(),sequence=count,
                    width=560,height=420,markers=markers,angles_degrees=[pose[i] for i in EDITABLE],
                    passive_DIP_source='current measured feedback (held during target preview)' if live else 'no valid feedback; reference zeros',
                    live_feedback_available=live and not state.get('demo',False),source_sequence=state.get('sequence'),
                    mujoco_version=mujoco.__version__,live_pose=False,fps=round(count/(time.monotonic()-started),1),demo=bool(state.get('demo')))
                time.sleep(max(0,.2-(time.monotonic()-begin)))
    except Exception as exc:
        metadata[kind+'-'+side]=dict(ok=False,timestamp=time.time(),error=str(exc))
        print(side,'target render:',exc,flush=True)
