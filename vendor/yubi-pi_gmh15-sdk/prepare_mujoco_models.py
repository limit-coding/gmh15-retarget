from pathlib import Path
import xml.etree.ElementTree as ET
import shutil
import json
import mujoco

def prepare(side):
    base=Path('/opt/vendor')/f'DexH15_{side}_urdf_P'/f'DexH15_{side}_urdf'
    target=Path('/opt/models')/side
    target.mkdir(parents=True,exist_ok=True)
    shutil.copytree(base/'meshes',target/'meshes',dirs_exist_ok=True)
    root=ET.parse(base/'urdf'/f'DexH15_{side}_urdf.urdf').getroot()
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
    for mesh in root.iter('mesh'):
        mesh.set('filename',Path(mesh.get('filename')).name)
    extension=ET.SubElement(root,'mujoco')
    ET.SubElement(extension,'compiler',meshdir=str(target/'meshes'),strippath='true',
                  discardvisual='false',fusestatic='false',balanceinertia='true',
                  boundmass='0.000001',boundinertia='0.000000001')
    urdf=target/'import.urdf';ET.ElementTree(root).write(urdf,encoding='utf-8',xml_declaration=True)
    model=mujoco.MjModel.from_xml_path(str(urdf))
    mjcf=target/'hand.xml';mujoco.mj_saveLastXML(str(mjcf),model)
    # Explicit visual-only kinematic replay: no actuators and no dynamic stepping.
    result={'side':side,'nq':model.nq,'nv':model.nv,'nu':model.nu,
            'joints':[model.joint(i).name for i in range(model.njnt)],
            'source':str(base/'urdf'/f'DexH15_{side}_urdf.urdf')}
    (target/'manifest.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)

if __name__=='__main__':
    for side in ('left','right'):prepare(side)
