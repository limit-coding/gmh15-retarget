from pathlib import Path
import xml.etree.ElementTree as E
import json
base=Path('/home/limit/gmh15-sdk/root/usr/share/dexhandsdk/config/force_ctl')
for side in ('left','right'):
    for variant in ('P','C'):
        p=base/f'DexH15_{side}_urdf_{variant}'/f'DexH15_{side}_urdf/urdf/DexH15_{side}_urdf.urdf'
        r=E.parse(p).getroot()
        print(json.dumps({'side':side,'variant':variant,'joints':[{'name':j.get('name'),'type':j.get('type'),'axis':j.find('axis').attrib if j.find('axis') is not None else None,'limit':j.find('limit').attrib if j.find('limit') is not None else None,'mimic':j.find('mimic').attrib if j.find('mimic') is not None else None} for j in r.findall('joint') if j.get('type')!='fixed']}),flush=True)
