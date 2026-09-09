"""Read serialized wheel/rigidbody physics settings; no asset or runtime mutation."""
from pathlib import Path
import hashlib
import json
import UnityPy

root=Path('aichallenge/simulator/AWSIM/AWSIM_Data')
env=UnityPy.load(str(root/'level1'))
objects={o.path_id:o for o in env.objects if o.assets_file.name=='level1'}
assert objects
transforms={o.path_id:o.parse_as_dict() for o in objects.values() if o.type.name=='Transform'}
go_transform={t['m_GameObject']['m_PathID']:i for i,t in transforms.items()}


def path_for(tid):
    t=transforms[tid];name=objects[t['m_GameObject']['m_PathID']].parse_as_dict()['m_Name']
    parent=t['m_Father']['m_PathID']
    return (path_for(parent) if parent else '')+'/'+name


rows=[]
for obj in objects.values():
    if obj.type.name not in ['WheelCollider','Rigidbody']:continue
    data=obj.parse_as_dict();tid=go_transform[data['m_GameObject']['m_PathID']];path=path_for(tid)
    if '/GoKart' not in path:continue
    chain=[];current=tid
    while current:
        t=transforms[current]
        chain.append(dict(transform_id=current,path=path_for(current),position=t['m_LocalPosition'],rotation=t['m_LocalRotation'],scale=t['m_LocalScale']))
        current=t['m_Father']['m_PathID']
    rows.append(dict(path=path,object_id=obj.path_id,type=obj.type.name,data=data,chain=chain))
OUT=Path('output/20260910-shared-plant-wheel-colliders-r1');OUT.mkdir(exist_ok=False)
report=dict(authority=False,meaning=__doc__,UnityPy_version=UnityPy.__version__,source='level1',sha256=hashlib.sha256((root/'level1').read_bytes()).hexdigest(),objects=rows)
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
for r in rows:
    if '/GoKart1/' in r['path'] or r['path'].endswith('/GoKart1'):print(r['path'],r['type'],json.dumps(r['data']),flush=True)
