"""Advance source-stamped tire observation to the IMU anchor using past commands.

SteeringReport observes before Vehicle.FixedUpdate actuator update. Reconstruct
updates in [report_source,anchor); the rollout then updates at anchor normally.
No delay fitting or future sensor use. Actual input-receive delay stays unresolved.
"""
from pathlib import Path
import importlib.util
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation

spec=importlib.util.spec_from_file_location('base',Path(__file__).with_name('compare_public_inputs.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
OUT=Path('output/20260910-shared-plant-steering-source-r1')


def advance_tire(value,source,anchor,history):
    count=round((anchor-source)/base.DT)
    assert 0<=count<=20 and abs(count*base.DT-(anchor-source))<1e-7
    for k in range(count):
        time=source+k*base.DT
        command=next((r for r in reversed(history) if r['stamp']<=time-.1),None)
        if command is None:return None
        demand=.7*np.clip(command['wire_steering'],-math.pi/6,math.pi/6)
        value+=np.clip(base.DT/(.02+base.DT)*(demand-value),-math.radians(80)*base.DT,math.radians(80)*base.DT)
    return value


def evaluate(name,domain,identity,begin,end):
    rows=json.loads(Path(f'output/20260909-force-step-ros-state-{name}-r1/{domain}.json').read_text())
    by={}
    for r in rows:by.setdefault(r['topic'].split('/')[-1],[]).append(r)
    for v in by.values():v.sort(key=lambda r:(r['stamp'],r['receipt']))
    def latest(key,t,receipt):return next((r for r in reversed(by[key]) if r['stamp']<=t and r['receipt']<=receipt),None)
    times,truth,true_tire=base.read_truth(name,identity);records=[]
    for imu in by['imu_raw']:
        t=imu['stamp'];receipt=imu['receipt']
        if not begin<=t<end:continue
        v=latest('velocity_status',t,receipt);steer=latest('steering_status',t,receipt)
        if v is None or steer is None or v['v']<.5 or max(t-v['stamp'],t-steer['stamp'])>.1:continue
        past=[r for r in by['control_cmd'] if r['stamp']<=t and r['receipt']<=receipt]
        if not past or past[0]['stamp']>t-.2:continue
        tire=advance_tire(steer['steering'],steer['stamp'],t,past)
        if tire is None:continue
        q=Rotation.from_quat([imu['orientation'][k] for k in 'xyzw'])*Rotation.from_euler('z',-math.pi/2)
        rate=imu['angular']['z'];state=np.array([0.,0.,q.as_euler('xyz')[2],v['v'],v['vy'],rate,math.atan(rate*1.087/(.75*v['v']))])
        history=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in past]
        future=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in by['control_cmd'] if t<r['stamp']<=t+1]
        j0=int(np.argmin(abs(times-t)))
        if abs(times[j0]-t)>1e-7:continue
        for model,initial_tire in [('source_held',steer['steering']),('source_propagated',tire)]:
            predictions=base.integrate(state,initial_tire,history,future,1.,'dynamic_yaw')
            for steps,pred,tire_next in predictions:
                target=t+steps*base.DT
                if target>=end:continue
                j=int(np.argmin(abs(times-target)))
                if abs(times[j]-target)>1e-6:continue
                actual=truth[j].copy();actual[:2]-=truth[j0,:2]
                e=pred[:6]-actual;e[2]=math.atan2(math.sin(e[2]),math.cos(e[2]))
                records.append(dict(t0=t,model=model,steps=steps,source_age_sec=t-steer['stamp'],
                  position_error_m=float(np.linalg.norm(e[:2])),yaw_error_rad=float(e[2]),speed_error_mps=float(e[3]),
                  tire_error_rad=float(tire_next-true_tire[j])))
    score={m:{str(h):{k:base.stats([r[k] for r in records if r['model']==m and r['steps']==h]) for k in ['position_error_m','yaw_error_rad','speed_error_mps','tire_error_rad']} for h in [20,50,100,200]} for m in ['source_held','source_propagated']}
    return dict(window_sec=[begin,end],score=score,rows=records)


OUT.mkdir(exist_ok=False)
report=dict(authority=False,meaning=__doc__,parameters=base.P,single_holdout=evaluate('single','d1',-823441338,35,59.54),
 dev2={d:evaluate('dev2',d,i,7.585,9.709999782) for d,i in [('d1',1594692888),('d2',-615963298)]})
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
for name,value in [('single',report['single_holdout']),*report['dev2'].items()]:
 print(name,{m:{h:{k:(v.get('mae'),v.get('maximum')) for k,v in f.items()} for h,f in hs.items()} for m,hs in value['score'].items()},flush=True)
