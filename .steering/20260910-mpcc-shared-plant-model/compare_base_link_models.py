"""Correct comparison origin to the serialized/observed controller base_link.

Prior body comparisons scored GoKart Rigidbody-origin motion. They correctly
show longitudinal force mismatch but cannot establish the controller's required
lateral state dimension. base_link is(0,.05,-.485) in the kart root; its COM is
+0.175992m forward. Private poses remain scoring-only, not predictor inputs.
"""
from pathlib import Path
import importlib.util
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation

spec=importlib.util.spec_from_file_location('base',Path(__file__).with_name('compare_public_inputs.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
OUT=Path('output/20260910-shared-plant-base-link-models-r1')
OFFSET=np.array(json.loads(Path('output/20260908-mpcc-stop-reference-single-r3/unity-collider-geometry.json').read_text())['base_link_in_kart_unity_xyz'])
base.P['com_x_m']-=OFFSET[2]
base.P['meaning']='Original fixed physical/contact values; pose origin corrected from Rigidbody to serialized controller base_link. No coefficient fit.'
READ_ROOT=base.read_truth
DERIVATIVE=base.derivative


def read_truth(name,identity):
    time,truth,tire=READ_ROOT(name,identity)
    raw=np.load(f'output/20260909-force-step-analysis-{name}-r1/physics.npz');ix={str(k):i for i,k in enumerate(raw['columns'])};a=raw['values'];a=a[a[:,ix['id']]==identity]
    shift=Rotation.from_quat(a[:,[ix['eq_'+c] for c in 'xyzw']]).apply(OFFSET)
    truth[:,:2]+=np.column_stack([shift[:,2],-shift[:,0]])
    return time,truth,tire


def derivative(s,wire,tire,model):
    if model!='reduced_force':return DERIVATIVE(s,wire,tire,model)
    # Zero lateral velocity at base_link, not at the Rigidbody origin or COM.
    # Retain the existing effective-yaw response law; replace wire=net only by
    # physical wheel force evaluated under this explicit reduced assumption.
    x,y,yaw,u,unused_vy,unused_r,rho=s
    r=.75*u*math.tan(rho)/1.087
    quasi=s.copy();quasi[4]=base.P['com_x_m']*r;quasi[5]=r
    force=DERIVATIVE(quasi,wire,tire,'dynamic_yaw')
    return np.array([u*math.cos(yaw),u*math.sin(yaw),r,force[3],0,0,(tire-rho)/.13])


base.read_truth=read_truth;base.derivative=derivative


def evaluate(name,domain,identity,begin,end):
    source=Path(f'output/20260909-force-step-ros-state-{name}-r1/{domain}.json');rows=json.loads(source.read_text());by={}
    for r in rows:by.setdefault(r['topic'].split('/')[-1],[]).append(r)
    for v in by.values():v.sort(key=lambda r:(r['stamp'],r['receipt']))
    def latest(key,t,receipt):return next((r for r in reversed(by[key]) if r['stamp']<=t and r['receipt']<=receipt),None)
    time,truth,tire=read_truth(name,identity);records=[]
    for imu in by['imu_raw']:
        t=imu['stamp'];receipt=imu['receipt']
        if not begin<=t<end:continue
        v=latest('velocity_status',t,receipt);steer=latest('steering_status',t,receipt)
        if v is None or steer is None or v['v']<.5 or max(t-v['stamp'],t-steer['stamp'])>.1:continue
        past=[r for r in by['control_cmd'] if r['stamp']<=t and r['receipt']<=receipt]
        if not past or past[0]['stamp']>t-.11:continue
        q=Rotation.from_quat([imu['orientation'][k] for k in 'xyzw'])*Rotation.from_euler('z',-math.pi/2)
        rate=imu['angular']['z'];state=np.array([0.,0.,q.as_euler('xyz')[2],v['v'],v['vy'],rate,math.atan(rate*1.087/(.75*v['v']))])
        history=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in past]
        future=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in by['control_cmd'] if t<r['stamp']<=t+1]
        j0=int(np.argmin(abs(time-t)))
        if abs(time[j0]-t)>1e-7:continue
        for model in ['legacy','reduced_force','dynamic_yaw']:
            predictions=base.integrate(state,steer['steering'],history,future,1.,model)
            for steps,pred,next_tire in predictions:
                target=t+steps*base.DT
                if target>=end:continue
                j=int(np.argmin(abs(time-target)))
                if abs(time[j]-target)>1e-6:continue
                actual=truth[j].copy();actual[:2]-=truth[j0,:2]
                e=pred[:6]-actual;e[2]=math.atan2(math.sin(e[2]),math.cos(e[2]))
                records.append(dict(t0=t,model=model,steps=steps,position_error_m=float(np.linalg.norm(e[:2])),
                  yaw_error_rad=float(e[2]),speed_error_mps=float(e[3]),tire_error_rad=float(next_tire-tire[j])))
    score={m:{str(h):{k:base.stats([r[k] for r in records if r['model']==m and r['steps']==h]) for k in ['position_error_m','yaw_error_rad','speed_error_mps','tire_error_rad']} for h in [20,50,100,200]} for m in ['legacy','reduced_force','dynamic_yaw']}
    return dict(window_sec=[begin,end],score=score,rows=records)


def absolute_position_check(domain,identity):
    report=json.loads(Path(f'output/20260909-initial-heading-node-replay-r4/{domain}/report.json').read_text())
    time,truth,tire=read_truth('dev2',identity);errors=[]
    for row in report['comparisons']:
        t=row['stamp_ns']*1e-9;j=int(np.argmin(abs(time-t)))
        if abs(time[j]-t)>1e-7:continue
        pose=row['output']['pose']['position']
        expected=truth[j,:2]+np.array([89637.703125,43503.5])
        errors.append(np.array([pose['x'],pose['y']])-expected)
    return dict(count=len(errors),x_error_m=base.stats(np.array(errors)[:,0]),y_error_m=base.stats(np.array(errors)[:,1]),
                limitation='MGRS offset is from serialized Environment1298; vertical height conversion excluded explicitly. GNSS quantization/noise remains.')


if __name__=='__main__':
    OUT.mkdir(exist_ok=False)
    report=dict(authority=False,meaning=__doc__,body_to_base_unity_xyz=OFFSET.tolist(),parameters=base.P,
     absolute_position_checks={d:absolute_position_check(d,i) for d,i in [('d1',1594692888),('d2',-615963298)]},
     single_holdout=evaluate('single','d1',-823441338,35,59.54),dev2={d:evaluate('dev2',d,i,7.585,9.709999782) for d,i in [('d1',1594692888),('d2',-615963298)]})
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('absolute',report['absolute_position_checks'],flush=True)
    for name,value in [('single',report['single_holdout']),*report['dev2'].items()]:
     print(name,{m:{h:{k:(v.get('mae'),v.get('maximum')) for k,v in fields.items() if k!='tire_error_rad'} for h,fields in z.items()} for m,z in value['score'].items()},flush=True)
