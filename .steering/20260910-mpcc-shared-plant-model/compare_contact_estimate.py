"""Causal past-window contact-weight estimate, using public sensors only.

Protocol fixed before execution:0.5s window, stable published longitudinal input
for0.2s before each interval, two bounded axle contact fractions, same frozen
physical coefficients. No future state, no changed holdout boundaries or retuning.
This is a model comparison; an estimated fraction is not a contact guarantee.
"""
from pathlib import Path
import importlib.util
import json
import math
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation

spec=importlib.util.spec_from_file_location('base',Path(__file__).with_name('compare_public_inputs.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
ORIGINAL_TRACTION=np.array(base.TRACTION);ORIGINAL_CORNER=np.array(base.CORNER)
# Per-grounded-wheel cornering is inferred from the existing directly measured
# physical/contact averages; parameters are frozen from16..35s training.
FULL_CORNER=ORIGINAL_CORNER/(4*ORIGINAL_TRACTION)
OUT=Path('output/20260910-shared-plant-contact-estimate-r1')


def estimate(by,t,receipt):
    series={key:[r for r in values if r['stamp']<=t and r['receipt']<=receipt] for key,values in by.items()}
    def interpolate(key,stamp,field):
        values=series[key];stamps=np.array([r['stamp'] for r in values])
        j=np.searchsorted(stamps,stamp)
        if j==0 or j>=len(stamps):return None
        a,b=values[j-1],values[j]
        if b['stamp']-a['stamp']>.1:return None
        f=(stamp-a['stamp'])/(b['stamp']-a['stamp'])
        return (1-f)*a[field]+f*b[field]
    imu=[r for r in series['imu_raw'] if t-.5<=r['stamp']<=t]
    A=[];B=[];intervals=[]
    for a,b in zip(imu,imu[1:]):
        dt=b['stamp']-a['stamp']
        if not .01<dt<=.1:continue
        u0=interpolate('velocity_status',a['stamp'],'v');u1=interpolate('velocity_status',b['stamp'],'v')
        v0=interpolate('velocity_status',a['stamp'],'vy');v1=interpolate('velocity_status',b['stamp'],'vy')
        tire=interpolate('steering_status',(a['stamp']+b['stamp'])/2,'steering')
        if any(v is None for v in [u0,u1,v0,v1,tire]) or min(u0,u1)<.5:continue
        commands=[r for r in series['control_cmd'] if a['stamp']-.2<=r['stamp']<=b['stamp']]
        if len(commands)<2 or commands[0]['stamp']>a['stamp']-.15:continue
        wires=[r['acceleration'] for r in commands]
        if max(wires)-min(wires)>.001:continue
        u=(u0+u1)/2;v=(v0+v1)/2;r=(a['angular']['z']+b['angular']['z'])/2
        wire=float(np.mean(wires));rolling=-min(.37,abs(u)/.01)
        mat=np.zeros((3,2))
        for n,(wx,wy) in enumerate(base.GEOMETRY):
            angle=tire if n<2 else 0;c=math.cos(angle);s=math.sin(angle)
            side=-s*(u-r*wy)+c*(v+r*wx)
            lateral=-FULL_CORNER[n]*side;drive=.25*(wire+rolling)
            ax=c*drive-s*lateral;ay=s*drive+c*lateral
            mat[:,n//2]+=np.array([ax,ay,(wx*ay-wy*ax)*base.P['mass_kg']/base.P['yaw_inertia_kgm2']*1.087])
        response=np.array([(u1-u0)/dt+.03*u-r*v,(v1-v0)/dt+.03*v+r*u,
                           ((b['angular']['z']-a['angular']['z'])/dt+r)*1.087])
        A.extend(mat);B.extend(response);intervals.append([a['stamp'],b['stamp']])
    if not A:return None
    A=np.array(A);B=np.array(B)
    if np.linalg.matrix_rank(A)<2:return None
    result=lsq_linear(A,B,bounds=(0,1),tol=1e-10)
    if not result.success:return None
    return dict(front=float(result.x[0]),rear=float(result.x[1]),intervals=intervals,
      residual_rms=float(np.sqrt(np.mean((A@result.x-B)**2))),condition=float(np.linalg.cond(A)))


def evaluate(name,domain,identity,begin,end):
    rows=json.loads(Path(f'output/20260909-force-step-ros-state-{name}-r1/{domain}.json').read_text())
    by={}
    for r in rows:by.setdefault(r['topic'].split('/')[-1],[]).append(r)
    for v in by.values():v.sort(key=lambda r:(r['stamp'],r['receipt']))
    def latest(key,t,receipt):return next((r for r in reversed(by[key]) if r['stamp']<=t and r['receipt']<=receipt),None)
    times,truth,tire_truth=base.read_truth(name,identity);records=[];estimates=[];missing=0
    for imu in by['imu_raw']:
        t=imu['stamp'];receipt=imu['receipt']
        if not begin<=t<end:continue
        velocity=latest('velocity_status',t,receipt);steer=latest('steering_status',t,receipt)
        if velocity is None or steer is None or velocity['v']<.5:continue
        if max(t-velocity['stamp'],t-steer['stamp'])>.1:continue
        calibration=estimate(by,t,receipt)
        if calibration is None:missing+=1;continue
        estimates.append(dict(t=t,**calibration))
        q=Rotation.from_quat([imu['orientation'][k] for k in 'xyzw'])*Rotation.from_euler('z',-math.pi/2)
        rate=imu['angular']['z'];state=np.array([0.,0.,q.as_euler('xyz')[2],velocity['v'],velocity['vy'],rate,math.atan(rate*1.087/(.75*velocity['v']))])
        past=[r for r in by['control_cmd'] if r['stamp']<=t and r['receipt']<=receipt]
        if not past or past[0]['stamp']>t-.11:continue
        history=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in past]
        future=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in by['control_cmd'] if t<r['stamp']<=t+1]
        j0=int(np.argmin(abs(times-t)))
        if abs(times[j0]-t)>1e-7:continue
        for model in ['fixed_training','past_contact_estimate']:
            weights=np.array([calibration['front']]*2+[calibration['rear']]*2)
            base.TRACTION=ORIGINAL_TRACTION if model=='fixed_training' else .25*weights
            base.CORNER=ORIGINAL_CORNER if model=='fixed_training' else FULL_CORNER*weights
            predictions=base.integrate(state,steer['steering'],history,future,1.,'dynamic_yaw')
            for steps,pred,tire in predictions:
                target=t+steps*base.DT
                if target>=end:continue
                j=int(np.argmin(abs(times-target)))
                if abs(times[j]-target)>1e-6:continue
                actual=truth[j].copy();actual[:2]-=truth[j0,:2]
                delta=pred[:6]-actual;delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
                records.append(dict(t0=t,model=model,steps=steps,position_error_m=float(np.linalg.norm(delta[:2])),
                  yaw_error_rad=float(delta[2]),speed_error_mps=float(delta[3]),lateral_error_mps=float(delta[4])))
    base.TRACTION=ORIGINAL_TRACTION;base.CORNER=ORIGINAL_CORNER
    scores={m:{str(h):{k:base.stats([r[k] for r in records if r['model']==m and r['steps']==h]) for k in ['position_error_m','yaw_error_rad','speed_error_mps','lateral_error_mps']} for h in [20,50,100,200]} for m in ['fixed_training','past_contact_estimate']}
    return dict(window_sec=[begin,end],missing_estimate_anchors=missing,estimates=estimates,score=scores,rows=records)


OUT.mkdir(exist_ok=False)
report=dict(authority=False,meaning=__doc__,parameters=base.P,
 limits=['Only anchors with an identifiable past estimate compare both arms; missing counts retained.',
  'Input stability over0.2s is an experiment condition, not a guarantee of0.2sapplication.',
  'Bounded least squares uses2axle-contact fractions; coefficient0..1 is a physical-domain constraint, not relaxed safety or command clamp.',
  'Online data can contain unmodeled pitch/roll/contact forces. No accepted production uncertainty envelope or Rest proof.'],
 single_holdout=evaluate('single','d1',-823441338,35,59.54),
 dev2={d:evaluate('dev2',d,i,7.585,9.709999782) for d,i in [('d1',1594692888),('d2',-615963298)]})
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
for name,value in [('single',report['single_holdout']),*report['dev2'].items()]:
 print(name,'missing',value['missing_estimate_anchors'],'estimates',len(value['estimates']))
 print({m:{h:{k:(v.get('count'),v.get('mae'),v.get('maximum')) for k,v in x.items()} for h,x in hs.items()} for m,hs in value['score'].items()},flush=True)
