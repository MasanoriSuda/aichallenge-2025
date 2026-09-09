"""Public-source initialized body rollouts; future physics only scores predictions.

Zero translation is a relative-displacement origin, not privileged localization.
Sensor and command histories use recorded local receipt plus nonfuture source.
The prescribed-publications arm has exogenous future commands, never sensors.
Published source is not simulator receipt/application; no latency bound claimed.
"""
from pathlib import Path
import bisect
import hashlib
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation

OUT=Path('output/20260910-shared-plant-public-inputs-r1')
P=json.loads(Path('output/20260909-force-step-body-models-r1/report.json').read_text())['parameters']
GEOMETRY=P['wheel_com_xy_m']; TRACTION=P['wheel_traction_fraction']; CORNER=P['wheel_cornering_per_sec']
DT=.005


def stats(a):
    a=np.asarray(a,dtype=float)
    return dict(count=len(a),mean=float(a.mean()),mae=float(abs(a).mean()),p95=float(np.percentile(abs(a),95)),maximum=float(abs(a).max())) if len(a) else dict(count=0)


def derivative(s,wire,tire,model):
    x,y,yaw,u,vy,r,rho=s
    if model=='legacy':
        r=.75*u*math.tan(rho)/1.087
        return np.array([u*math.cos(yaw),u*math.sin(yaw),r,wire,0,0,(tire-rho)/.13])
    acceleration=max(wire,-u/.01) if u<0 else wire
    rolling=-math.copysign(min(P['rolling_mps2'],abs(u)/.01),u) if u else 0
    fx=fy=torque=0.
    for n,(wx,wy) in enumerate(GEOMETRY):
        angle=tire if n<2 else 0.;c=math.cos(angle);sn=math.sin(angle)
        side=-sn*(u-r*wy)+c*(vy+r*wx)
        lateral=-CORNER[n]*side
        drive=TRACTION[n]*(acceleration+rolling)
        ax=c*drive-sn*lateral;ay=sn*drive+c*lateral
        fx+=ax;fy+=ay;torque+=wx*ay-wy*ax
    ref_vy=vy-P['com_x_m']*r
    return np.array([u*math.cos(yaw)-ref_vy*math.sin(yaw),u*math.sin(yaw)+ref_vy*math.cos(yaw),r,
                     fx-P['drag_per_sec']*u+r*vy,fy-P['drag_per_sec']*vy-r*u,
                     torque*P['mass_kg']/P['yaw_inertia_kgm2']-P['angular_drag_per_sec']*r,0.])


def integrate(initial,tire_initial,command_history,future_schedule,duration,model):
    """No simulator truth argument or global truth access. Times relative to anchor."""
    commands=command_history+future_schedule
    stamps=[p[0] for p in commands]
    assert stamps==sorted(stamps)
    def command(t):
        i=bisect.bisect_right(stamps,t)-1
        if i<0:raise ValueError('incomplete command prefix')
        return commands[i][1:]
    state=initial.copy();tire=tire_initial;predictions=[]
    for k in range(round(duration/DT)):
        elapsed=k*DT
        wire,unused=command(elapsed)
        unused,delayed=command(elapsed-.1)
        # Ackermann tire-angle wire is converted directly to degrees then limited;
        # effective local grip is0.7. Actual receive/application uncertainty remains.
        demand=.7*np.clip(delayed,-math.pi/6,math.pi/6)
        tire+=np.clip(DT/(.02+DT)*(demand-tire),-math.radians(80)*DT,math.radians(80)*DT)
        a=derivative(state,wire,tire,model)
        state+=DT*derivative(state+.5*DT*a,wire,tire,model)
        if k+1 in [20,50,100,200]:predictions.append((k+1,state.copy(),tire))
    return predictions


def read_truth(name,identity):
    path=Path(f'output/20260909-force-step-analysis-{name}-r1/physics.npz')
    raw=np.load(path);ix={str(k):i for i,k in enumerate(raw['columns'])};a=raw['values'];a=a[a[:,ix['id']]==identity]
    q=a[:,[ix['eq_'+c] for c in 'xyzw']];rot=Rotation.from_quat(q).as_matrix()
    v=np.einsum('nji,nj->ni',rot,a[:,[ix['ev_'+c] for c in 'xyz']])
    w=np.einsum('nji,nj->ni',rot,a[:,[ix['ew_'+c] for c in 'xyz']])
    yaw=np.arctan2(rot[:,2,2],rot[:,0,2])-np.pi/2
    truth=np.stack([a[:,ix['ep_z']],-a[:,ix['ep_x']],yaw,v[:,2],-v[:,0],-w[:,1]],axis=1)
    return a[:,ix['fixed']],truth,-a[:,ix['steer_deg']]*np.pi/180


def evaluate(name,domain,identity,begin,end):
    source=Path(f'output/20260909-force-step-ros-state-{name}-r1/{domain}.json')
    rows=json.loads(source.read_text());by={}
    for row in rows:by.setdefault(row['topic'].split('/')[-1],[]).append(row)
    for series in by.values():series.sort(key=lambda r:(r['stamp'],r['receipt']))
    def latest(kind,t,receipt):
        return next((r for r in reversed(by[kind]) if r['stamp']<=t and r['receipt']<=receipt),None)
    times,truth,true_tire=read_truth(name,identity)
    records=[];skipped={};max_skew=0.
    for imu in by['imu_raw']:
        t=imu['stamp'];receipt=imu['receipt']
        if not begin<=t<end:continue
        velocity=latest('velocity_status',t,receipt);steer=latest('steering_status',t,receipt)
        if velocity is None or steer is None or velocity['v']<.5:continue
        skew=max(t-velocity['stamp'],t-steer['stamp']);max_skew=max(max_skew,skew)
        if skew>.1:skipped['stale_sensor']=skipped.get('stale_sensor',0)+1;continue
        public=[r for r in by['control_cmd'] if r['stamp']<=t and r['receipt']<=receipt]
        if not public or public[0]['stamp']>t-.11:continue
        q=Rotation.from_quat([imu['orientation'][k] for k in 'xyzw'])*Rotation.from_euler('z',-math.pi/2)
        yaw=q.as_euler('xyz')[2];rate=imu['angular']['z']
        state=np.array([0.,0.,yaw,velocity['v'],velocity['vy'],rate,math.atan(rate*1.087/(.75*velocity['v']))])
        history=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in public]
        future=[(r['stamp']-t,r['acceleration'],r['wire_steering']) for r in by['control_cmd'] if t<r['stamp']<=t+1]
        j0=int(np.argmin(abs(times-t)))
        if abs(times[j0]-t)>1e-7:continue
        for schedule in ['hold_last_published','prescribed_publications']:
            for model in ['legacy','dynamic_yaw']:
                predictions=integrate(state,steer['steering'],history,future if schedule=='prescribed_publications' else [],1.,model)
                for steps,pred,tire in predictions:
                    target=t+steps*DT
                    if target>=end:continue
                    j=int(np.argmin(abs(times-target)))
                    if abs(times[j]-target)>1e-6:continue
                    actual=truth[j].copy();actual[:2]-=truth[j0,:2]
                    delta=pred[:6]-actual;delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
                    records.append(dict(t0=t,model=model,schedule=schedule,steps=steps,sensor_skew_sec=skew,
                        position_error_m=float(np.linalg.norm(delta[:2])),yaw_error_rad=float(delta[2]),
                        speed_error_mps=float(delta[3]),lateral_error_mps=float(delta[4]),
                        tire_error_rad=float(tire-true_tire[j]),initial_speed_error_mps=float(state[3]-truth[j0,3])))
    score={}
    for schedule in ['hold_last_published','prescribed_publications']:
        score[schedule]={}
        for model in ['legacy','dynamic_yaw']:
            score[schedule][model]={}
            for steps in [20,50,100,200]:
                group=[r for r in records if r['schedule']==schedule and r['model']==model and r['steps']==steps]
                score[schedule][model][str(steps)]={key:stats([r[key] for r in group]) for key in ['position_error_m','yaw_error_rad','speed_error_mps','lateral_error_mps','tire_error_rad','initial_speed_error_mps']}
    return dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),window_sec=[begin,end],maximum_sensor_skew_sec=max_skew,skipped=skipped,score=score,rows=records)


if __name__=='__main__':
    OUT.mkdir(exist_ok=False)
    report=dict(authority=False,meaning=__doc__,parameters=P,nominal_publication_to_longitudinal_delay_sec=0,
      limitations=['Longitudinal latest-value10Hz Update/actual DDS receipt unobserved; immediate nominal publication is not selected as a verified plant delay.',
       'Sensor initial vx/vy and steering are latest arrived values up to0.1s old, held to IMU source epoch. Source skew is reported, not silently relabelled as exact.',
       'Contact training values fixed; complete Rest/gear/sleep and uncertainty certificate remain open.',
       'Legacy comparison uses same public initial yaw/speed and prescribed actuator output, not full runtime seven-state prefix.'],
      single_holdout=evaluate('single','d1',-823441338,35,59.54),
      dev2={d:evaluate('dev2',d,i,7.585,9.709999782) for d,i in [('d1',1594692888),('d2',-615963298)]})
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for name,value in [('single_holdout',report['single_holdout']),*report['dev2'].items()]:
        print(name,'max_sensor_skew',value['maximum_sensor_skew_sec'])
        for schedule,models in value['score'].items():
            print(schedule,{model:{h:{k:round(v['mae'],6) for k,v in fields.items() if 'mae' in v} for h,fields in horizons.items()} for model,horizons in models.items()},flush=True)
