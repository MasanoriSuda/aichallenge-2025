"""Causal body-plant rollout with prescribed wire/tire inputs, no future body state.

The input boundary is AFTER the steering actuator. Recorded tire-angle and wire
sequences are exogenous test inputs, not claimed available at a prior decision.
Contact and body-state futures are withheld. Not an end-to-end controller proof.
"""
from pathlib import Path
import json
import math
import numpy as np

out=Path('output/20260909-force-step-body-models-r1');out.mkdir(exist_ok=False)
def read(path):
    raw=np.load(path);return raw['values'],{str(k):i for i,k in enumerate(raw['columns'])}
def field(a,index,key):return a[:,index[key]]
def vector(a,index,key):return a[:,[index[key+x] for x in 'xyz']]
def rotation(a,index):
    q=a[:,[index['eq_'+x] for x in 'xyzw']];q/=np.linalg.norm(q,axis=1)[:,None]
    x,y,z,w=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
        2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)
def states(a,ix):
    rot=rotation(a,ix);vel=np.einsum('nji,nj->ni',rot,vector(a,ix,'ev_'))
    omega=np.einsum('nji,nj->ni',rot,vector(a,ix,'ew_'))
    position=vector(a,ix,'ep_')
    yaw=np.arctan2(rot[:,2,2],rot[:,0,2])
    # Plane coordinates X=Unity world x, Y=Unity world z; local left=-Unity right.
    return np.stack([position[:,0],position[:,2],yaw,vel[:,2],-vel[:,0],-omega[:,1]],axis=1)
single,ix=read('output/20260909-force-step-analysis-single-r1/physics.npz')
truth_single=states(single,ix);t=field(single,ix,'fixed')
training=(t>=16)&(t<35)&(truth_single[:,3]>=.5)&(field(single,ix,'sleep')==0)
rot=rotation(single,ix)
offset=np.einsum('nji,nj->ni',rot,vector(single,ix,'com_')-vector(single,ix,'ep_'))
com_x=float(np.mean(offset[training,2]));mass=float(field(single,ix,'mass')[0]);inertia=float(field(single,ix,'inertia_y')[0])
geometry=[];traction=[];cornering=[]
for n in range(4):
    key=f'w{n}_';ground=field(single,ix,key+'ground');mask=training&(ground==1)
    point=np.einsum('nji,nj->ni',rot,vector(single,ix,key+'p_')-vector(single,ix,'com_'))
    geometry.append([float(np.mean(point[mask,2])),float(np.mean(-point[mask,0]))])
    traction.append(float(np.mean(ground[training]))/4)
    cornering.append(float(np.mean((ground*field(single,ix,key+'sprung')/field(single,ix,'mass')*.236/field(single,ix,'dt'))[training])))
# Preserve left/right symmetry; training does not assign a preferred side.
for start in [0,2]:
    forward=np.mean([geometry[start][0],geometry[start+1][0]])
    half_track=np.mean([abs(geometry[start][1]),abs(geometry[start+1][1])])
    for n in [start,start+1]:geometry[n]=[forward,half_track if n%2==0 else -half_track]
    traction[start:start+2]=[np.mean(traction[start:start+2])]*2
    cornering[start:start+2]=[np.mean(cornering[start:start+2])]*2
parameters=dict(training_sec=[16,35],training_samples=int(training.sum()),com_x_m=com_x,
    mass_kg=mass,yaw_inertia_kgm2=inertia,wheel_com_xy_m=geometry,
    wheel_traction_fraction=traction,wheel_cornering_per_sec=cornering,
    rolling_mps2=.37,drag_per_sec=.03,angular_drag_per_sec=1,
    meaning='Direct averages of runtime physical/contact values over training window, symmetrized. No fitting to future body motion. Diagnostic only.')
def derivative(s,wire,delta,model):
    x,y,yaw,u,vy,r,rho=s
    if model!='dynamic_yaw':r=.75*u*math.tan(rho)/1.087
    if model=='legacy':
        return np.array([u*math.cos(yaw),u*math.sin(yaw),r,wire,0,0,(delta-rho)/.13])
    acceleration=wire
    if u<0 and acceleration < -u/.01:acceleration=-u/.01
    rolling=-math.copysign(min(.37,abs(u)/.01),u) if u else 0
    fx=fy=torque=0.
    for n,(wx,wy) in enumerate(geometry):
        angle=delta if n<2 else 0.;c=math.cos(angle);sn=math.sin(angle)
        point_u=u-r*wy;point_v=vy+r*wx
        slip=-sn*point_u+c*point_v
        lateral=-cornering[n]*slip
        drive=traction[n]*(acceleration+rolling)
        ax=c*drive-sn*lateral;ay=sn*drive+c*lateral
        fx+=ax;fy+=ay;torque+=wx*ay-wy*ax
    base_vy=vy-com_x*r
    return np.array([u*math.cos(yaw)-base_vy*math.sin(yaw),
        u*math.sin(yaw)+base_vy*math.cos(yaw),r,
        fx-.03*u+r*vy,fy-.03*vy-r*u,
        torque*mass/inertia-r if model=='dynamic_yaw' else 0,
        (delta-rho)/.13])
def stats(a):
    a=np.asarray(a);return dict(count=len(a),mae=float(np.mean(abs(a))),p95=float(np.percentile(abs(a),95)),maximum=float(np.max(abs(a)))) if len(a) else dict(count=0)
def evaluate(data,ix,begin,end):
    truth=states(data,ix);time=field(data,ix,'fixed');wire=field(data,ix,'wire')
    delta=-field(data,ix,'steer_deg')*np.pi/180.;dt=float(field(data,ix,'dt')[0])
    rows=[]
    for start in range(0,len(data)-201,20):
        if not begin<=time[start]<end:continue
        if truth[start,3]<.5:continue
        initial=np.r_[truth[start],math.atan(truth[start,5]*1.087/(.75*truth[start,3]))]
        for model in ['legacy','response_yaw_lateral','dynamic_yaw']:
            s=initial.copy()
            for k in range(200):
                j=start+k
                if time[j+1]>=end:break
                # Same imposed force-input series in every arm. RK midpoint.
                d1=derivative(s,wire[j],delta[j],model)
                s+=dt*derivative(s+.5*dt*d1,wire[j],delta[j],model)
                if k+1 in [20,50,100,200] and truth[j+1,3]>=.5:
                    error=s[:6]-truth[j+1];error[2]=math.atan2(math.sin(error[2]),math.cos(error[2]))
                    predicted_r=s[5] if model=='dynamic_yaw' else .75*s[3]*math.tan(s[6])/1.087
                    rows.append(dict(model=model,t0=float(time[start]),duration_sec=(k+1)*dt,
                        horizon_steps=k+1,position_error_m=float(np.linalg.norm(error[:2])),
                        yaw_error_rad=float(error[2]),longitudinal_error_mps=float(error[3]),
                        lateral_error_mps=float(error[4]) if model!='legacy' else None,
                        yaw_rate_error_radps=float(predicted_r-truth[j+1,5])))
    score={}
    for model in ['legacy','response_yaw_lateral','dynamic_yaw']:
        score[model]={}
        for steps in [20,50,100,200]:
            group=[r for r in rows if r['model']==model and r['horizon_steps']==steps]
            score[model][str(steps)]=dict(duration_sec=steps*dt,**{key:stats([r[key] for r in group if r[key] is not None]) for key in ['position_error_m','yaw_error_rad','longitudinal_error_mps','lateral_error_mps','yaw_rate_error_radps']})
    return dict(window_sec=[begin,end],score=score,rows=rows)
report=dict(authority=False,parameters=parameters,
    single_training=evaluate(single,ix,16,35),single_holdout=evaluate(single,ix,35,59.54),dev2={})
dev2,di=read('output/20260909-force-step-analysis-dev2-r1/physics.npz')
for identity,domain in [(1594692888,'d1'),(-615963298,'d2')]:
    subset=dev2[field(dev2,di,'id')==identity]
    report['dev2'][domain]=dict(pre_first_emergency=evaluate(subset,di,7.585,9.709999782),
        after_first_emergency_different_inputs=evaluate(subset,di,9.709999782,24.3))
report['limitations']=[__doc__,
    'Moving intervals only; rest/gear boundaries remain independent tests.',
    'Initial physics body state is an oracle; current ROS state compatibility requires separate validation.',
    'All contact/coefficient values are fixed from training and no future contact is used.',
    'Legacy is the same seven-state physical yaw/longitudinal law evaluated at imposed tire-input boundary, not full runtime command/actuator replay.',
    'Post-Emergency input windows are explicitly separate; they cannot validate a counterfactual normal continuation.']
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(parameters,indent=2))
for name,result in [('train',report['single_training']),('holdout',report['single_holdout']),*[(d,r['pre_first_emergency']) for d,r in report['dev2'].items()]]:
    print(name,{m:{h:{k:v for k,v in stats.items() if k in ['position_error_m','longitudinal_error_mps','yaw_error_rad']} for h,stats in hs.items()} for m,hs in result['score'].items()})
