"""Identify four physical effective axle weights on frozen16..35s training only.

Future body states are supervised targets, never input features. Initial body and
prescribed actual wire/tire sequence are diagnostic plant-identification inputs.
Public initialization/application is separately validated after freezing weights.
No parameter is optimized on the previously inspected holdout or dev2 failure.
"""
from pathlib import Path
import importlib.util
import json
import math
import numpy as np
from scipy.optimize import least_squares

spec=importlib.util.spec_from_file_location('base',Path(__file__).with_name('compare_public_inputs.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
P=base.P
RAW=np.load('output/20260909-force-step-analysis-single-r1/physics.npz')
IX={str(k):i for i,k in enumerate(RAW['columns'])};A=RAW['values']
TIME,TRUTH,DELTA=base.read_truth('single',-823441338)
WIRE=A[:,IX['wire']];DT=float(A[0,IX['dt']])
FULL_CORNER=np.array(P['wheel_cornering_per_sec'])/(4*np.array(P['wheel_traction_fraction']))
INITIAL=np.array([4*P['wheel_traction_fraction'][0],4*P['wheel_traction_fraction'][2],
                  4*P['wheel_traction_fraction'][0],4*P['wheel_traction_fraction'][2]])
START=np.array([j for j in range(0,len(A)-100,40) if 16<=TIME[j] and TIME[j+100]<35 and TRUTH[j,3]>=.5])
TARGET=np.stack([TRUTH[START+h]-np.column_stack([TRUTH[START,:2],np.zeros((len(START),4))]) for h in [20,50,100]])


def derivative(s,wire,tire,weights):
    yaw,u,vy,r=s[:,2],s[:,3],s[:,4],s[:,5]
    acceleration=np.where(u<0,np.maximum(wire,-u/.01),wire)
    rolling=-np.sign(u)*np.minimum(.37,abs(u)/.01)
    fx=np.zeros(len(s));fy=fx.copy();torque=fx.copy()
    for n,(wx,wy) in enumerate(P['wheel_com_xy_m']):
        angle=tire if n<2 else np.zeros(len(s));c=np.cos(angle);sn=np.sin(angle)
        side=-sn*(u-r*wy)+c*(vy+r*wx)
        lateral=-FULL_CORNER[n]*weights[2+n//2]*side
        drive=.25*weights[n//2]*(acceleration+rolling)
        ax=c*drive-sn*lateral;ay=sn*drive+c*lateral
        fx+=ax;fy+=ay;torque+=wx*ay-wy*ax
    refvy=vy-P['com_x_m']*r
    return np.column_stack([u*np.cos(yaw)-refvy*np.sin(yaw),u*np.sin(yaw)+refvy*np.cos(yaw),r,
                            fx-.03*u+r*vy,fy-.03*vy-r*u,torque*P['mass_kg']/P['yaw_inertia_kgm2']-r])


def rollout(weights,start):
    s=TRUTH[start].copy();s[:,:2]=0;values=[]
    for k in range(100):
        wire=WIRE[start+k];tire=DELTA[start+k]
        d=derivative(s,wire,tire,weights)
        s+=DT*derivative(s+.5*DT*d,wire,tire,weights)
        if k+1 in [20,50,100]:values.append(s.copy())
    return np.stack(values)


def loss(weights):
    error=rollout(weights,START)-TARGET
    error[:,:,2]=np.arctan2(np.sin(error[:,:,2]),np.cos(error[:,:,2]))
    # Metre-equivalent physical displacement objective: yaw at front footprint,
    # velocity at each horizon. Weights are declared once, before identification.
    for j,h in enumerate([20,50,100]):
        error[j,:,2]*=1.615
        error[j,:,3:5]*=h*DT
        error[j,:,5]*=1.615*h*DT
    return error.ravel()


OUT=Path('output/20260910-shared-plant-body-identification-r1');OUT.mkdir(exist_ok=False)
result=least_squares(loss,INITIAL,bounds=(np.zeros(4),np.ones(4)),max_nfev=60,
                     ftol=1e-10,xtol=1e-10,gtol=1e-10)
parameters=dict(P)
parameters['wheel_traction_fraction']=[result.x[0]/4]*2+[result.x[1]/4]*2
parameters['wheel_cornering_per_sec']=(FULL_CORNER*np.repeat(result.x[2:],2)).tolist()
parameters['meaning']='Four effective drive/cornering axle weights identified only on single16..35s using causal rollouts and future truth solely as targets.0..1physical parameter bounds; no production promotion.'
report=dict(authority=False,meaning=__doc__,training_anchors=len(START),training_source_sec=[16,35],horizons_sec=[20*DT,50*DT,100*DT],
 initial_weights=INITIAL.tolist(),identified_weights=result.x.tolist(),initial_residual_rms=float(np.sqrt(np.mean(loss(INITIAL)**2))),
 identified_residual_rms=float(np.sqrt(np.mean(loss(result.x)**2))),solver_success=bool(result.success),solver_message=result.message,
 function_evaluations=int(result.nfev),parameters=parameters)
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
# Weights now frozen. Score the complete, unchanged held-out public-input set.
base.GEOMETRY=parameters['wheel_com_xy_m'];base.TRACTION=parameters['wheel_traction_fraction'];base.CORNER=parameters['wheel_cornering_per_sec']
heldout=dict(authority=False,parameters=parameters,single_holdout=base.evaluate('single','d1',-823441338,35,59.54),
 dev2={d:base.evaluate('dev2',d,i,7.585,9.709999782) for d,i in [('d1',1594692888),('d2',-615963298)]})
(OUT/'heldout.json').write_text(json.dumps(heldout,indent=2)+'\n')
for name,value in [('single',heldout['single_holdout']),*heldout['dev2'].items()]:
 print(name,{h:{k:(v.get('mae'),v.get('maximum')) for k,v in fields.items() if k in ['position_error_m','speed_error_mps','yaw_error_rad']} for h,fields in value['score']['prescribed_publications']['dynamic_yaw'].items()},flush=True)
