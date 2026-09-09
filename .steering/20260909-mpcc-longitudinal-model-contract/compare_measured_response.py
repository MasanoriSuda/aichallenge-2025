"""Offline, measured-state-conditioned plant comparison, never MPC authority.

Fit first two laps; report later same-run intervals and independent dev2 input.
Midpoint measured steering/velocity are identification features, NOT causal
future-state forecasts. True application log supplies the external input.
"""
from pathlib import Path
import collections,hashlib,json,re,struct
import numpy as np
out=Path('output/20260909-longitudinal-response-comparison-r2');out.mkdir(exist_ok=False)

def parse(run):
 rx=[];app=[]
 for line in (run/'unity-player.log').read_text(errors='replace').splitlines():
  if not line.startswith('MPCC_INPUT_OBS '):continue
  assert not line.startswith('MPCC_INPUT_OBS error'),line
  r={k:v for k,v in re.findall(r'(\w+)=([^ ]+)',line)};r['event']=line.split()[1]
  for k in ['id','seq','source_ns','mono_ticks','ros_ns']:
   if k in r:r[k]=int(r[k])
  for k in ['input','unity','selected_input','selected_unity']:
   if k in r:r[k]=float(r[k])
  (rx if r['event']=='rx' else app).append(r)
 lookup={(r['id'],r['seq']):r for r in rx}
 assert len(lookup)==len(rx)
 for r in app:
  source=lookup[(r['id'],r['seq'])]
  assert source['source_ns']==r['source_ns'] and struct.pack('f',source['input'])==struct.pack('f',r['input'])==struct.pack('f',r['selected_input'])
  assert r['emergency']=='False' and r['unity']==r['selected_unity']
 return rx,app

def dataset(run,inputs,domain):
 data=json.loads((inputs/(domain+'.json')).read_text());rx,all_app=parse(run)
 wire={(r['stamp_ns'],struct.pack('f',r['acceleration'])) for r in data if 'wire_steering' in r}
 counts=collections.Counter(r['id'] for r in rx if (r['source_ns'],struct.pack('f',r['input'])) in wire)
 rid=counts.most_common(1)[0][0];assert counts[rid]>100
 app=[r for r in all_app if r['id']==rid];times=np.array([r['ros_ns']*1e-9 for r in app]);accel=np.array([r['input'] for r in app])
 assert np.all(np.diff(times)>0)
 velocity=[r for r in data if r['topic'].endswith('velocity_status')]
 steering={r['stamp_ns']:r['steering'] for r in data if 'steering' in r}
 rows=[];excluded=collections.Counter()
 # Vehicle.ComputeVehicleState subtracts absolute Euler angles before division.
 # Recover each principal increment BEFORE averaging; never trim whole samples.
 fixed_dt=float(np.float32(.005))
 wrap_rate=float(np.float32(np.float32(360.0)/np.float32(fixed_dt))*np.float32(np.pi/180.))
 wrapped=sum(abs(x['yaw_rate'])>wrap_rate/2 for x in velocity)
 def principal_rate(value):return value-wrap_rate*np.floor(value/wrap_rate+.5)
 for a,b in zip(velocity,velocity[1:]):
  begin=a['stamp'];end=b['stamp'];dt=end-begin
  if dt<=0 or dt>.05:excluded['source_dt']+=1;continue
  if min(a['v'],b['v'])<.5:excluded['low_speed']+=1;continue
  if a['stamp_ns'] not in steering or b['stamp_ns'] not in steering:excluded['missing_exact_steering_stamp']+=1;continue
  if begin<times[0] or end>times[-1]:excluded['outside_application_trace']+=1;continue
  cuts=[begin,*times[(times>begin)&(times<end)],end]
  integral=sum((right-left)*accel[np.searchsorted(times,left,side='right')-1] for left,right in zip(cuts,cuts[1:]))
  v=(a['v']+b['v'])/2;vy=(a['vy']+b['vy'])/2;w=(principal_rate(a['yaw_rate'])+principal_rate(b['yaw_rate']))/2
  delta=(steering[a['stamp_ns']]+steering[b['stamp_ns']])/2
  mu=(1+np.cos(delta))/2
  drive=integral/dt
  rolling_drag=(drive-.37)*mu-.03*v+w*vy
  # Sum front lateral cancellation force projected onto body x:
  # -C1*(v*sin²d-vy*sin(d)cos(d)) + C2*w*sin(d)cos(d).
  rows.append([begin,dt,(b['v']-a['v'])/dt,drive,rolling_drag,
    -(v*np.sin(delta)**2-vy*np.sin(delta)*np.cos(delta)),w*np.sin(delta)*np.cos(delta),v,vy,w,delta])
 array=np.array(rows)
 return array,dict(run=str(run),domain=domain,receiver_id=rid,wire_key_matches=dict(counts),received=sum(r['id']==rid for r in rx),applied=len(app),retained_intervals=len(rows),excluded=dict(excluded),euler_winding_samples=wrapped,euler_wrap_rate_radps=wrap_rate,bag_inputs_sha256=hashlib.sha256((inputs/(domain+'.json')).read_bytes()).hexdigest())

single,sm=dataset(Path('output/20260909-longitudinal-observed-single-r1'),Path('output/20260909-longitudinal-observed-inputs-r1'),'d1')
train=single[(single[:,0]>=15)&(single[:,0]<90)]
X=train[:,5:7];target=train[:,2]-train[:,4]
coef,_,rank,singular=np.linalg.lstsq(X,target,rcond=None)
flat=float(np.mean(train[:,2]-train[:,3]))
params=dict(training_window_sec=[15,90],samples=len(train),C1_per_sec=float(coef[0]),C2_m_per_sec=float(coef[1]),implied_front_com_distance_m=float(coef[1]/coef[0]),implied_front_sprung_mass_fraction=float(coef[0]/(.236/.005)),rank=int(rank),singular_values=singular.tolist(),flat_offset_mps2=flat)

def score(a):
 prediction={'wire_equals_net':a[:,3],'rolling_drag_component':a[:,4],
             'flat_training_offset':a[:,3]+flat,'wheel_coupling_fit':a[:,4]+a[:,5:7]@coef}
 result={}
 for name,p in prediction.items():
  err=(p-a[:,2])*a[:,1]
  result[name]=dict(intervals=len(a),velocity_increment_mae_mps=float(np.mean(np.abs(err))),velocity_increment_p95_mps=float(np.percentile(np.abs(err),95)),velocity_increment_max_mps=float(np.max(np.abs(err))),mean_acceleration_error_mps2=float(np.mean(p-a[:,2])))
 return result
report=dict(meaning=__doc__,authority=False,parameters=params,single_metadata=sm,
            training=score(train),held_out_single=score(single[(single[:,0]>=90)&(single[:,0]<250)]),dev2={})
np.save(out/'single-intervals.npy',single)
for d in ['d1','d2']:
 a,meta=dataset(Path('output/20260909-actuation-observed-dev2-r2'),Path('output/20260909-actuation-observed-response-r2'),d)
 # Pre-firstD1Emergency only, so its changed future authority is not mixed.
 subset=a[(a[:,0]>=6)&(a[:,0]+a[:,1]<=9.834999780)]
 report['dev2'][d]=dict(metadata=meta,pre_emergency=score(subset))
 np.save(out/(d+'-intervals.npy'),a)
report['limitations']=['Measured-state-conditioned system identification; future state samples cannot become a current decision input.',
 'Raw heading_rate producer directly subtracts absolute Euler angles; principal increments are recovered per sample using local200Hz float32physics step before midpoint averaging. This is an analysis correction, not raw angular truth or a production fix.',
 'First two laps train; later same-run holdout and separate dev2 are reported without refitting.',
 'Application field assignment is observed; next Unity FixedUpdate force use can lag it.',
 'RollingR=.37, drag=.03, skid=.236, fixedstep=.005 are local simulator values/continuous approximations, not2026official vehicle bounds.',
 'Fit does not supply future lateral velocity/yaw states to the current seven-state formulation; no production promotion.',
 'Low-speed/rest and source-time gaps are explicitly excluded from moving force fit and require independent coverage.']
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
