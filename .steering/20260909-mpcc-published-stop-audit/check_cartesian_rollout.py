"""Independent Cartesian ODE of recorded controls; no QP solve or Frenet dynamics."""
from pathlib import Path
import hashlib,json,math
import numpy as np
from scipy.integrate import solve_ivp
import yaml
run=Path('output/20260909-published-stop-observation-dev2-r1')
source=next((run/'d2/mpcc_architecture_snapshots').glob('000000000453*/snapshot.yaml'))
evidence=yaml.safe_load(source.read_text())['execution_evidence']
report=yaml.safe_load((run/'executed-stop-replay/report.yaml').read_text())
points=report['points']
first=points[0]; initial=evidence['semantic_initial_state']
state=np.array([first['x_m'],first['y_m'],first['yaw_rad'],initial['velocity_mps'],initial['steering_rad'],initial['response_steering_rad']])
time=0.;segments=[]
for u in evidence['control_stages']:
 end=time+u['duration_sec']
 def rhs(t,x):
  speed=max(0.,x[3]); a=u['acceleration_mps2']
  return [speed*math.cos(x[2]),speed*math.sin(x[2]),evidence['yaw_response_gain']*speed*math.tan(x[5])/evidence['wheelbase_m'],a if speed>0 or a>0 else 0.,u['steering_rate_radps'],(x[4]-x[5])/evidence['yaw_response_time_constant_sec']]
 result=solve_ivp(rhs,[time,end],state,method='DOP853',rtol=1e-11,atol=1e-10,max_step=.001,dense_output=True)
 assert result.success
 segments.append((time,end,result.sol));state=result.y[:,-1];time=end
errors=[]
for p in points:
 t=p['elapsed_sec'];sol=next(s for begin,end,s in segments if begin-1e-9<=t<=end+1e-9);x=sol(t)
 errors.append(dict(elapsed_sec=t,position_error_m=math.hypot(p['x_m']-x[0],p['y_m']-x[1]),yaw_error_rad=math.atan2(math.sin(p['yaw_rad']-x[2]),math.cos(p['yaw_rad']-x[2])),speed_error_mps=p['velocity_mps']-max(0.,x[3])))
out=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_sequence=evidence['source_sequence'],solver_invocations=0,independent_model='Cartesian x/y/yaw/v/delta/rho ODE, recorded a/rate only; nu has no physical velocity',samples=len(errors),maximum_position_error_m=max(p['position_error_m'] for p in errors),maximum_yaw_error_rad=max(abs(p['yaw_error_rad']) for p in errors),maximum_speed_error_mps=max(abs(p['speed_error_mps']) for p in errors),errors=errors)
(run/'executed-stop-replay/cartesian-oracle.json').write_text(json.dumps(out,indent=2)+'\n')
print({k:v for k,v in out.items() if k!='errors'})
assert out['maximum_position_error_m']<.001
assert out['maximum_yaw_error_rad']<.001
