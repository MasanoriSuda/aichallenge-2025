"""Score causal velocity predictors against later reports; no future inputs."""
from pathlib import Path
import bisect,json,sys
import numpy as np
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path('output/20260909-published-stop-observation-dev2-r1')
domain=sys.argv[2] if len(sys.argv)>2 else '2'
m=json.load(open(root/('d'+domain+'-longitudinal-full-motion.json')))['rows']
commands=m['/control/command/control_cmd'];odom=m['/localization/kinematic_state'];acc=m['/localization/acceleration'];truth=m['/vehicle/status/velocity_status']
rows=[]
filtered_command_acceleration=0.; previous_observation=None
acc_by_time={r[0]:r for r in acc}
commands=sorted(commands,key=lambda r:(r[0],r[1]));command_times=[c[0] for c in commands]
truth_times=[r[0] for r in truth];truth_values=[r[2] for r in truth]
for o in odom:
 t,receipt,v=o[0],o[1],o[5];end=t+.13
 if previous_observation is None: previous_observation=t
 begin=previous_observation
 if t>begin:
  active_index=bisect.bisect_right(command_times,begin-.13)-1
  command_a=commands[active_index][3] if active_index>=0 else 0.
  integral=0.;last=begin
  for c in commands[max(0,active_index+1):bisect.bisect_right(command_times,t-.13)]:
   event=c[0]+.13
   integral+=command_a*(event-last);command_a=c[3];last=event
  integral+=command_a*(t-last)
  # Exact configured gain; no fitting. Model the same derivative+LP operator.
  filtered_command_acceleration=.9*filtered_command_acceleration+.1*integral/(t-begin)
 elif t==begin:
  filtered_command_acceleration*=.9
 previous_observation=t
 arow=[acc_by_time[t]] if t in acc_by_time else []
 if not arow or end>truth[-1][0]:continue
 observed=arow[-1][2];receipt=max(receipt,arow[-1][1])
 # At the recorded control callback, odometry and acceleration have arrived.
 # Use only commands serialized by the observation's source time and reception.
 known=[c for c in commands[max(0,bisect.bisect_left(command_times,t-.13)-1):bisect.bisect_left(command_times,t)] if c[1]<=receipt]
 if not known:continue
 active=[c for c in known if c[0]+.13<=t];events=[c for c in known if t<c[0]+.13<end]
 if not active:continue
 reference=active[-1][3]
 models={'observed_hold':observed,'absolute_committed':reference,'observed_plus_command_delta':observed,'matched_input_filter':reference+observed-filtered_command_acceleration}
 speed={k:v for k in models};start=t
 for c in events+[None]:
  stop=c[0]+.13 if c else end
  for k,a in models.items():speed[k]=max(0.,speed[k]+a*(stop-start))
  if c:
   models['absolute_committed']=c[3];models['observed_plus_command_delta']=observed+c[3]-reference;models['matched_input_filter']=c[3]+observed-filtered_command_acceleration
  start=stop
 truth_v=float(np.interp(end,truth_times,truth_values))
 rows.append(dict(observation_sec=t,control_origin_sec=end,observed_speed_mps=v,observed_acceleration_mps2=observed,active_command_acceleration_mps2=reference,filtered_command_acceleration_mps2=filtered_command_acceleration,known_command_count=len(known),interpolated_future_velocity_mps=truth_v,predictions=speed,errors={k:s-truth_v for k,s in speed.items()}))
summaries={}
for name,predicate in [('all',lambda r:True),('moving',lambda r:r['observed_speed_mps']>.1),('moving_positive_command',lambda r:r['observed_speed_mps']>.1 and r['active_command_acceleration_mps2']>.1),('moving_negative_command',lambda r:r['observed_speed_mps']>.1 and r['active_command_acceleration_mps2']<-.1),('before_stop',lambda r:r['observation_sec']<11.63),('brake_transition',lambda r:11.63<=r['observation_sec']<=11.795),('after_transition',lambda r:r['observation_sec']>11.795),('positive_applied_command',lambda r:r['active_command_acceleration_mps2']>.1),('negative_applied_command',lambda r:r['active_command_acceleration_mps2']<-.1),('near_zero_applied_command',lambda r:abs(r['active_command_acceleration_mps2'])<=.1)]:
 group=[r for r in rows if predicate(r)]
 summaries[name]={k:dict(samples=len(group),mae_mps=float(np.mean([abs(r['errors'][k]) for r in group])) if group else None,maximum_absolute_error_mps=max([abs(r['errors'][k]) for r in group],default=None)) for k in ['observed_hold','absolute_committed','observed_plus_command_delta','matched_input_filter']}
out=dict(scope='recorded observations, same 0.13s declared origins; future status interpolation only scores outputs; no simulator-phase fit',summaries=summaries,rows=rows)
(root/('d'+domain+'-velocity-model-comparison.json')).write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(summaries,indent=2))
