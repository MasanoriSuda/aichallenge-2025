"""Post-run public-topic timing and observed control-origin prediction errors."""
from pathlib import Path
import bisect
import hashlib
import json
import math
import re
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

run, out = map(Path, sys.argv[1:3])
assert (run/'monitor-summary.json').exists(), 'Requires completed teardown'
out.mkdir(exist_ok=False)
topics = ['/localization/kinematic_state', '/vehicle/status/velocity_status',
 '/sensing/imu/imu_raw', '/vehicle/status/steering_status', '/control/command/control_cmd',
 '/v2x/vehicle_positions', '/planning/scenario_planning/trajectory']
def summary(values):
 a=np.asarray(values,dtype=float)
 if not len(a): return {'count':0}
 return dict(count=len(a),mean=float(np.mean(a)),mae=float(np.mean(np.abs(a))),
  p95_abs=float(np.percentile(np.abs(a),95)),p99_abs=float(np.percentile(np.abs(a),99)),maximum_abs=float(np.max(np.abs(a))))
def stamp(m):
 x=getattr(getattr(m,'header',None),'stamp',getattr(m,'stamp',None))
 return None if x is None else x.sec+x.nanosec*1e-9
report={'run':str(run),'commit':json.loads((run/'manifest.json').read_text())['baseline_commit'],
 'meaning':'Completed run only. Topic source and bag receipt are distinct. Prediction scores linearly interpolate surrounding public observations at the logged control origin; no extrapolation or future data used by controller. Includes startup/finish unless stated. Error is empirical, not a guaranteed bound.', 'domains':{}}
for domain in sorted(p.name for p in run.iterdir() if re.fullmatch('d[1-4]',p.name)):
 reader=rosbag2_py.SequentialReader();bag=run/domain/'rosbag2_autoware'
 reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 classes={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
 reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
 times={t:[] for t in classes}; states={t:[] for t in classes};invalid_commands=0
 while reader.has_next():
  topic,data,ns=reader.read_next();m=deserialize_message(data,classes[topic]);t=stamp(m);times[topic].append((t,ns*1e-9))
  value=None
  if topic=='/localization/kinematic_state':
   p=m.pose.pose.position;q=m.pose.pose.orientation
   value=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
  elif topic=='/vehicle/status/velocity_status':value=[m.longitudinal_velocity,m.lateral_velocity]
  elif topic=='/sensing/imu/imu_raw':value=[m.angular_velocity.z]
  elif topic=='/vehicle/status/steering_status':value=[m.steering_tire_angle]
  elif topic=='/control/command/control_cmd':
   value=[m.longitudinal.speed,m.longitudinal.acceleration,m.lateral.steering_tire_angle]
   invalid_commands+=not all(math.isfinite(v) for v in value)
  if value is not None and t is not None:states[topic].append((t,value))
 result={'topics':{},'invalid_commands':invalid_commands}
 for topic,rows in times.items():
  entry={'count':len(rows)}
  for index,label in [(0,'source'),(1,'bag_receipt')]:
   ts=[r[index] for r in rows if r[index] is not None];dt=np.diff(ts)
   entry[label]={'span_sec':ts[-1]-ts[0] if len(ts)>1 else None,'hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None,
    'duplicate_intervals':int(sum(dt==0)),'backward_intervals':int(sum(dt<0)),'interval_ms':summary(dt*1000)}
  result['topics'][topic]=entry
 for topic in states:states[topic]=sorted(dict(states[topic]).items())
 state_times={topic:[x[0] for x in rows] for topic,rows in states.items()}
 def interpolate(topic,t):
  rows=states.get(topic,[]);ts=state_times.get(topic,[]);i=bisect.bisect_left(ts,t)
  if i<len(rows) and ts[i]==t:return rows[i][1]
  if i==0 or i==len(rows) or ts[i]-ts[i-1]>.5:return None
  f=(t-ts[i-1])/(ts[i]-ts[i-1]);a=np.array(rows[i-1][1]);delta=np.array(rows[i][1])-a
  if topic=='/localization/kinematic_state':delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
  return a+f*delta
 log=re.sub(r'\x1b\[[0-9;]*m','',(run/domain/'autoware.log').read_text(errors='replace'))
 errors=[];samples=[]
 for line in log.splitlines():
  if 'MPCC body prediction:' not in line:continue
  d={k:float(v) for k,v in re.findall(r'(\w+)=(-?[0-9]+(?:\.[0-9]+)?)',line.split('MPCC body prediction:',1)[1])}
  pose=interpolate('/localization/kinematic_state',d['origin']);vel=interpolate('/vehicle/status/velocity_status',d['origin']);yaw=interpolate('/sensing/imu/imu_raw',d['origin']);tire=interpolate('/vehicle/status/steering_status',d['origin'])
  if any(x is None for x in [pose,vel,yaw,tire]):continue
  row=dict(origin=d['origin'],speed=d['source_u'],position_m=math.hypot(d['x']-pose[0],d['y']-pose[1]),yaw_rad=math.atan2(math.sin(d['yaw']-pose[2]),math.cos(d['yaw']-pose[2])),u_mps=d['u']-vel[0],vy_mps=d['vy']-vel[1],yaw_rate_radps=d['r']-yaw[0],tire_rad=d['tire']-tire[0]);samples.append(row)
 for label,selected in [('all',samples),('moving',[r for r in samples if r['speed']>.1])]:
  result['prediction_'+label]={key:summary([r[key] for r in selected]) for key in ['position_m','yaw_rad','u_mps','vy_mps','yaw_rate_radps','tire_rad']}
 cycles=re.findall(r'Control callback runtime: cycles=(\d+), elapsed_ms=([\d.]+)/([\d.]+)\(avg/max\).*?overruns=(\d+)',log)
 result['callback']={'cycles':sum(int(r[0]) for r in cycles),'weighted_mean_ms':sum(int(r[0])*float(r[1]) for r in cycles)/sum(int(r[0]) for r in cycles) if cycles else None,'maximum_ms':max((float(r[2]) for r in cycles),default=None),'overruns':sum(int(r[3]) for r in cycles),'overrun_windows':[r for r in cycles if int(r[3])],'per_cycle_p95_p99':'not recorded; do not substitute percentiles of window means'}
 result['actuation_join_rejections']=sum(int(x) for x in re.findall(r'production actuation join: joined=\d+, rejected=(\d+)',log))
 report['domains'][domain]=result
 (out/(domain+'-prediction-samples.json')).write_text(json.dumps(samples,indent=2)+'\n')
report['bags']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in run.glob('d*/rosbag2_autoware/*.mcap')}
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
for domain,row in report['domains'].items():print(json.dumps({'domain':domain,'callback':row['callback'],'moving_position':row['prediction_moving']['position_m'],'commands':row['topics'].get('/control/command/control_cmd'),'invalid_commands':row['invalid_commands'],'join_rejections':row['actuation_join_rejections']}),flush=True)
