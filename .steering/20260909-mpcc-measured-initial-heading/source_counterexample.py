"""Same-source physical quaternion/declared TF and initialization counterexamples."""
from pathlib import Path
import json,math
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
out=Path('output/20260909-initial-heading-counterexample-r1');out.mkdir(exist_ok=False)
cal=Path('aichallenge/workspace/src/aichallenge_submit/racing_kart_sensor_kit_description/config/awsim_sensor_kit_calibration.yaml')
yaw=yaml.safe_load(cal.read_text())['sensor_kit_base_link']['imu_link']['yaw']
report=dict(authority=False,baseline='094941b5',declared_body_to_imu_yaw=yaw,runs={})
for name,domain,identity,end in [('single','d1',-823441338,59.54),('dev2','d1',1594692888,9.709999782),('dev2','d2',-615963298,9.709999782)]:
 raw=np.load(f'output/20260909-force-step-analysis-{name}-r1/physics.npz');ix={str(k):i for i,k in enumerate(raw['columns'])};a=raw['values'];a=a[a[:,ix['id']]==identity];time=a[:,ix['fixed']]
 rows=json.loads(Path(f'output/20260909-force-step-ros-state-{name}-r1/{domain}.json').read_text())
 old=[];corrected=[];measured=[]
 for row in rows:
  if not time[0]<=row['stamp']<end:continue
  j=np.argmin(abs(time-row['stamp']))
  if abs(time[j]-row['stamp'])>1e-7:continue
  b=R.from_quat([-a[j,ix['eq_z']],a[j,ix['eq_x']],-a[j,ix['eq_y']],a[j,ix['eq_w']]])
  if row['topic'].endswith('imu_raw'):
   i=R.from_quat([row['orientation'][c] for c in 'xyzw'])
   old.append((b.inv()*i*R.from_euler('z',-yaw)).magnitude())
   corrected.append((b.inv()*i*R.from_euler('z',-math.pi/2)).magnitude())
  if row['topic'].endswith('kinematic_state'):
   q=R.from_quat([row['orientation'][c] for c in 'xyzw']);by=b.as_euler('xyz')[2];qy=q.as_euler('xyz')[2]
   measured.append(dict(stamp=row['stamp'],body_yaw=by,odom_yaw=qy,error=math.atan2(math.sin(qy-by),math.cos(qy-by))))
 report['runs'][name+'-'+domain]=dict(frame_samples=len(old),old_frame_error_min=float(min(old)),old_frame_error_max=float(max(old)),corrected_frame_error_max=float(max(corrected)),state_rows=measured)
report['failures']=['Existing simulation IMU TF produces a pi-radian attitude error.', 'Current dev2 odometry orientation does not represent the starting vehicle heading; nearest path direction is not an observation.']
report['limitations']=['tf_static and initial_pose3d are absent from recorded topic catalog; declared TF is from the loaded simulation xacro/config path, runtime declaration must be validated by generated URDF.', 'Corrected quaternion is a diagnostic oracle comparison, not yet production code.']
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
for k,v in report['runs'].items():print(k,{p:x for p,x in v.items() if p!='state_rows'},'max_odom_yaw_error',max(abs(x['error']) for x in v['state_rows']))
assert all(v['old_frame_error_max']<1e-6 for v in report['runs'].values()), 'FAIL: declared IMU frame violates same-source body-attitude invariant'
