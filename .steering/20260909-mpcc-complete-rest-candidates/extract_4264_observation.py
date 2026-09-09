from pathlib import Path
import hashlib,json,math,yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
run=Path('/output/20260909-complete-stop-dev2-r1')
source=next((run/'d1/mpcc_architecture_snapshots').glob('000000004264*/snapshot.yaml'))
world=yaml.safe_load(source.read_text())['source'];pose=world['replay_world']['current_pose']
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'d1/rosbag2_autoware'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=['/localization/kinematic_state']))
matched=[]
while reader.has_next():
 _,data,ns=reader.read_next();m=deserialize_message(data,Odometry);p=m.pose.pose.position
 if abs(p.x-pose['x_m'])<1e-9 and abs(p.y-pose['y_m'])<1e-9:
  q=m.pose.pose.orientation
  matched.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,receipt=ns*1e-9,x=p.x,y=p.y,z=p.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),current_speed_mps=abs(m.twist.twist.linear.x)))
assert len(matched)==1,matched
out=Path('/output/20260909-complete-rest-4264-observation');out.mkdir(exist_ok=False)
(out/'observation.json').write_text(json.dumps(dict(world=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),matched=matched,controller_speed_source='odom.twist.twist.linear.x at mpc_controller_cpp.cpp:58291; current_time_steering0.092016and previous command age0.020are six-decimal same-decision diagnostics'),indent=2)+'\n')
print(matched)
