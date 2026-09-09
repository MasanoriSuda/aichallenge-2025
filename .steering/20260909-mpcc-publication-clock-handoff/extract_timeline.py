"""Read-only same-run ROS records; receipt times are bag receipt, not callback steady time."""
from pathlib import Path
import hashlib,json,math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

run=Path('/output/20260909-ego-viability-dev2-r1')
out=Path('/output/20260909-publication-clock-timeline');out.mkdir(exist_ok=False)
reader=rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(run/'d1/rosbag2_autoware'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
topics=['/localization/kinematic_state','/control/command/control_cmd','/vehicle/status/steering_status']
classes={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
rows=[]
while reader.has_next():
    topic,data,ns=reader.read_next()
    m=deserialize_message(data,classes[topic]);stamp=m.header.stamp if hasattr(m,'header') else m.stamp
    sec=stamp.sec+stamp.nanosec*1e-9
    if not 9.5<=sec<=10.2:continue
    row=dict(topic=topic,stamp=sec,receipt=ns*1e-9)
    if topic.endswith('kinematic_state'):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        row.update(x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),v=m.twist.twist.linear.x,yaw_rate=m.twist.twist.angular.z)
    elif topic.endswith('control_cmd'):
        row.update(speed=m.longitudinal.speed,acceleration=m.longitudinal.acceleration,wire_steering=m.lateral.steering_tire_angle,wire_steering_rate=m.lateral.steering_tire_rotation_rate)
    else:row['measured_steering']=m.steering_tire_angle
    rows.append(row)
(out/'timeline.json').write_text(json.dumps(rows,indent=2)+'\n')
(out/'manifest.json').write_text(json.dumps(dict(run=str(run),domain=1,topics=topics,rows=len(rows),bag_sha256=hashlib.sha256((run/'d1/rosbag2_autoware/rosbag2_autoware_0.mcap').read_bytes()).hexdigest(),limits='No publisher solution ID in control_cmd. Bag receipt cannot recover controller steady receipt age exactly. No claim that selection implies publication.'),indent=2)+'\n')
for row in rows:
    if 9.79<=row['stamp']<=10.01:print(json.dumps(row),flush=True)
