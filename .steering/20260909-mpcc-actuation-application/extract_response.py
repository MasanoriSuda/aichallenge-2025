from pathlib import Path
import json,hashlib,math,sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/output/20260909-stop-viability-dev2-r1')
out=Path(sys.argv[2]) if len(sys.argv)>2 else Path('/output/20260909-actuation-response-r1')
out.mkdir(exist_ok=False)
full='--all' in sys.argv
manifest=[]
for domain in ['d1','d2']:
    bag=run/domain/'rosbag2_autoware';reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    wanted=['/vehicle/status/velocity_status','/vehicle/status/steering_status','/sensing/imu/imu_raw','/clock','/control/command/control_cmd','/localization/kinematic_state']
    classes={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in wanted};reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    rows=[]
    while reader.has_next():
        topic,data,ns=reader.read_next();m=deserialize_message(data,classes[topic])
        stamp=m.clock if topic=='/clock' else (m.header.stamp if hasattr(m,'header') else m.stamp)
        stamp_ns=stamp.sec*1000000000+stamp.nanosec;sec=stamp_ns*1e-9
        if not full and not 8.8<=sec<=10.9:continue
        r=dict(topic=topic,stamp=sec,stamp_ns=stamp_ns,receipt=ns*1e-9)
        if topic.endswith('velocity_status'):r.update(v=m.longitudinal_velocity,vy=m.lateral_velocity,yaw_rate=m.heading_rate)
        elif topic.endswith('steering_status'):r['steering']=m.steering_tire_angle
        elif topic.endswith('imu_raw'):r.update(accel=dict(x=m.linear_acceleration.x,y=m.linear_acceleration.y,z=m.linear_acceleration.z),angular=dict(x=m.angular_velocity.x,y=m.angular_velocity.y,z=m.angular_velocity.z),orientation=dict(x=m.orientation.x,y=m.orientation.y,z=m.orientation.z,w=m.orientation.w))
        elif topic.endswith('control_cmd'):r.update(speed=m.longitudinal.speed,acceleration=m.longitudinal.acceleration,wire_steering=m.lateral.steering_tire_angle)
        elif topic.endswith('kinematic_state'):r.update(v=m.twist.twist.linear.x,vy=m.twist.twist.linear.y,yaw_rate=m.twist.twist.angular.z,x=m.pose.pose.position.x,y=m.pose.pose.position.y)
        rows.append(r)
    (out/(domain+'.json')).write_text(json.dumps(rows,indent=2)+'\n')
    manifest.append(dict(domain=domain,run=str(run),rows=len(rows),bag_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in bag.glob('*.mcap')}))
(out/'manifest.json').write_text(json.dumps(dict(inputs=manifest,meaning='Raw same-run response; bag receipt is not Unity receive/apply time; future samples for scoring only; no authority'),indent=2)+'\n')
print(json.dumps(manifest),flush=True)
