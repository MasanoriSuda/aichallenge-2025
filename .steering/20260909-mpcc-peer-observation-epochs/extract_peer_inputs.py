"""Decode same-run inputs only; D2 odometry is an estimate, not ground truth."""
from pathlib import Path
import hashlib, json, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

run = Path('/output/20260909-publication-clock-dev2-r1')
out = Path('/output/20260909-peer-epochs-inputs-r1')
out.mkdir(exist_ok=False)
manifest = []
for domain in ['d1', 'd2']:
    bag = run/domain/'rosbag2_autoware'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    wanted = ['/v2x/vehicle_positions', '/localization/kinematic_state', '/control/command/control_cmd']
    classes = {t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in wanted}
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    rows = []
    while reader.has_next():
        topic, data, ns = reader.read_next()
        m = deserialize_message(data, classes[topic])
        stamp = m.header.stamp if hasattr(m, 'header') else m.stamp
        sec = stamp.sec + stamp.nanosec*1e-9
        if not 0 <= sec <= 10.1:
            continue
        row = dict(topic=topic, stamp=sec, receipt=ns*1e-9)
        if topic.endswith('vehicle_positions'):
            row['frame'] = m.header.frame_id
            row['vehicles'] = [dict(id=v.vehicle_id, stamp=v.header.stamp.sec+v.header.stamp.nanosec*1e-9,
                frame=v.header.frame_id, x=v.position.x, y=v.position.y, z=v.position.z,
                stddev_x=v.covariance.x, stddev_y=v.covariance.y) for v in m.vehicles]
        elif topic.endswith('kinematic_state'):
            p=m.pose.pose.position; q=m.pose.pose.orientation
            row.update(x=p.x, y=p.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                v=m.twist.twist.linear.x, yaw_rate=m.twist.twist.angular.z)
        else:
            row.update(speed=m.longitudinal.speed, acceleration=m.longitudinal.acceleration,
                wire_steering=m.lateral.steering_tire_angle)
        rows.append(row)
    (out/(domain+'.json')).write_text(json.dumps(rows, indent=2)+'\n')
    manifest.append(dict(domain=domain, run=str(run), rows=len(rows),
        bag_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in bag.glob('*.mcap')}))
(out/'manifest.json').write_text(json.dumps(dict(inputs=manifest,
    limits='Bag receipt is not controller receipt; D2 Odometry is estimated and no V2X body heading exists. No authority.'), indent=2)+'\n')
print(json.dumps(manifest), flush=True)
