"""Read complete MCAP records around the pre-Recovery physical slowdown."""
import json
import math
from pathlib import Path
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

root = Path(sys.argv[1])
uri = str(root / 'd1/rosbag2_autoware')
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
topics = ['/control/command/control_cmd', '/localization/kinematic_state',
          '/vehicle/status/velocity_status', '/sensing/imu/imu_raw',
          '/sensing/gnss/pose_with_covariance', '/vehicle/status/steering_status']
reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
rows = {t: [] for t in topics}

def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

while reader.has_next():
    topic, data, receive_ns = reader.read_next()
    msg = deserialize_message(data, types[topic])
    stamp = msg.header.stamp if hasattr(msg, 'header') else msg.stamp
    t = seconds(stamp)
    if not 244 <= t <= 256:
        continue
    prefix = [t, receive_ns * 1e-9]
    if topic == topics[0]:
        values = [msg.longitudinal.speed, msg.longitudinal.acceleration, msg.lateral.steering_tire_angle]
    elif topic == topics[1]:
        p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values = [p.x, p.y, p.z, v.x, v.y, v.z, roll, pitch, yaw, w.z]
    elif topic == topics[2]:
        values = [msg.longitudinal_velocity, msg.lateral_velocity, msg.heading_rate]
    elif topic == topics[3]:
        a, w = msg.linear_acceleration, msg.angular_velocity
        values = [a.x, a.y, a.z, w.x, w.y, w.z]
    elif topic == topics[5]:
        values = [msg.steering_tire_angle]
    else:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values = [p.x, p.y, p.z, roll, pitch, yaw]
    rows[topic].append(prefix + values)
result = dict(uri=uri, source_window_sec=[244, 256],
              columns={topics[0]: ['source', 'receive', 'speed', 'acceleration', 'wire_steering'],
                       topics[1]: ['source', 'receive', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'yaw_rate'],
                       topics[2]: ['source', 'receive', 'vx', 'vy', 'yaw_rate'],
                       topics[3]: ['source', 'receive', 'ax', 'ay', 'az', 'wx', 'wy', 'wz'],
                       topics[4]: ['source', 'receive', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'],
                       topics[5]: ['source', 'receive', 'measured_steering_rad']}, rows=rows)
(root / 'delay-prefix-motion.json').write_text(json.dumps(result, indent=2) + '\n')
print({topic: len(values) for topic, values in rows.items()})
for t in (247, 248, 248.5, 249, 249.25, 249.5, 249.75, 250, 251, 252):
    print('t', t)
    for topic in topics[:3]:
        row = min(rows[topic], key=lambda row: abs(row[0] - t))
        print(topic, [round(v, 5) for v in row])
