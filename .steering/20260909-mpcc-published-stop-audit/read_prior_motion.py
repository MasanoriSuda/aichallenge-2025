"""Read Domain2 motion around the previously missed published Stop loss."""
import json
import math
from pathlib import Path
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

root = Path(sys.argv[1])
window_start = float(sys.argv[2]) if len(sys.argv) > 2 else 240.0
window_end = float(sys.argv[3]) if len(sys.argv) > 3 else 270.0
uri = str(root / 'd2/rosbag2_autoware')
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
topics = ['/control/command/control_cmd', '/localization/kinematic_state',
          '/vehicle/status/velocity_status', '/sensing/imu/imu_raw',
          '/sensing/gnss/pose_with_covariance', '/vehicle/status/steering_status',
          '/localization/acceleration']
reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
rows = {t: [] for t in topics}

def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

while reader.has_next():
    topic, data, receive_ns = reader.read_next()
    msg = deserialize_message(data, types[topic])
    stamp = msg.header.stamp if hasattr(msg, 'header') else msg.stamp
    t = seconds(stamp)
    if not window_start <= t <= window_end:
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
    elif topic == topics[6]:
        values = [msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z]
    else:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values = [p.x, p.y, p.z, roll, pitch, yaw]
    rows[topic].append(prefix + values)
result = dict(uri=uri, source_window_sec=[window_start, window_end],
              columns={topics[0]: ['source', 'receive', 'speed', 'acceleration', 'wire_steering'],
                       topics[1]: ['source', 'receive', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'yaw_rate'],
                       topics[2]: ['source', 'receive', 'vx', 'vy', 'yaw_rate'],
                       topics[3]: ['source', 'receive', 'ax', 'ay', 'az', 'wx', 'wy', 'wz'],
                       topics[4]: ['source', 'receive', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'],
                       topics[5]: ['source', 'receive', 'measured_steering_rad'],
                       topics[6]: ['source','receive','ax','ay','az']}, rows=rows)
(root / 'd2-published-stop-motion.json').write_text(json.dumps(result, indent=2) + '\n')
print({topic: len(values) for topic, values in rows.items()})
