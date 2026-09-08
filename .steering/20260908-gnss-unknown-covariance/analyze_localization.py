"""Measure recorded covariance and sensor-reconstructed body versus EKF pose."""
import bisect
import json
import math
from pathlib import Path
import statistics
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

root = Path(sys.argv[1])
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(root / 'd1/rosbag2_autoware'), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
topics = ['/sensing/gnss/nav_sat_fix', '/sensing/gnss/pose_with_covariance',
          '/sensing/imu/imu_raw', '/localization/kinematic_state',
          '/localization/imu_gnss_poser/pose_with_covariance']
reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
data = {topic: {} for topic in topics}


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


while reader.has_next():
    topic, serialized, _ = reader.read_next()
    msg = deserialize_message(serialized, types[topic])
    stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
    if topic == topics[0]:
        row = [msg.altitude, msg.position_covariance_type]
    elif topic == topics[2]:
        q = msg.orientation
        row = [q.x, q.y, q.z, q.w]
    else:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        row = [p.x, p.y, yaw(q)]
        if topic == topics[3]:
            row += [msg.twist.twist.linear.x]
        else:
            row += [msg.pose.covariance[i] for i in (0, 7, 14)]
    data[topic][stamp] = row

odom_times = sorted(data[topics[3]])
samples = []
for stamp, pose in sorted(data[topics[1]].items()):
    if stamp not in data[topics[0]] or stamp not in data[topics[2]] or stamp < 20e9:
        continue
    index = bisect.bisect_right(odom_times, stamp)
    if not 0 < index < len(odom_times):
        continue
    t0, t1 = odom_times[index-1:index+1]
    if t1-t0 > 50000000:
        continue
    a, b = data[topics[3]][t0], data[topics[3]][t1]
    if min(a[3], b[3]) < 1.0:
        continue
    fraction = (stamp-t0)/(t1-t0)
    x, y = [a[i]+fraction*(b[i]-a[i]) for i in (0, 1)]
    qx, qy, qz, qw = data[topics[2]][stamp]
    scale = math.sqrt(.5) / math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    bx, by, bz, bw = [(qx-qy)*scale, (qx+qy)*scale, (qz-qw)*scale, (qw+qz)*scale]
    forward = [1-2*(by*by+bz*bz), 2*(bx*by+bw*bz), 2*(bx*bz-bw*by)]
    body_x = pose[0]-.26*math.cos(pose[2])+.26*forward[0]
    body_y = pose[1]-.26*math.sin(pose[2])+.26*forward[1]
    body_yaw = math.atan2(forward[1], forward[0])
    dx, dy = x-body_x, y-body_y
    lateral = -math.sin(body_yaw)*dx+math.cos(body_yaw)*dy
    samples.append(dict(source=stamp*1e-9, body_xy=[body_x, body_y],
                        body_yaw=body_yaw, ekf_xy=[x, y],
                        position_error=math.hypot(dx, dy), lateral_error=lateral,
                        gnss_altitude=data[topics[0]][stamp][0]))


def summary(values):
    values = sorted(values)
    if not values:
        return dict(count=0)
    return dict(count=len(values), mean=statistics.mean(values),
                p95=values[min(len(values)-1, math.ceil(.95*len(values))-1)],
                max=values[-1], min=values[0])


near = [s for s in samples if 89663 < s['body_xy'][0] < 89671
        and 43164 < s['body_xy'][1] < 43172]
report = dict(run_id=root.name, method=__doc__,
              limitations='GNSS/IMU reconstruction uses the audited local sensor transforms; quantization remains. EKF XY is interpolated to sensor source time. Source>20s, speed>1m/s, odom gap<=50ms.',
              covariance_types=sorted({r[1] for r in data[topics[0]].values()}),
              ekf_input_position_covariances=sorted({tuple(r[3:6]) for r in data[topics[4]].values()}),
              moving_position_error=summary([s['position_error'] for s in samples]),
              moving_absolute_lateral_error=summary([abs(s['lateral_error']) for s in samples]),
              contact_region_position_error=summary([s['position_error'] for s in near]),
              moving_gnss_altitude=summary([s['gnss_altitude'] for s in samples]),
              samples=samples)
(root / 'localization-analysis.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({k: v for k, v in report.items() if k != 'samples'}, indent=2))
