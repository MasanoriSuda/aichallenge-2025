"""Same-run receive timing and the single-vehicle terminal boundary."""

import json
from pathlib import Path
import statistics
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quantile(values, fraction):
    ordered = sorted(values)
    i = (len(ordered) - 1) * fraction
    lo = int(i)
    return ordered[lo] + (ordered[min(lo + 1, len(ordered) - 1)] - ordered[lo]) * (i - lo)


root = Path(sys.argv[1])
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(root / "d1/rosbag2_autoware"), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
topics = ["/control/command/control_cmd", "/localization/kinematic_state"]
reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
controls, motion = [], []
while reader.has_next():
    topic, data, receive_ns = reader.read_next()
    msg = deserialize_message(data, types[topic])
    if topic == topics[0]:
        controls.append([stamp_seconds(msg.stamp), receive_ns * 1e-9,
                         msg.longitudinal.speed, msg.longitudinal.acceleration,
                         msg.lateral.steering_tire_angle])
    elif 258 <= stamp_seconds(msg.header.stamp) <= 273:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        motion.append([stamp_seconds(msg.header.stamp), receive_ns * 1e-9,
                       p.x, p.y, msg.twist.twist.linear.x, q.x, q.y, q.z, q.w])
stats = {}
for clock, index in [("source_sec", 0), ("receive_sec", 1)]:
    gaps = [1000 * (b[index] - a[index]) for a, b in zip(controls, controls[1:])]
    stats[clock] = {"count": len(controls), "duration_sec": controls[-1][index] - controls[0][index],
                    "gap_ms_mean": statistics.mean(gaps), "gap_ms_p95": quantile(gaps, .95),
                    "gap_ms_p99": quantile(gaps, .99), "gap_ms_max": max(gaps),
                    "nonpositive_gaps": sum(x <= 0 for x in gaps),
                    "gaps_over_50_ms": sum(x > 50 for x in gaps)}
result = {"control_stats": stats, "control_columns": ["source_s", "receive_s", "speed_mps", "accel_mps2", "wire_steering_rad"],
          "controls_258_273s": [c for c in controls if 258 <= c[0] <= 273],
          "motion_columns": ["source_s", "receive_s", "x", "y", "odom_speed_mps", "qx", "qy", "qz", "qw"],
          "motion_258_273s": motion}
(root / "bag-analysis.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(stats, indent=2))
