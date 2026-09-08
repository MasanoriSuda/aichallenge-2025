"""Read immutable diagnostic bags; keep receive and source clocks separate."""

import json
from pathlib import Path
import statistics
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quantile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


root = Path(sys.argv[1])
result = {}
for domain in (1, 2):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(root / f"d{domain}" / "rosbag2_autoware"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    selected = ["/control/command/control_cmd", "/localization/kinematic_state", "/v2x/vehicle_positions"]
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    controls, ego, peers = [], [], []
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == selected[0]:
            controls.append({"source_sec": seconds(msg.stamp), "receive_sec": receive_ns * 1e-9,
                             "speed_mps": msg.longitudinal.speed, "acceleration_mps2": msg.longitudinal.acceleration,
                             "steering_wire_rad": msg.lateral.steering_tire_angle})
        elif topic == selected[1]:
            if 29 <= seconds(msg.header.stamp) <= 37:
                ego.append({"source_sec": seconds(msg.header.stamp), "receive_sec": receive_ns * 1e-9,
                            "x_m": msg.pose.pose.position.x, "y_m": msg.pose.pose.position.y,
                            "forward_speed_mps": msg.twist.twist.linear.x})
        else:
            for vehicle in msg.vehicles:
                if 29 <= seconds(vehicle.header.stamp) <= 37:
                    peers.append({"source_sec": seconds(vehicle.header.stamp), "receive_sec": receive_ns * 1e-9,
                                  "id": vehicle.vehicle_id, "x_m": vehicle.position.x, "y_m": vehicle.position.y})
    stats = {}
    for clock in ("source_sec", "receive_sec"):
        gaps = [1000 * (b[clock] - a[clock]) for a, b in zip(controls, controls[1:])]
        stats[clock] = {"count": len(controls), "duration_sec": controls[-1][clock] - controls[0][clock],
                        "gap_ms_mean": statistics.mean(gaps), "gap_ms_p95": quantile(gaps, .95),
                        "gap_ms_p99": quantile(gaps, .99), "gap_ms_max": max(gaps),
                        "nonpositive_gaps": sum(gap <= 0 for gap in gaps),
                        "gaps_over_50_ms": sum(gap > 50 for gap in gaps)}
    result[f"d{domain}"] = {"control_stats": stats, "ego_29_37s": ego, "peers_29_37s": peers,
                            "control_29_37s": [c for c in controls if 29 <= c["source_sec"] <= 37]}
destination = root / "bag-analysis.json"
destination.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({domain: data["control_stats"] for domain, data in result.items()}, indent=2))
