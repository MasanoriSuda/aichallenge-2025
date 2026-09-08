"""Extract source-clock localization and V2X positions without changing bags."""
import json
from pathlib import Path
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

root = Path(sys.argv[1])
result = {}
for domain in ("d1", "d2"):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root / domain / "rosbag2_autoware"), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    topics = ["/localization/kinematic_state", "/v2x/vehicle_positions", "/control/command/control_cmd"]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    own, peers, controls = [], [], []
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        m = deserialize_message(data, types[topic])
        if topic == topics[0] and 15 <= sec(m.header.stamp) <= 75:
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            own.append([sec(m.header.stamp), p.x, p.y, m.twist.twist.linear.x, q.x, q.y, q.z, q.w])
        elif topic == topics[1]:
            for v in m.vehicles:
                if 15 <= sec(v.header.stamp) <= 75:
                    peers.append([v.vehicle_id, sec(v.header.stamp), v.position.x, v.position.y])
        elif topic == topics[2] and 15 <= sec(m.stamp) <= 75:
            controls.append([sec(m.stamp), m.longitudinal.speed, m.longitudinal.acceleration, m.lateral.steering_tire_angle])
    result[domain] = {"own": own, "peers": peers, "controls": controls}
(root / "motion.json").write_text(json.dumps(result, separators=(",", ":")) + "\n")
print({d: {k: len(v) for k, v in rows.items()} for d, rows in result.items()})
