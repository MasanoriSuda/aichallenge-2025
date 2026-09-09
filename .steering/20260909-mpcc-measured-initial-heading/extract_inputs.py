"""Record complete standard ROS inputs/outputs and TF for the heading audit."""
from pathlib import Path
import hashlib
import json
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

run, out = map(Path, sys.argv[1:3])
out.mkdir(exist_ok=False)
topics = ['/sensing/gnss/nav_sat_fix', '/sensing/gnss/pose_with_covariance',
          '/sensing/imu/imu_raw', '/localization/initial_pose3d',
          '/localization/imu_gnss_poser/pose_with_covariance',
          '/localization/kinematic_state', '/tf_static']
manifest = []
for domain in sorted(p.name for p in run.iterdir() if p.name in ['d1', 'd2']):
    reader = rosbag2_py.SequentialReader()
    bag = run/domain/'rosbag2_autoware'
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    classes = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    rows = []
    while reader.has_next():
        topic, data, ns = reader.read_next()
        message = deserialize_message(data, classes[topic])
        if topic != '/tf_static':
            stamp = message.header.stamp
            if stamp.sec + stamp.nanosec*1e-9 > 10.1:
                continue
        rows.append(dict(topic=topic, receipt_ns=ns, message=message_to_ordereddict(message)))
    (out/(domain+'.json')).write_text(json.dumps(rows, indent=2)+'\n')
    manifest.append(dict(domain=domain, rows=len(rows)))
(out/'manifest.json').write_text(json.dumps(dict(run=str(run), domains=manifest,
    bags={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in run.glob('d*/rosbag2_autoware/*.mcap')}), indent=2)+'\n')
print(json.dumps(manifest), flush=True)
