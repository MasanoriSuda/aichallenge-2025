"""Extract completed-run public input and command chronology around the r10 loss."""
from pathlib import Path
import json
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

run, out = map(Path, sys.argv[1:3])
assert (run / 'monitor-summary.json').exists()
out.mkdir(exist_ok=False)
topics = ['/clock', '/vehicle/status/velocity_status', '/sensing/imu/imu_raw',
          '/vehicle/status/steering_status', '/control/command/control_cmd']
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(run / 'd1/rosbag2_autoware'), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
classes = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
rows = []
while reader.has_next():
    topic, data, ns = reader.read_next()
    m = deserialize_message(data, classes[topic])
    stamp = getattr(getattr(m, 'header', None), 'stamp', getattr(m, 'stamp', getattr(m, 'clock', None)))
    t = None if stamp is None else stamp.sec + stamp.nanosec * 1e-9
    if t is not None and 28.1 <= t <= 28.65:
        rows.append(dict(topic=topic, source_sec=t, bag_receipt_sec=ns * 1e-9, message=message_to_ordereddict(m)))
(out / 'd1-public-interval.json').write_text(json.dumps(rows, indent=2) + '\n')
print(json.dumps({'topics': list(classes), 'rows': len(rows), 'run': str(run)}))
