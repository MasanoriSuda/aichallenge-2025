"""Locate source-stamp anomalies in the same run, without conflating receipt time."""
import bisect
import json
from pathlib import Path
import re
import sys
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

root = Path(sys.argv[1])
transitions = []
for line in (root/'d1/autoware.log').read_text(errors='replace').splitlines():
    match = re.search(r'\[(\d+\.\d+)\].*AWSIM vehicle state observed: state=(\S+)', line)
    if match:
        transitions.append((float(match[1]), match[2].split('\x1b')[0]))
times = [entry[0] for entry in transitions]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(root/'d1/rosbag2_autoware'), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
topic = '/control/command/control_cmd'
msg_type = get_message(next(t.type for t in reader.get_all_topics_and_types() if t.name == topic))
reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
previous = None
anomalies = []
while reader.has_next():
    _, data, receive_ns = reader.read_next()
    message = deserialize_message(data, msg_type)
    current = dict(source=message.stamp.sec+message.stamp.nanosec*1e-9,
                   receive=receive_ns*1e-9, speed=message.longitudinal.speed,
                   acceleration=message.longitudinal.acceleration)
    if previous is not None:
        gap = current['source'] - previous['source']
        if gap <= 0. or gap > .050:
            state_index = bisect.bisect_right(times, current['receive'])-1
            anomalies.append(dict(source_gap_sec=gap,
                                  receive_gap_sec=current['receive']-previous['receive'],
                                  observed_phase=transitions[state_index][1] if state_index >= 0 else None,
                                  previous=previous, current=current))
    previous = current
result = dict(scope='source/receipt clock distinction; phase joined by receipt wall-clock',
              transitions=transitions, anomalies=anomalies)
if anomalies:
    lower = anomalies[0]['previous']['receive'] - .1
    upper = anomalies[-1]['current']['receive'] + .1
    names = ['/clock', '/localization/kinematic_state', '/vehicle/status/velocity_status']
    diagnostic = rosbag2_py.SequentialReader()
    diagnostic.open(rosbag2_py.StorageOptions(uri=str(root/'d1/rosbag2_autoware'), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in diagnostic.get_all_topics_and_types() if t.name in names}
    diagnostic.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    rows = {name: [] for name in types}
    while diagnostic.has_next():
        name, data, receive_ns = diagnostic.read_next()
        receive = receive_ns*1e-9
        if not lower <= receive <= upper:
            continue
        msg = deserialize_message(data, types[name])
        stamp = msg.clock if name == '/clock' else msg.header.stamp
        row = dict(receive=receive, source=stamp.sec+stamp.nanosec*1e-9)
        if name == '/localization/kinematic_state':
            row.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y, speed=msg.twist.twist.linear.x)
        elif name == '/vehicle/status/velocity_status':
            row['speed'] = msg.longitudinal_velocity
        rows[name].append(row)
    result['anomaly_observation_window'] = dict(receive_range=[lower, upper], rows=rows)
(root/'source-clock-anomalies.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
