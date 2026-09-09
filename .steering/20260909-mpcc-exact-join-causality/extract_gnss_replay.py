"""Preserve exact GNSS inputs and same-stamp original node outputs, both Domains."""
from pathlib import Path
import hashlib
import json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
run=Path('/output/20260909-final-authority-dev2-r1')
out=Path('/output/20260909-exact-join-gnss-inputs');out.mkdir(exist_ok=False)
for domain in ['d1','d2']:
    reader=rosbag2_py.SequentialReader()
    bag=run/domain/'rosbag2_autoware'
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    topics=['/sensing/gnss/nav_sat_fix','/sensing/gnss/pose_with_covariance']
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    fixes={};poses={}
    while reader.has_next():
        topic,data,ns=reader.read_next();message=deserialize_message(data,types[topic])
        stamp=message.header.stamp;sec=stamp.sec+stamp.nanosec*1e-9
        if sec>10.089999774:continue
        key=(stamp.sec,stamp.nanosec)
        row=dict(receipt=ns*1e-9,message=message_to_ordereddict(message))
        (fixes if topic.endswith('nav_sat_fix') else poses)[key]=row
    rows=[dict(fix=fixes[key],original=poses.get(key)) for key in sorted(fixes)]
    (out/(domain+'.json')).write_text(json.dumps(rows,indent=2)+'\n')
    print(domain,len(rows),'same-stamp original outputs',sum(r['original'] is not None for r in rows),flush=True)
(out/'manifest.json').write_text(json.dumps(dict(run=str(run),source='Exact recorded NavSatFix inputs and original GNSS node outputs; EKF or physical ground truth not inferred',bags={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in run.glob('d*/rosbag2_autoware/*.mcap')}),indent=2)+'\n')
