from pathlib import Path
import json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
run=Path('/output/20260909-final-authority-dev2-r1')
out=Path('/output/20260909-exact-join-peer-inputs');out.mkdir(exist_ok=False)
for domain in ['d1','d2']:
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run/domain/'rosbag2_autoware'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    metadata=reader.get_all_topics_and_types()
    (out/(domain+'-topics.json')).write_text(json.dumps({t.name:t.type for t in metadata},indent=2)+'\n')
    topics=[t.name for t in metadata if (domain=='d1' and t.name=='/v2x/vehicle_positions') or (domain=='d2' and 'gnss' in t.name)]
    types={t.name:get_message(t.type) for t in metadata if t.name in topics};reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    rows=[]
    while reader.has_next():
        topic,data,ns=reader.read_next()
        if not 1788919065.5<=ns*1e-9<=1788919067.:continue
        rows.append(dict(topic=topic,receipt=ns*1e-9,message=message_to_ordereddict(deserialize_message(data,types[topic]))))
    (out/(domain+'-inputs.json')).write_text(json.dumps(rows,indent=2)+'\n')
    print(domain,topics,len(rows),flush=True)
