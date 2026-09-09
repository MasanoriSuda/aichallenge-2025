from pathlib import Path
import hashlib,json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
run=Path('/output/20260909-peer-viability-dev2-r1');out=Path('/output/20260909-ego-viability-ekf-inputs');out.mkdir(exist_ok=False)
topics={'/localization/kinematic_state':'odom','/localization/imu_gnss_poser/pose_with_covariance':'pose','/localization/twist_estimator/twist_with_covariance':'twist'}
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'d1/rosbag2_autoware'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics};reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
rows=[]
while reader.has_next():
 topic,data,ns=reader.read_next();m=deserialize_message(data,types[topic]);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if not 9.0<=stamp<=10.1:continue
 rows.append(dict(kind=topics[topic],stamp=stamp,receipt=ns*1e-9,message=message_to_ordereddict(m)))
(out/'inputs.json').write_text(json.dumps(rows,indent=2)+'\n')
(out/'manifest.json').write_text(json.dumps(dict(run=str(run),domain=1,rows=len(rows),purpose='Shared recorded input sequence and reinitialisation for mathematical parity of old/new EKF with frozen per-callback clock; not reconstruction of all original hidden filter state',bags={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (run/'d1/rosbag2_autoware').glob('*.mcap')}),indent=2)+'\n')
print('Preserved',len(rows),'EKF input/odometry events',flush=True)
