"""Same-run active topic rates and source/receipt gaps, before teardown."""
import bisect,json,re,sys
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path(sys.argv[1]);domain=sys.argv[2]
monitor=json.load(open(root/'monitor-summary.json'));shutdown=monitor['shutdown_started_wall_sec']
transitions=[]
for raw in (root/domain/'autoware.log').read_text(errors='replace').splitlines():
 line=re.sub(r'\x1b\[[0-9;]*m','',raw)
 match=re.search(r'\[(\d+\.\d+)\].*AWSIM vehicle state observed: state=(\S+)',line)
 if match:transitions.append((float(match[1]),match[2]))
transition_times=[p[0] for p in transitions]
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(root/domain/'rosbag2_autoware'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
requested=['/control/command/control_cmd','/localization/kinematic_state','/localization/acceleration','/vehicle/status/velocity_status','/sensing/imu/imu_raw','/v2x/vehicle_positions','/clock']
types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in requested}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)));rows={k:[] for k in types}
while reader.has_next():
 topic,data,ns=reader.read_next();receive=ns*1e-9
 index=bisect.bisect_right(transition_times,receive)-1
 if index<0 or transitions[index][1] not in ['ready','start'] or receive>=shutdown:continue
 msg=deserialize_message(data,types[topic]);stamp=msg.clock if topic=='/clock' else msg.header.stamp if hasattr(msg,'header') else msg.stamp if hasattr(msg,'stamp') else None
 rows[topic].append((receive,None if stamp is None else stamp.sec+stamp.nanosec*1e-9))
summary={}
for topic,values in rows.items():
 receive_gaps=[b[0]-a[0] for a,b in zip(values,values[1:])]
 source_gaps=[b[1]-a[1] for a,b in zip(values,values[1:]) if a[1] is not None and b[1] is not None]
 summary[topic]=dict(messages=len(values),receive_hz=(len(values)-1)/(values[-1][0]-values[0][0]) if len(values)>1 else None,receive_gap_max_ms=max(receive_gaps,default=0)*1000,receive_gaps_over50ms=sum(g>.05 for g in receive_gaps),source_gap_max_ms=max(source_gaps,default=0)*1000,source_duplicates_or_backward=sum(g<=0 for g in source_gaps),source_gaps_over50ms=sum(g>.05 for g in source_gaps))
result=dict(run_id=root.name,domain=domain,scope='Ready/Start by same-run receive wall-time, before explicit shutdown; source timing and receipt timing distinct',missing_topics=sorted(set(requested)-set(types)),topics=summary)
(root/(domain+'-topic-timing.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
