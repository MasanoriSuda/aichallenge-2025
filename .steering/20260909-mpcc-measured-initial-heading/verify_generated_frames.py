from pathlib import Path
import json
import math
import subprocess
import xml.etree.ElementTree as E
from ament_index_python.packages import get_package_share_directory

out = Path('/output/20260909-initial-heading-generated-frames-r1'); out.mkdir(exist_ok=False)
model = Path(get_package_share_directory('tier4_vehicle_launch'))/'urdf/vehicle.xacro'
calibration = Path(get_package_share_directory('racing_kart_sensor_kit_description'))/'config'
report = {}
for simulation in ['true', 'false']:
    command = ['xacro', str(model), 'vehicle_model:=racing_kart',
               'sensor_model:=racing_kart_sensor_kit', 'config_dir:='+str(calibration),
               'simulation:='+simulation]
    data = subprocess.check_output(command)
    (out/('simulation-'+simulation+'.urdf')).write_bytes(data)
    joints = {j.find('child').get('link'): j for j in E.fromstring(data).findall('joint')}
    paths = {}
    for child in ['imu_link', 'gnss_link']:
        current = child; chain = []; yaw = 0.0
        while current != 'base_link':
            j = joints[current]; origin = j.find('origin')
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
            assert rpy[:2] == [0.0, 0.0]
            yaw += rpy[2]
            chain.append(dict(child=current, parent=j.find('parent').get('link'),
                              rpy=rpy, xyz=origin.get('xyz')))
            current = j.find('parent').get('link')
        paths[child] = dict(chain=chain, body_relative_yaw=yaw)
    expected = math.pi/2 if simulation == 'true' else -math.pi/2
    assert abs(paths['imu_link']['body_relative_yaw']-expected) < 1e-10
    assert paths['gnss_link']['body_relative_yaw'] == 0.0
    report[simulation] = dict(command=command, paths=paths)
(out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2), flush=True)
