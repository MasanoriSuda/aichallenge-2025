"""Compare public native arithmetic against a separately compiled frozen source."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

root = Path.cwd()
p = root / 'aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros'
s = Path(__file__).resolve().parent
baseline = root / 'output/20260911-nine-state-shared-arithmetic-baseline/mpcc_vehicle_model.cpp'
out = root / ('output/20260911-nine-state-shared-arithmetic-' + sys.argv[1])
out.mkdir(exist_ok=False)
names = ('integration_steps', 'valid', 'finite', 'fingerprint', 'derivative', 'advance', 'nominal_stop_map_distance')
old = baseline.read_text()
for name in names:
    old = re.sub(r'\b' + name + r'\b', 'baseline_' + name, old)
reference = out / 'baseline.cpp'
reference.write_text(old)
files = [baseline, p / 'src/mpcc_vehicle_model.cpp', p / 'include/multi_purpose_mpc_ros/mpcc_vehicle_model.hpp', p / 'include/multi_purpose_mpc_ros/mpcc_vehicle_model_kernel.hpp', s / 'compare_shared_kernel.cpp', s / '../native_vehicle_model.cpp', Path(__file__).resolve()]
for source in files:
    target = out / 'sources' / source.relative_to(root)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
cmd = ['g++', '-std=c++17', '-O2', '-I' + str(p / 'include'), str(reference), str(p / 'src/mpcc_vehicle_model.cpp'), str(s / 'compare_shared_kernel.cpp'), '-o', str(out / 'compare')]
with (out / 'build.log').open('w') as log:
    result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=120)
print((out / 'build.log').read_text(), flush=True)
(out / 'manifest.json').write_text(json.dumps(dict(command=cmd, compiler=subprocess.check_output(['g++', '--version'], text=True), return_code=result.returncode, files={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}), indent=2) + '\n')
result.check_returncode()
parameters = root / 'output/20260910-empirical-plant-native-r1/parameters.txt'
assert parameters.exists(), 'use the preserved native parameter fixture'
shutil.copy2(parameters, out / 'parameters.txt')
with parameters.open() as source, (out / 'run.log').open('w') as log:
    result = subprocess.run([str(out / 'compare')], stdin=source, stdout=log, stderr=subprocess.STDOUT, timeout=120)
print((out / 'run.log').read_text(), flush=True)
result.check_returncode()
