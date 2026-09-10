from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys

s = Path(__file__).resolve().parent
p = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
out = Path('output/20260911-nine-state-tire-enclosure-' + sys.argv[1])
out.mkdir(exist_ok=False)
files = [s / 'compare_tire_bounds.cpp', s / 'interval_model.hpp', s / '../native_vehicle_model.cpp',
         p / 'src/mpcc_vehicle_model.cpp', p / 'include/multi_purpose_mpc_ros/mpcc_vehicle_model_kernel.hpp',
         Path('output/20260910-empirical-plant-native-r1/parameters.txt')]
for source in files:
    shutil.copy2(source, out / source.name)
cmd = ['g++', '-std=c++17', '-O2', '-I' + str(p / 'include'), str(files[0]), str(files[3]), '-o', str(out / 'compare')]
with (out / 'build.log').open('w') as log:
    result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=120)
(out / 'manifest.json').write_text(json.dumps(dict(command=cmd, build_return_code=result.returncode,
    files={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}), indent=2) + '\n')
result.check_returncode()
with files[-1].open() as parameters, (out / 'run.log').open('w') as log:
    result = subprocess.run([str(out / 'compare')], stdin=parameters, stdout=log, stderr=subprocess.STDOUT, timeout=120)
print((out / 'run.log').read_text(), flush=True)
result.check_returncode()
