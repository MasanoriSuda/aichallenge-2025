"""Use the actual build and loader; old source files stay immutable."""
from pathlib import Path
import hashlib
import json
import math
import shlex
import subprocess
import yaml

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
steering = Path('/task-steering/20260909-mpcc-coordinate-consistency')
old = Path('/output/20260908-execution-evidence-single-r3')
out = Path('/output/20260909-coordinate-fixed-input')
out.mkdir(exist_ok=False)
link = shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
index = link.index('-o')
command = [link[0], '-std=c++17', '-O2', '-I'+str(package/'include'), '-I/usr/include/eigen3',
           str(steering/'compare_saved.cpp'), '-o', str(out/'compare_saved')] + link[index+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
results = []
for sequence in [6782, 7405]:
    paths = list((old/'d1/mpcc_architecture_snapshots').glob(f'{sequence:012d}-*/snapshot.yaml'))
    assert len(paths) == 1, paths
    source = paths[0]
    target = out/str(sequence)
    with (out/f'{sequence}-prepare.log').open('w') as log:
        subprocess.run([str(out/'compare_saved'), str(source), str(target)], cwd=build,
                       stdout=log, stderr=subprocess.STDOUT, check=True, timeout=60)
    report = yaml.safe_load((target/'model-input-and-rollout.yaml').read_text())
    item = {k: v for k, v in report.items() if k != 'points'}
    item['source_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    if sequence == 6782:
        oracle = json.loads((old/'executed-replay/frame-comparison.json').read_text())
        assert len(report['points']) == len(oracle['rows'])
        errors = []
        for point, reference in zip(report['points'], oracle['rows']):
            assert abs(point['elapsed_sec'] - reference['elapsed_sec']) < 1e-9
            pose = reference['poses']['C']
            errors.append(dict(elapsed_sec=point['elapsed_sec'],
                               position_m=math.hypot(point['x_m']-pose[0], point['y_m']-pose[1]),
                               yaw_rad=math.atan2(math.sin(point['yaw_rad']-pose[2]), math.cos(point['yaw_rad']-pose[2]))))
        item['error_vs_independent_cartesian'] = {
            str(horizon): max(row['position_m'] for row in errors if row['elapsed_sec'] <= horizon+1e-9)
            for horizon in [0.055, 0.1, 0.2, errors[-1]['elapsed_sec']]}
        (target/'oracle-errors.json').write_text(json.dumps(errors, indent=2)+'\n')
    for mode, extra in [('architectures', []), ('wall-restoration', ['--wall-restoration-only'])]:
        with (target/f'{mode}.log').open('w') as log:
            process = subprocess.run([str(build/'mpcc_architecture_compare'), report['candidate_snapshot'], *extra],
                                     cwd=target, stdout=log, stderr=subprocess.STDOUT, timeout=60)
        item[mode+'_exit'] = process.returncode
    results.append(item)
    print(json.dumps(item), flush=True)
(out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
