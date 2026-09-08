"""Compare only numerical representations on the new run's first normal QP."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
out = Path('/output/20260909-side-peer-dev2-normal-qp')
out.mkdir(exist_ok=False)
source = next(Path('/output/20260909-side-peer-stop-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000000381*/snapshot.yaml'))
probe = Path(__file__).with_name('probe.cpp')
link = shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command = [link[0], '-std=c++17', '-O2', '-I'+str(package/'include'), '-I/usr/include/eigen3',
           str(probe), '-o', str(out/'probe')] + link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
with (out/'comparison.log').open('w') as log:
    subprocess.run([str(out/'probe'), 'qp', str(source), str(out/'qp')], cwd=out,
                   stdout=log, stderr=subprocess.STDOUT, check=True, timeout=60)
(out/'manifest.json').write_text(json.dumps(dict(source=str(source),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    helper_sha256=hashlib.sha256(probe.read_bytes()).hexdigest(),
    binary_sha256=hashlib.sha256((out/'probe').read_bytes()).hexdigest()), indent=2)+'\n')
