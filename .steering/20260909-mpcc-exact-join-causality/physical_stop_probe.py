"""Compile zero-solve evidence replay before starting the simulator."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess
import sys

s = Path(__file__).resolve().parent
p = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out = Path('/output/20260909-exact-join-physical-tool-' + sys.argv[1])
out.mkdir(exist_ok=False)
link = shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
retained_link = shlex.split((b/'CMakeFiles/test_mpcc_rate_resolved_retained_revalidation.dir/link.txt').read_text())
archives = list(dict.fromkeys(arg for arg in link + retained_link
                              if arg.endswith('.a') and 'gtest' not in arg))
dependencies = [arg for arg in link[link.index('-o')+2:] if not arg.endswith('.a')]
cmd = [link[0], '-std=c++17', '-O2', '-I'+str(p/'include'), '-I'+str(p/'src'),
       '-I/usr/include/eigen3', str(s/'physical_stop_probe.cpp'), '-o', str(out/'replay'),
       '-Wl,--start-group'] + archives + ['-Wl,--end-group'] + dependencies
with (out/'build.log').open('w') as log:
    result = subprocess.run(cmd, cwd=b, stdout=log, stderr=subprocess.STDOUT, timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd, return_code=result.returncode,
    files={str(s/name):hashlib.sha256((s/name).read_bytes()).hexdigest() for name in
           ('physical_stop_probe.cpp',)},
    purpose='Independent physical stopping controls witness search; no QP certificate or authority'), indent=2)+'\n')
print((out/'build.log').read_text(), flush=True)
result.check_returncode()

source=next(Path('/output/20260909-final-authority-dev2-r1/d1/mpcc_architecture_snapshots').glob('000000000945*/snapshot.yaml'))
with (out/'native.log').open('w') as log:
    result=subprocess.run([str(out/'replay'),str(source),str(out/'report.yaml')],cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=120)
print((out/'native.log').read_text(),flush=True)
result.check_returncode()
