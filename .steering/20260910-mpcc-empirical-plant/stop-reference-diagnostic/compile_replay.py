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
out = Path('/output/20260910-nine-state-revalidation-tool-' + sys.argv[1])
out.mkdir(exist_ok=False)
link = shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
retained_link = shlex.split((b/'CMakeFiles/test_mpcc_rate_resolved_retained_revalidation.dir/link.txt').read_text())
archives = list(dict.fromkeys(arg for arg in link + retained_link
                              if arg.endswith('.a') and 'gtest' not in arg))
dependencies = [arg for arg in link[link.index('-o')+2:] if not arg.endswith('.a')]
cmd = [link[0], '-std=c++17', '-O2', '-I'+str(p/'include'), '-I'+str(p/'src'),
       '-I/usr/include/eigen3', str(s/'replay_revalidation.cpp'), '-o', str(out/'replay'),
       '-Wl,--start-group'] + archives + ['-Wl,--end-group'] + dependencies
with (out/'build.log').open('w') as log:
    result = subprocess.run(cmd, cwd=b, stdout=log, stderr=subprocess.STDOUT, timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd, return_code=result.returncode,
    files={str(s/name):hashlib.sha256((s/name).read_bytes()).hexdigest() for name in
           ('replay_revalidation.cpp','../revalidation_input.hpp','../replay_certified_plan.cpp')},
    purpose='Exact r11 normal/track Stop reference comparison and native trajectory samples; same production retained source included, no runtime authority'), indent=2)+'\n')
print((out/'build.log').read_text(), flush=True)
result.check_returncode()
