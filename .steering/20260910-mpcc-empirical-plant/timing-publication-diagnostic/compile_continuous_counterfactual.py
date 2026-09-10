"""Diagnostic only: remove the publication hold to test Stop suffix closure.

This deliberately non-publishable counterfactual is never installed or linked
into production. The generated copy and both source hashes remain in its output.
"""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess
import sys

s = Path(__file__).resolve().parent
p = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out = Path('/output/20260910-nine-state-continuous-counterfactual-' + sys.argv[1])
out.mkdir(exist_ok=False)
source = p / 'src/mpcc_rate_resolved_physical_adapter.cpp'
original = source.read_text()
needle = '        applied_control.steering_rate_radps = 0.0;'
assert original.count(needle) == 1
variant = out / 'continuous-counterfactual.cpp'
variant.write_text(original.replace(needle, '        // Diagnostic only: retain the original continuous rate.'))
link = shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
retained_link = shlex.split((b/'CMakeFiles/test_mpcc_rate_resolved_retained_revalidation.dir/link.txt').read_text())
archives = list(dict.fromkeys(arg for arg in link + retained_link
                             if arg.endswith('.a') and 'gtest' not in arg))
dependencies = [arg for arg in link[link.index('-o')+2:] if not arg.endswith('.a')]
driver = s / 'stop_recursion.cpp'
cmd = [link[0], '-std=c++17', '-O2', '-I'+str(p/'include'), '-I'+str(p/'src'),
       '-I/usr/include/eigen3', str(driver), str(variant), '-o', str(out/'replay'),
       '-Wl,--start-group'] + archives + ['-Wl,--end-group'] + dependencies
with (out/'build.log').open('w') as log:
    result = subprocess.run(cmd, cwd=b, stdout=log, stderr=subprocess.STDOUT, timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(authority=False, command=cmd,
    return_code=result.returncode, files={str(f):hashlib.sha256(f.read_bytes()).hexdigest()
    for f in [source, variant, driver, Path(__file__)]}), indent=2)+'\n')
print((out/'build.log').read_text(), flush=True)
result.check_returncode()
