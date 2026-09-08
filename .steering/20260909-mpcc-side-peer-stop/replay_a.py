"""Recheck the first normal arm after unifying numerical tangent selection."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out = Path('/output/20260909-side-peer-normal-tangent-after')
out.mkdir(exist_ok=False)
source = next(Path('/output/20260909-peer-envelope-dev2-r1/world-target-comparison-r2/396/candidate').glob('*/snapshot.yaml'))
(out / 'main.cpp').write_text(r'''
#include "mpcc_architecture_comparison.cpp"
#include <iostream>
int main(int argc, char **argv) {
  namespace c = multi_purpose_mpc_ros::mpcc_architecture_comparison;
  namespace a = multi_purpose_mpc_ros::mpcc_architecture_snapshot;
  std::string detail;
  const auto recorded = a::load_recorded_interaction_snapshot(argv[1], &detail);
  if (!recorded || !a::interaction_snapshot_matches_fingerprint(
      recorded->source, recorded->interaction_fingerprint)) { std::cout << detail; return 2; }
  const auto r = c::evaluate_arm(c::Arm::PersistentA, recorded->source,
    recorded->interaction_fingerprint, recorded->interaction_fingerprint,
    c::resolve_audit_terminal_successor(recorded->source));
  std::cout << "source=" << recorded->interaction_fingerprint << " stage=" << c::to_string(r.stage)
    << " solver=" << multi_purpose_mpc_ros::mpcc_rate_resolved_shadow::to_string(r.solver_outcome)
    << " solve_ms=" << r.solver_compute_ms << " bundle=" << r.bundle.has_value()
    << " detail=" << r.detail << '\n';
  return r.bundle ? 0 : 4;
}
''')
link = shlex.split((build / 'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command = [link[0], '-std=c++17', '-O2', '-I'+str(package/'include'), '-I'+str(package/'src'),
           '-I/usr/include/eigen3', str(out/'main.cpp'), '-o', str(out/'probe')]
command += link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
with (out/'probe.log').open('w') as log:
    completed = subprocess.run([str(out/'probe'), str(source)], cwd=out, stdout=log,
                               stderr=subprocess.STDOUT, timeout=60)
manifest = dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                program_sha256=hashlib.sha256((out/'probe').read_bytes()).hexdigest(),
                return_code=completed.returncode)
(out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
print((out/'probe.log').read_text(), flush=True)
