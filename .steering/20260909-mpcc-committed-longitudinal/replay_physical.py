"""Reconstruct the captured Stop and revalidate against its exact failure world."""
from pathlib import Path
import shlex,subprocess
package=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
steering=Path('/task-steering/20260909-mpcc-committed-longitudinal')
run=Path('/output/20260909-published-stop-observation-dev2-r1')
out=Path('/output/20260909-committed-longitudinal-physical-replay')
out.mkdir(exist_ok=True)
link=shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command=[link[0],'-std=c++17','-O2','-I'+str(package/'include'),'-I'+str(package/'src'),'-I/usr/include/eigen3',str(steering/'replay_physical.cpp'),'-o',str(out/'reconstruct_execution'),'libmulti_purpose_mpc_ros_mpcc_rate_resolved_retained_revalidation.a','libmulti_purpose_mpc_ros_mpcc_latest_state_feedback.a','libmulti_purpose_mpc_ros_mpc_state_prediction.so','libmulti_purpose_mpc_ros_mpcc_rate_resolved_certified_plan.a']+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
 subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
source=next((run/'d2/mpcc_architecture_snapshots').glob('000000000453*/snapshot.yaml'))
with (out/'native.log').open('w') as log:
 subprocess.run([str(out/'reconstruct_execution'),str(source),str(out/'report.yaml'),str(steering/'replay-input.yaml')],cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
print(out/'report.yaml',flush=True)
