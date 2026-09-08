from pathlib import Path
import hashlib,json,shlex,subprocess
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-complete-stop-suffix')
out=Path('/output/20260909-complete-stop-free-artifact-after');out.mkdir(exist_ok=False)
src=(p/'src/mpcc_architecture_comparison.cpp').read_text()
start=src.index('ArmResult evaluate_arm(')
anchor='  const auto adapted = physical::build('
idx=src.index(anchor,start)
src=src[:idx]+'  captured_execution=solved.execution_artifact;\n'+src[idx:]
src='#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"\n#include <memory>\nextern std::shared_ptr<const multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact::ExecutionArtifact> captured_execution;\n'+src
(out/'comparison_capture.cpp').write_text(src)
link=shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I'+str(p/'src'),'-I/usr/include/eigen3','-I'+str(out),str(s/'capture_free_stop.cpp'),'-o',str(out/'compare')]+['libmulti_purpose_mpc_ros_mpcc_rate_resolved_production_adapter.a','libmulti_purpose_mpc_ros_mpcc_rate_resolved_command_candidate.a','libmulti_purpose_mpc_ros_mpcc_rate_resolved_retained_revalidation.a','libmulti_purpose_mpc_ros_mpcc_latest_state_feedback.a','libmulti_purpose_mpc_ros_mpc_state_prediction.so','libmulti_purpose_mpc_ros_mpcc_rate_resolved_certified_plan.a']+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
world=next(Path('/output/20260909-failure-bundle-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000001629*/snapshot.yaml'))
with (out/'native.log').open('w') as log:subprocess.run([str(out/'compare'),str(world),str(out/'report.yaml')],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
(out/'manifest.json').write_text(json.dumps(dict(world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),command=cmd,authority=False),indent=2)+'\n')
print((out/'native.log').read_text())
