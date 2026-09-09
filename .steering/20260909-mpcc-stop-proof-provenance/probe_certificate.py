from pathlib import Path
import shlex,subprocess,hashlib,json
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-stop-proof-provenance')
out=Path('/output/20260909-stop-provenance-certificate');out.mkdir(exist_ok=False)
src=p/'src/mpcc_rate_resolved_stop_lattice_shadow.cpp'
copy='#include <yaml-cpp/yaml.h>\nextern bool diagnostic_bind_solved_tolerance;\nextern YAML::Node diagnostic_certificate;\n'+src.read_text()
anchor='  result.bound_tolerance_m = replay.bound_tolerance_m;'
assert copy.count(anchor)==1
copy=copy.replace(anchor,'  result.bound_tolerance_m = diagnostic_bind_solved_tolerance ? trajectory.lateral_bound_tolerance_m : replay.bound_tolerance_m;')
anchor='        result.solver_outcome = solved.outcome;'
assert copy.count(anchor)==1
copy=copy.replace(anchor,'        diagnostic_execution=solved.execution_artifact;\n'+anchor)
copy='#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"\nextern std::shared_ptr<const multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact::ExecutionArtifact> diagnostic_execution;\n'+copy
anchor='        result.dynamic_valid = dynamic_result.valid;'
assert copy.count(anchor)==1
copy=copy.replace(anchor,r'''        diagnostic_certificate["source_validation_reason"]=dynamic::to_string(dynamic_result.source_validation_reason);
        diagnostic_certificate["replay_tolerance"]=stop_solver_source->replay_world->bound_tolerance_m;
        diagnostic_certificate["trajectory_tolerance"]=exact.lateral_bound_tolerance_m;
        diagnostic_certificate["snapshot_tolerance"]=wall_snapshot.bound_tolerance_m;
        diagnostic_certificate["wall_outcome"]=physical::to_string(wall_result.outcome);
        diagnostic_certificate["dynamic_valid"]=dynamic_result.valid;
        diagnostic_certificate["dynamic_clear"]=dynamic_result.clear;
        diagnostic_certificate["candidate_fingerprint"]=stop_solver_source->identity.source_context.fingerprint;
'''+anchor)
path=out/'stop_shadow_observation.cpp';path.write_text(copy)
link=shlex.split((b/'CMakeFiles/test_mpcc_architecture_comparison.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3',str(s/'probe_certificate.cpp'),str(path),'-o',str(out/'probe')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
results=[]
for run,domain,decision in [('20260909-complete-rest-single-r1','d1',2056),('20260909-complete-rest-single-r1','d1',2162),('20260909-complete-rest-dev2-r1','d2',923)]:
 source=next(Path('/output',run,domain,'mpcc_architecture_snapshots').glob(f'{decision:012d}*/snapshot.yaml'))
 work=out/str(decision);work.mkdir()
 with (work/'native.log').open('w') as log:r=subprocess.run([str(out/'probe'),str(source),str(work/'report.yaml')],cwd=work,stdout=log,stderr=subprocess.STDOUT,timeout=120)
 print(decision,r.returncode,(work/'native.log').read_text(),flush=True)
 results.append(dict(source=str(source),world_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),return_code=r.returncode));r.check_returncode()
(out/'manifest.json').write_text(json.dumps(dict(baseline_commit='ae6862aa',source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),copy_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),command=cmd,results=results,authority=False),indent=2)+'\n')
