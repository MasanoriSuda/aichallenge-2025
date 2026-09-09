from pathlib import Path
import hashlib,json,shlex,subprocess,yaml
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-complete-rest-candidates')
out=Path('/output/20260909-complete-rest-production-replay');out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/test_mpcc_architecture_comparison.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3',str(s/'probe_production.cpp'),'-o',str(out/'probe'),
 'libmulti_purpose_mpc_ros_mpcc_rate_resolved_production_adapter.a','libmulti_purpose_mpc_ros_mpcc_rate_resolved_command_candidate.a',
 'libmulti_purpose_mpc_ros_mpcc_rate_resolved_retained_revalidation.a','libmulti_purpose_mpc_ros_mpcc_latest_state_feedback.a',
 'libmulti_purpose_mpc_ros_mpc_state_prediction.so','libmulti_purpose_mpc_ros_mpcc_rate_resolved_certified_plan.a']+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
results=[]
for scene,run,domain,speed,steering in [('1629','20260909-failure-bundle-dev2-r1','d2',4.342925292866534,-0.198264),('4264','20260909-complete-stop-dev2-r1','d1',8.718348824128569,0.092016)]:
 work=out/scene;work.mkdir()
 world=next(Path('/output',run,domain,'mpcc_architecture_snapshots').glob(f'{int(scene):012d}*/snapshot.yaml'))
 cfg=dict(current_speed_mps=speed,current_time_steering_rad=steering,limits='Speed from same-run odometry; steering from six-decimal same-decision diagnostics. Native replay has no runtime publication or actual async adoption. Original world/control origin retained; candidate owns uniform source maximum stage dt and terminal rest.')
 (work/'input.yaml').write_text(yaml.safe_dump(cfg))
 with (work/'native.log').open('w') as log:r=subprocess.run([str(out/'probe'),str(world),str(work/'report.yaml'),str(work/'input.yaml')],cwd=work,stdout=log,stderr=subprocess.STDOUT,timeout=120)
 print(scene,r.returncode,(work/'native.log').read_text(),flush=True)
 results.append(dict(scene=scene,source=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),return_code=r.returncode))
 r.check_returncode()
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,results=results,authority=False),indent=2)+'\n')
