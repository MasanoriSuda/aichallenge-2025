"""Bounded offline formulation experiment, no production objective/authority change."""
from pathlib import Path
import shlex,subprocess,hashlib,json
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-stop-proof-provenance')
out=Path('/output/20260909-exact-join-missions');out.mkdir(exist_ok=False)
src=p/'src/mpcc_rate_resolved_stop_control_lattice.cpp'
copy='extern int diagnostic_stop_mission;\n'+src.read_text()
anchor='  result.detail = "accepted/free-controls-through-rest";'
assert copy.count(anchor)==1
copy=copy.replace(anchor,r'''
  if (diagnostic_stop_mission != 0) {
    auto & variant=result.candidate;
    if (diagnostic_stop_mission>=2) {
      const auto law=build_current_world_maximum_braking_candidate(variant,solver_tolerance);
      if (!law.accepted())return law;
      variant=law.candidate;
      if (diagnostic_stop_mission==2) {
        for(std::size_t i=0;i<variant.request.states.size();++i) {
          variant.request.states[i].weight=source.request.states[i].weight;
          variant.request.states[i].linear_cost=source.request.states[i].linear_cost;
        }
        for(std::size_t i=0;i<variant.request.inputs.size();++i) {
          variant.request.inputs[i].weight=source.request.inputs[i].weight;
          variant.request.inputs[i].linear_cost=source.request.inputs[i].linear_cost;
        }
        variant.request.input_delta_weight=source.request.input_delta_weight;
      }
    } else {
      for(auto & state:variant.request.states){state.weight.setZero();state.linear_cost.setZero();}
      for(auto & input:variant.request.inputs){input.weight.setZero();input.linear_cost.setZero();}
      variant.request.input_delta_weight.setZero();
    }
    variant.identity.source_context.cost_schema_id+="/offline-stop-mission-v1/"+std::to_string(diagnostic_stop_mission);
    variant.identity.source_context=contract::seal_problem_context(variant.identity.source_context);
  }
'''+anchor)
path=out/'stop_candidate_copy.cpp';path.write_text(copy)
link=shlex.split((b/'CMakeFiles/test_mpcc_architecture_comparison.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3',str(s/'compare_missions.cpp'),str(path),'-o',str(out/'compare')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
results=[]
for run,domain,decision in [('20260909-final-authority-dev2-r1','d1',945)]:
    source=next(Path('/output',run,domain,'mpcc_architecture_snapshots').glob(f'{decision:012d}*/snapshot.yaml'))
    work=out/str(decision);work.mkdir()
    with (work/'native.log').open('w') as log:r=subprocess.run([str(out/'compare'),str(source),str(work/'report.yaml')],cwd=work,stdout=log,stderr=subprocess.STDOUT,timeout=120)
    print(decision,r.returncode,(work/'native.log').read_text(),flush=True)
    results.append(dict(source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),return_code=r.returncode));r.check_returncode()
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),copy_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),results=results,authority=False,limits='Four explicit candidate problems per sealed world. Same uniform source maximum clock, all observed peer QP guidance and physical proofs. Objective and additional braking-law constraints are experimental formulation differences, not production tuning. No current-observation join or async publication.'),indent=2)+'\n')
