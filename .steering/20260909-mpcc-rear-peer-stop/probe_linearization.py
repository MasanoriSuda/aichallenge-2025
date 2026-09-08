"""Inspect selected tangent state+input support on the sealed1629world."""
from pathlib import Path
import hashlib,json,shlex,subprocess

p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-rear-peer-stop')
out=Path('/output/20260909-rear-peer-linearization-domain')
out.mkdir(exist_ok=False)
src=p/'src/mpcc_rate_resolved_adapter.cpp'
text='#include <yaml-cpp/yaml.h>\nextern bool diagnostic_restrict_model_domain;\nextern YAML::Node diagnostic_linearization_observations;\n'+src.read_text()
text=text.replace('const auto & linearization_state = *selected_state;', 'auto linearization_state = *selected_state;')
anchor='    const auto linearization = model::linearize_temporal_frenet(\n      model::LinearizationRequest{\n        linearization_state[model::kLateralIndex],'
assert text.count(anchor)==1
probe=r'''
    YAML::Node diagnostic;
    diagnostic["stage"] = stage;
    diagnostic["state_progress_m"] = linearization_state[model::kProgressIndex];
    diagnostic["virtual_speed_mps"] = linearization_input[model::kVirtualProgressSpeedIndex];
    diagnostic["dt_sec"] = semantic_input.stage_dt_sec;
    diagnostic["raw_terminal_progress_m"] = primal[(stage+1)*model::kStateDimension+model::kProgressIndex];
    if (request.course_frame.knots) {
      const auto & frame=request.course_frame;
      const double lower=frame.knots->front().progress_m-frame.progress_origin_m;
      const double upper=frame.knots->back().progress_m-frame.progress_origin_m;
      const double start=linearization_state[model::kProgressIndex];
      const double dt=semantic_input.stage_dt_sec;
      const double end=start+dt*linearization_input[model::kVirtualProgressSpeedIndex];
      diagnostic["frame_lower_m"]=lower;diagnostic["frame_upper_m"]=upper;
      diagnostic["selected_terminal_progress_m"]=end;
      diagnostic["outside_domain"]=end<lower || end>upper;
      if (diagnostic_restrict_model_domain) {
        const double v_lower=std::max(problem.input_lower[problem_input+model::kVirtualProgressSpeedIndex],(lower-start)/dt);
        const double v_upper=std::min(problem.input_upper[problem_input+model::kVirtualProgressSpeedIndex],(upper-start)/dt);
        if (v_lower<=v_upper) {
          linearization_input[model::kVirtualProgressSpeedIndex]=std::clamp(linearization_input[model::kVirtualProgressSpeedIndex],v_lower,v_upper);
        }
        diagnostic["projected_virtual_speed_mps"]=linearization_input[model::kVirtualProgressSpeedIndex];
      }
    }
    diagnostic_linearization_observations.push_back(diagnostic);
'''
text=text.replace(anchor,probe+anchor)
copy=out/'adapter_probe.cpp';copy.write_text(text)
link=shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3',str(s/'probe_linearization.cpp'),str(copy),'-o',str(out/'probe')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
world=next(Path('/output/20260909-failure-bundle-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000001629*/snapshot.yaml'))
with (out/'native.log').open('w') as log:subprocess.run([str(out/'probe'),str(world),str(out/'report.yaml')],cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),copy_sha256=hashlib.sha256(copy.read_bytes()).hexdigest(),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),command=cmd,authority=False),indent=2)+'\n')
print((out/'native.log').read_text())
