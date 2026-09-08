"""Zero-solve replay with observation-only instrumentation of a source copy.

Never overwrite production/build source. Preserve the complete generated copy
and hashes so the two inserted diagnostic blocks are reviewable.
"""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess
import yaml

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
steering = Path('/task-steering/20260909-mpcc-complete-stop-suffix')
run = Path('/output/20260909-rear-peer-dev2-r1')
out = Path('/output/20260909-complete-stop-968-replay-after')
out.mkdir(exist_ok=False)
source = package/'src/mpcc_rate_resolved_retained_revalidation.cpp'
instrumented = '#include <yaml-cpp/yaml.h>\nextern YAML::Node mpcc_diagnostic_terminal_traces;\n'+source.read_text()
anchor = '    dynamic_proof::finalize(request.obstacles, terminal_stop_dynamic);'
assert instrumented.count(anchor) == 1
probe = r'''
    // Offline observation only: read the already built full stopping trajectory.
    YAML::Node diagnostic;
    diagnostic["normal_path_reference"] = stop_profile != nullptr;
    diagnostic["minimum_clearance_m"] = terminal_stop_dynamic.minimum_clearance_m;
    diagnostic["minimum_elapsed_sec"] = terminal_stop_dynamic.minimum_clearance_elapsed_sec;
    diagnostic["minimum_pose"]["x_m"] = terminal_stop_dynamic.minimum_clearance_pose.x_m;
    diagnostic["minimum_pose"]["y_m"] = terminal_stop_dynamic.minimum_clearance_pose.y_m;
    diagnostic["minimum_pose"]["yaw_rad"] = terminal_stop_dynamic.minimum_clearance_pose.yaw_rad;
    diagnostic["checked_pose_count"] = terminal_stop_dynamic.checked_pose_count;
    diagnostic["valid"] = terminal_stop_dynamic.valid;
    diagnostic["clear"] = terminal_stop_dynamic.clear;
    diagnostic["blocker"] = terminal_stop_dynamic.blocking_obstacle_id;
    diagnostic["full_path"] = YAML::Node(YAML::NodeType::Sequence);
    for (std::size_t i = 0; i < terminal_stop_trajectory.elapsed_time_sec.size(); ++i) {
      const auto endpoint = exact_physical_state_at(terminal_stop_trajectory, i);
      const auto state = as_predicted_state(endpoint, current_control_state, execution.course_progress_origin_m);
      const auto pose = reconstruct_pose(source, state);
      if (!pose) throw std::runtime_error("diagnostic full stop pose unavailable");
      YAML::Node point;
      point["elapsed_sec"] = prediction_delay_sec + terminal_stop_trajectory.elapsed_time_sec[i];
      point["x_m"] = pose->x_m; point["y_m"] = pose->y_m; point["yaw_rad"] = pose->yaw_rad;
      point["velocity_mps"] = terminal_stop_trajectory.velocity_mps[i];
      diagnostic["full_path"].push_back(point);
    }
    mpcc_diagnostic_terminal_traces.push_back(diagnostic);
'''
instrumented = instrumented.replace(anchor, anchor+probe)
copy = out/'retained_revalidation_instrumented.cpp'
copy.write_text(instrumented)
world_path = next((run/'d2/mpcc_architecture_snapshots').glob('000000000968*/snapshot.yaml'))
world = yaml.safe_load(world_path.read_text())
state = world['source']['semantic_request']['initial_state']
config = dict(current_world=str(world_path),
    physical_progress_m=world['source']['course_progress_origin_m']+state[1]+state[4],
    path_length_m=1000.0, current_speed_mps=1.762816,
    current_time_steering_rad=-0.278635, previous_published_command_age_sec=0.02,
    maximum_acceleration_mps2=1.37,
    limits='Current physical steering rounded from controller log to six decimals; current speed rounded from same-decision runtime log; publication age from matching command stamp. Path length 1000m is unused non-wrap surrogate, no lap alignment here. Full stop trajectory is diagnostic only and never grants authority.')
(out/'input.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
link = shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command = [link[0], '-std=c++17','-O2','-I'+str(package/'include'),'-I'+str(package/'src'),'-I/usr/include/eigen3',
    str(steering/'replay_968.cpp'),str(copy),'-o',str(out/'reconstruct_execution'),
    'libmulti_purpose_mpc_ros_mpcc_rate_resolved_stop_successor_bundle.a','libmulti_purpose_mpc_ros_mpcc_latest_state_feedback.a','libmulti_purpose_mpc_ros_mpc_state_prediction.so',
    'libmulti_purpose_mpc_ros_mpcc_rate_resolved_certified_plan.a']+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
original = run/'d2-executed-354/snapshot.yaml'
with (out/'native.log').open('w') as log:
    subprocess.run([str(out/'reconstruct_execution'),str(original),str(out/'report.yaml'),str(out/'input.yaml')],
        cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
(out/'manifest.json').write_text(json.dumps(dict(solver_invocations=0,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    instrumented_copy_sha256=hashlib.sha256(copy.read_bytes()).hexdigest(),
    world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest(),
    published_source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
    authority=False,command=command),indent=2)+'\n')
print(out/'report.yaml',flush=True)
