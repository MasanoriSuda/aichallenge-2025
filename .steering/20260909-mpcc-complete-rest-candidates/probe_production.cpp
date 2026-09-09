// Exact production candidate entry, followed by the current-world join.
// This tool never publishes or mutates a Store; it is a native replay.
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_lattice_shadow.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"
#include "artifact_capture.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>
int main(int argc,char **argv) {
  if(argc!=4)return 2;
  namespace m=multi_purpose_mpc_ros;
  namespace a=m::mpcc_architecture_snapshot;
  namespace stop=m::mpcc_rate_resolved_stop_lattice_shadow;
  const auto config=YAML::LoadFile(argv[3]);
  std::string detail;
  auto record=a::load_recorded_interaction_snapshot(argv[1],&detail);
  if(!record)throw std::runtime_error(detail);
  m::mpcc_rate_resolved_shadow::SolverContext solver;
  const auto result=stop::evaluate_current_world(record->source,solver,{},stop::EvaluationMode::DirectSevenStateOnly);
  YAML::Node row;
  row["source_fingerprint"]=record->interaction_fingerprint;
  row["producer"]=stop::to_string(result.reason);
  row["producer_detail"]=result.detail;
  row["producer_ms"]=result.total_compute_ms;
  row["candidate_count"]=result.attempted_candidate_count;
  row["upstream_identity_preserved"]=m::mpcc_rate_resolved_execution_artifact::same_identity(result.source_normal_identity,record->source.identity);
  row["replay_assumptions"]=config;
  if(result.certified_stop_plan) {
    const auto plan=result.certified_stop_plan;
    const auto &candidate=*plan->solver_source_snapshot;
    const auto &world=*candidate.replay_world;
    row["candidate_fingerprint"]=a::fingerprint_interaction_snapshot(candidate);
    row["candidate_problem_fingerprint"]=candidate.identity.source_context.fingerprint;
    row["distinct_candidate_context"]=candidate.identity.source_context.fingerprint!=record->source.identity.source_context.fingerprint;
    capture_execution(*plan->execution_artifact,"artifact.yaml");
    const auto capture=a::record_proof_failure(candidate,a::PipelineStage::PhysicalProof,"offline-production-candidate-observation","Source capture only. This replay has no publication authority.","source");
    row["source_capture"]=capture.snapshot_file.string();
          namespace retained=m::mpcc_rate_resolved_retained_revalidation;
          retained::Request request;
          request.plan=plan;
          request.decision_id=candidate.identity.source_context.decision_id;
          request.now_sec=candidate.identity.snapshot_sec;
          request.control_origin_sec=candidate.control_prediction_origin_sec;
          request.execution_clock={retained::ExecutionClockKind::TimeAlignedCandidate,0.0,0.0};
          request.current_intent=candidate.identity.source_context.intent;
          request.control_origin_physical_progress_m=candidate.course_progress_origin_m+candidate.request.initial_state[1]+candidate.request.initial_state[4];
          request.path_length_m=1000.0;request.circular=true;request.progress_continuity_tolerance_m=1.5;
          request.measured_to_control_path=world.control_prefix;
          request.measured_to_control_elapsed_sec=world.control_prefix_elapsed_sec;
          request.control_pose=world.control_prefix.back();
          request.current_wall_grid=candidate.wall_grid;request.current_footprint=world.physical_footprint;
          request.obstacles={world.observation_generation,world.observed_sec,{},world.current};
          for(const auto & peer:world.obstacles){retained::DynamicObstacle o;o.id=peer.id;o.circle={peer.x_m,peer.y_m,peer.velocity_x_mps,peer.velocity_y_mps,peer.radius_m};request.obstacles.obstacles.push_back(o);}
          request.current_speed_mps=config["current_speed_mps"].as<double>();
          request.control_origin_speed_mps=candidate.request.initial_state[3];
          request.current_time_steering_rad=config["current_time_steering_rad"].as<double>();
          request.current_steering_rad=candidate.request.current_steering_rad;
          request.current_response_steering_rad=candidate.request.current_response_steering_rad;
          request.previous_published_steering_rad=request.current_steering_rad;
          request.previous_published_command_age_sec=0.02;
          request.stop_lateral_policy=world.terminal_stop_lateral_policy;
          request.minimum_acceleration_mps2=world.terminal_stop_minimum_acceleration_mps2;
          request.maximum_acceleration_mps2=1.37;
          const auto joined=retained::evaluate(request);
          row["join"]=retained::to_string(joined.reason);
          row["join_wall_scope"]=retained::to_string(joined.static_wall_scope);
          row["join_dynamic_scope"]=retained::to_string(joined.dynamic_obstacle_scope);
          row["join_continuation_scope"]=m::mpcc_rate_resolved_physical_adapter::to_string(joined.continuation_scope);
          row["join_wall_clear"]=joined.continuation_path_clearance.valid && joined.continuation_path_clearance.clear;
          row["join_terminal_dynamic_min"]=joined.terminal_stop_minimum_dynamic_clearance_m;
          row["join_terminal_wall_clear"]=joined.terminal_stop_path_clearance.valid && joined.terminal_stop_path_clearance.clear;
          if(joined.proof) {
            row["continuation_terminal_speed"]=joined.proof->continuation_trajectory.velocity_mps.back();
            row["solved_suffix_used"]=joined.terminal_stop_uses_solved_suffix;
            const auto command=m::mpcc_rate_resolved_production_adapter::build(joined);
            row["command_adapter_available"]=command.authority.has_value();
            if(command.authority)row["first_acceleration_mps2"]=command.authority->command.acceleration_mps2;
          }

  }
  row["authority"]=false;
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  e<<row;std::ofstream(argv[2])<<e.c_str()<<'\n';std::cout<<row<<'\n';
}
