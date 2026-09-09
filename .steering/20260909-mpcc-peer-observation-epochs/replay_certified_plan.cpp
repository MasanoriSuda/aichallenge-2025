#include "certified_plan_input.hpp"
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include <cmath>
#include <iostream>
namespace m=multi_purpose_mpc_ros;
namespace retained=m::mpcc_rate_resolved_retained_revalidation;
namespace physical=m::mpcc_rate_resolved_physical_wall;
int main(int argc,char **argv) {
  if(argc!=4)return 2;
  const std::filesystem::path path(argv[1]);const std::string key(argv[3]);
  if(key!="publication_bundle" && key!="revalidation_evidence" && key!="previous_accepted_revalidation_evidence")return 2;
  const auto root=YAML::LoadFile(path.string());
  const auto observation=root[key];
  const auto plan=mpcc_observation::read_source_free_plan(observation["certified_plan_evidence"],path.parent_path());
  const auto fresh_wall=physical::evaluate(*plan->physical_snapshot);
  mpcc_observation::require(fresh_wall.outcome==plan->physical_outcome && fresh_wall.diagnostic.reason==plan->physical_diagnostic.reason,"recorded wall proof does not reproduce");
  const auto eq=[](double a,double b){return a==b || (std::isnan(a) && std::isnan(b));};
  mpcc_observation::require(fresh_wall.diagnostic.stage_index==plan->physical_diagnostic.stage_index,"wall diagnostic mismatch: stage_index");
  mpcc_observation::require(fresh_wall.diagnostic.waypoint_id==plan->physical_diagnostic.waypoint_id,"wall diagnostic mismatch: waypoint_id");
  mpcc_observation::require(eq(fresh_wall.diagnostic.path_distance_m,plan->physical_diagnostic.path_distance_m),"wall diagnostic mismatch: path_distance_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.lateral_m,plan->physical_diagnostic.lateral_m),"wall diagnostic mismatch: lateral_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.lag_m,plan->physical_diagnostic.lag_m),"wall diagnostic mismatch: lag_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.lower_bound_m,plan->physical_diagnostic.lower_bound_m),"wall diagnostic mismatch: lower_bound_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.upper_bound_m,plan->physical_diagnostic.upper_bound_m),"wall diagnostic mismatch: upper_bound_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.bound_reserve_m,plan->physical_diagnostic.bound_reserve_m),"wall diagnostic mismatch: bound_reserve_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.heading_offset_rad,plan->physical_diagnostic.heading_offset_rad),"wall diagnostic mismatch: heading_offset_rad");
  mpcc_observation::require(eq(fresh_wall.diagnostic.reference_progress_m,plan->physical_diagnostic.reference_progress_m),"wall diagnostic mismatch: reference_progress_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.solved_progress_m,plan->physical_diagnostic.solved_progress_m),"wall diagnostic mismatch: solved_progress_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.progress_delta_m,plan->physical_diagnostic.progress_delta_m),"wall diagnostic mismatch: progress_delta_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.pose_x_m,plan->physical_diagnostic.pose_x_m),"wall diagnostic mismatch: pose_x_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.pose_y_m,plan->physical_diagnostic.pose_y_m),"wall diagnostic mismatch: pose_y_m");
  mpcc_observation::require(eq(fresh_wall.diagnostic.pose_yaw_rad,plan->physical_diagnostic.pose_yaw_rad),"wall diagnostic mismatch: pose_yaw_rad");
  mpcc_observation::require(fresh_wall.diagnostic.out_of_map==plan->physical_diagnostic.out_of_map,"wall diagnostic mismatch: out_of_map");
  mpcc_observation::require(fresh_wall.diagnostic.contact_cell_count==plan->physical_diagnostic.contact_cell_count,"wall diagnostic mismatch: contact_cell_count");
  mpcc_observation::require(fresh_wall.diagnostic.swept_rejected_path_index==plan->physical_diagnostic.swept_rejected_path_index,"wall diagnostic mismatch: swept_rejected_path_index");
  mpcc_observation::require(fresh_wall.diagnostic.swept_checked_pose_count==plan->physical_diagnostic.swept_checked_pose_count,"wall diagnostic mismatch: swept_checked_pose_count");
  mpcc_observation::require(fresh_wall.diagnostic.swept_rejected_substep==plan->physical_diagnostic.swept_rejected_substep,"wall diagnostic mismatch: swept_rejected_substep");
  mpcc_observation::require(fresh_wall.diagnostic.swept_rejected_subdivision_count==plan->physical_diagnostic.swept_rejected_subdivision_count,"wall diagnostic mismatch: swept_rejected_subdivision_count");
  mpcc_observation::require(eq(fresh_wall.diagnostic.swept_rejected_segment_ratio,plan->physical_diagnostic.swept_rejected_segment_ratio),"wall diagnostic mismatch: swept_rejected_segment_ratio");
  YAML::Node output;
  output["input"]=path.string();output["role"]=key;
  output["authority"]=false;output["solver_invocations"]=0;
  output["source_sequence"]=plan->execution_artifact->identity.sequence;
  output["source_problem_fingerprint"]=plan->execution_artifact->identity.source_context.fingerprint;
  output["source_snapshot_sec"]=plan->execution_artifact->identity.snapshot_sec;
  output["solver_source_status"]="absent";
  output["original_physical_wall_proof"]=physical::to_string(fresh_wall.outcome);
  output["wall_diagnostic_all_fields_equal"]=true;
  output["parent_dynamic_proof"]="not recorded; no inference";
  output["original_terminal_speed_mps"]=plan->physical_snapshot->trajectory.velocity_mps.back();
  if(key!="publication_bundle") {
    mpcc_observation::require(observation["status"].as<std::string>()=="present","recorded Request association invalid");
    auto request=mpcc_observation::read_request(observation["request"],path.parent_path());
    if(key=="revalidation_evidence") {
      std::string detail;
      auto world=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(path,&detail);
      mpcc_observation::require(world.has_value(),detail.c_str());
      mpcc_observation::require(request.decision_id==world->source.identity.source_context.decision_id &&
        request.now_sec==world->source.identity.snapshot_sec && request.control_origin_sec==world->source.control_prediction_origin_sec,"Request/failure world identity mismatch");
      output["failure_interaction_fingerprint"]=world->interaction_fingerprint;
    }
    request.plan=plan;
    const auto current=retained::evaluate(request);
    const auto stop=retained::evaluate_stop_successor(request);
    auto n=output["current_revalidation"];
    n["decision_id"]=request.decision_id;n["inspected_sequence"]=plan->execution_artifact->identity.sequence;
    n["reason"]=retained::to_string(current.reason);n["proof_available"]=current.proof.has_value();
    n["clock_kind"]=retained::to_string(current.execution_clock_kind);n["cursor_elapsed_sec"]=current.cursor_elapsed_sec;
    n["physical_progress_delta_m"]=current.progress_difference_m;n["pose_error_m"]=current.control_pose_error_m;n["yaw_error_rad"]=current.control_yaw_error_rad;
    n["current_speed_mps"]=request.current_speed_mps;n["control_speed_mps"]=request.control_origin_speed_mps;n["expected_speed_mps"]=current.expected_speed_mps;
    n["physical_steering_now_rad"]=request.current_time_steering_rad;n["previous_steering_rad"]=request.previous_published_steering_rad;
    n["previous_command_age_sec"]=request.previous_published_command_age_sec;n["expected_steering_rad"]=current.expected_steering_rad;
    n["steering_delta_rad"]=current.steering_difference_rad;n["steering_lower_rad"]=current.reachable_steering_lower_rad;
    n["steering_upper_rad"]=current.reachable_steering_upper_rad;n["steering_duration_sec"]=current.steering_reachability_duration_sec;
    n["continuation_model"]=m::mpcc_rate_resolved_physical_adapter::to_string(current.continuation_reason);
    n["continuation_exact"]=m::race_mpcc_foundation::exact_physical_execution_trajectory_reason_name(current.continuation_exact_reason);
    n["continuation_scope"]=m::mpcc_rate_resolved_physical_adapter::to_string(current.continuation_scope);
    n["continuation_wall_clear"]=current.continuation_path_clearance.valid && current.continuation_path_clearance.clear;
    n["continuation_wall_reject_index"]=current.continuation_path_clearance.rejected_path_index;
    n["terminal_attempted"]=current.terminal_stop_attempted;n["terminal_certified"]=current.terminal_stop_certified;
    n["terminal_uses_solved_suffix"]=current.terminal_stop_uses_solved_suffix;
    n["terminal_reference_attempts"]=current.terminal_stop_reference_attempts;
    n["terminal_wall_valid"]=current.terminal_stop_path_clearance.valid;n["terminal_wall_clear"]=current.terminal_stop_path_clearance.clear;
    n["terminal_wall_reject_index"]=current.terminal_stop_path_clearance.rejected_path_index;
    n["terminal_dynamic_min_m"]=current.terminal_stop_minimum_dynamic_clearance_m;
    n["terminal_dynamic_blocker"]=current.terminal_stop_blocking_obstacle_id;
    n["independent_stop"]=retained::to_string(stop.reason);
    n["independent_stop_wall_clear"]=stop.successor_path_clearance.valid && stop.successor_path_clearance.clear;
    n["independent_stop_dynamic_clear"]=stop.dynamic_clearance.valid && stop.dynamic_clearance.clear;
    n["independent_stop_dynamic_min_m"]=stop.dynamic_clearance.minimum_clearance_m;
  }
  YAML::Emitter emitter;emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << output;std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
}
