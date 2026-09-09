#include "revalidation_input.hpp"
// Offline diagnostic: consume the recorded artifact directly, without a solve.
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"
#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <cmath>
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "mpcc_architecture_comparison.cpp"
#include "multi_purpose_mpc_ros/mpc_state_prediction.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_successor_bundle.hpp"
namespace m = multi_purpose_mpc_ros;
YAML::Node mpcc_diagnostic_terminal_traces(YAML::NodeType::Sequence);
int main(int argc, char **argv) {
  if (argc != 4) return 2;
  std::string detail;
  auto record = m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1], &detail);
  if (!record) throw std::runtime_error(detail);
  auto root = YAML::LoadFile(argv[1]);
  auto node = root["execution_evidence"];
  if (node["schema"].as<std::string>() != "mpcc-inspected-execution-artifact/v1") return 3;
  m::mpcc_rate_resolved_execution_artifact::ExecutionArtifact value;
  value.identity = record->source.identity;
  if (node["source_sequence"].as<uint64_t>() != value.identity.sequence ||
      node["source_problem_fingerprint"].as<uint64_t>() != value.identity.source_context.fingerprint ||
      node["source_snapshot_sec"].as<double>() != value.identity.snapshot_sec) return 4;
  value.prediction_origin_sec = node["prediction_origin_sec"].as<double>();
  value.publication_interval_sec = node["publication_interval_sec"].as<double>();
  value.completed_sec = node["completed_sec"].as<double>();
  value.course_progress_origin_m = node["course_progress_origin_m"].as<double>();
  value.semantic_initial_steering_rad = node["semantic_initial_steering_rad"].as<double>();
  value.semantic_initial_response_steering_rad = node["semantic_initial_response_steering_rad"].as<double>();
  value.wheelbase_m = node["wheelbase_m"].as<double>();
  value.yaw_response_gain = node["yaw_response_gain"].as<double>();
  value.yaw_response_time_constant_sec = node["yaw_response_time_constant_sec"].as<double>();
  value.minimum_frenet_denominator = node["minimum_frenet_denominator"].as<double>();
  value.maximum_abs_steering_rad = node["maximum_abs_steering_rad"].as<double>();
  value.maximum_abs_steering_rate_radps = node["maximum_abs_steering_rate_radps"].as<double>();
  value.physical_global_tolerance = node["physical_global_tolerance"].as<double>();
  value.maximum_constraint_violation = node["maximum_constraint_violation"].as<double>();
  value.maximum_normalized_constraint_violation = node["maximum_normalized_constraint_violation"].as<double>();
  value.terminal_intent_contract.active = node["terminal_intent_contract"]["active"].as<bool>();
  value.terminal_intent_contract.lateral_reference_m = node["terminal_intent_contract"]["lateral_reference_m"].as<double>();
  value.terminal_intent_contract.lateral_tolerance_m = node["terminal_intent_contract"]["lateral_tolerance_m"].as<double>();
  value.terminal_intent_contract.heading_reference_rad = node["terminal_intent_contract"]["heading_reference_rad"].as<double>();
  value.terminal_intent_contract.heading_tolerance_rad = node["terminal_intent_contract"]["heading_tolerance_rad"].as<double>();
  value.terminal_intent_certificate.active = node["terminal_intent_certificate"]["active"].as<bool>();
  value.terminal_intent_certificate.solved_horizon_steps = node["terminal_intent_certificate"]["solved_horizon_steps"].as<std::size_t>();
  value.terminal_intent_certificate.solved_lateral_m = node["terminal_intent_certificate"]["solved_lateral_m"].as<double>();
  value.terminal_intent_certificate.solved_heading_rad = node["terminal_intent_certificate"]["solved_heading_rad"].as<double>();
  for (const auto & item : node["predicted_states"]) {
    m::mpcc_rate_resolved_execution_artifact::PredictedState entry;
    entry.lateral_m = item["lateral_m"].as<double>();
    entry.lag_m = item["lag_m"].as<double>();
    entry.heading_offset_rad = item["heading_offset_rad"].as<double>();
    entry.velocity_mps = item["velocity_mps"].as<double>();
    entry.progress_m = item["progress_m"].as<double>();
    entry.steering_rad = item["steering_rad"].as<double>();
    entry.response_steering_rad = item["response_steering_rad"].as<double>();
    value.predicted_states.push_back(entry);
  }
  for (const auto & item : node["control_stages"]) {
    m::mpcc_rate_resolved_execution_artifact::ControlStage entry;
    entry.acceleration_mps2 = item["acceleration_mps2"].as<double>();
    entry.steering_rate_radps = item["steering_rate_radps"].as<double>();
    entry.virtual_progress_speed_mps = item["virtual_progress_speed_mps"].as<double>();
    entry.duration_sec = item["duration_sec"].as<double>();
    entry.virtual_progress_lower_mps = item["virtual_progress_lower_mps"].as<double>();
    entry.virtual_progress_upper_mps = item["virtual_progress_upper_mps"].as<double>();
    entry.acceleration_lower_mps2 = item["acceleration_lower_mps2"].as<double>();
    entry.acceleration_upper_mps2 = item["acceleration_upper_mps2"].as<double>();
    entry.path_curvature_radpm = item["path_curvature_radpm"].as<double>();
    value.control_stages.push_back(entry);
  }
  value.course_frame = record->source.request.course_frame;
  if (!node["source_course_frame_bound"].as<bool>() ||
      node["coordinate_state_schema"].as<std::string>() != value.identity.source_context.state_schema_id ||
      node["course_frame_progress_origin_m"].as<double>() != value.course_progress_origin_m)
    throw std::runtime_error("incompatible recorded coordinate identity");
  const auto initial = node["semantic_initial_state"];
  value.semantic_initial_state = m::mpcc_rate_resolved_execution_artifact::PredictedState{
    initial["lateral_m"].as<double>(), initial["lag_m"].as<double>(),
    initial["heading_offset_rad"].as<double>(), initial["velocity_mps"].as<double>(),
    initial["progress_m"].as<double>(), initial["steering_rad"].as<double>(),
    initial["response_steering_rad"].as<double>()};
  value.nominal_path_distance_m = node["nominal_path_distance_m"].as<std::vector<double>>();
  value.lateral_lower_m = node["lateral_lower_m"].as<std::vector<double>>();
  value.lateral_upper_m = node["lateral_upper_m"].as<std::vector<double>>();
  auto result = m::mpcc_rate_resolved_physical_adapter::build(value, value.identity.source_context.intent, value.identity.source_context.stage_geometry_id);
  YAML::Node output;
  output["input"] = argv[1];
  output["solver_invocations"] = 0;
  output["trajectory_role"] = "inspected-execution-not-publication";
  output["reason"] = m::mpcc_rate_resolved_physical_adapter::to_string(result.reason);
  output["artifact_reason"] = m::mpcc_rate_resolved_execution_artifact::to_string(result.artifact_reason);
  output["prediction_origin_sec"] = value.prediction_origin_sec;
  output["points"] = YAML::Node(YAML::NodeType::Sequence);
  auto append = [&](double t, double progress, double lateral, double lag, double heading, double speed) {
    auto frame = m::mpc_stage_geometry::sample_course_frame(record->source.wall_course_frame_knots, progress, 1e-9);
    if (!frame) throw std::runtime_error("course frame unavailable");
    auto pose = m::mpcc_execution_contract::reconstruct_planar_pose_from_frenet(
      {frame->x_m, frame->y_m, frame->heading_rad}, {lateral, lag, heading});
    if (!pose) throw std::runtime_error("pose unavailable");
    YAML::Node row;
    row["elapsed_sec"] = t;
    row["source_prediction_time_sec"] = value.prediction_origin_sec + t;
    row["x_m"] = pose->x_m; row["y_m"] = pose->y_m; row["yaw_rad"] = pose->yaw_rad;
    row["velocity_mps"] = speed; row["progress_m"] = progress; row["lag_m"] = lag;
    row["lateral_m"] = lateral; row["heading_offset_rad"] = heading;
    output["points"].push_back(row);
  };
  if (result.exact_trajectory) {
    const auto & first = *value.semantic_initial_state;
    append(0., value.course_progress_origin_m + first.progress_m, first.lateral_m, first.lag_m, first.heading_offset_rad, first.velocity_mps);
    const auto & exact = *result.exact_trajectory;
    for (std::size_t i = 0; i < exact.elapsed_time_sec.size(); ++i)
      append(exact.elapsed_time_sec[i], exact.progress_m[i], exact.lateral_m[i], exact.lag_m[i], exact.heading_offset_rad[i], exact.velocity_mps[i]);
  }
  if (result.exact_trajectory) {
    namespace cmp = m::mpcc_architecture_comparison;
    namespace retained = m::mpcc_rate_resolved_retained_revalidation;
    namespace certified = m::mpcc_rate_resolved_certified_plan;
    const auto physical = cmp::wall_snapshot(record->source, *record->source.replay_world, *result.exact_trajectory);
    const auto wall = m::mpcc_rate_resolved_physical_wall::evaluate(physical);
    const auto dynamic = m::mpcc_rate_resolved_dynamic_proof::evaluate_current_world(record->source, physical);
    const auto plan = certified::build(
      std::make_shared<const m::mpcc_rate_resolved_execution_artifact::ExecutionArtifact>(value), physical, wall,
      std::make_shared<const m::mpcc_rate_resolved_shadow::Snapshot>(record->source));
    output["original_execution"]["wall"] = m::mpcc_rate_resolved_physical_wall::to_string(wall.outcome);
    output["original_execution"]["dynamic_clear"] = dynamic.valid && dynamic.clear;
    output["original_execution"]["dynamic_clearance_m"] = dynamic.minimum_clearance_m;
    output["original_execution"]["terminal_speed_mps"] = result.exact_trajectory->velocity_mps.back();
    output["original_execution"]["rest_tolerance_mps"] = std::max(1e-9,value.physical_global_tolerance);
    output["original_execution"]["certified_reason"] = certified::to_string(plan.reason);
    if (!plan.plan) throw std::runtime_error("original published artifact cannot be certified");
    const auto failure=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[3],&detail);
    if(!failure)throw std::runtime_error(detail);
    const auto failure_node=YAML::LoadFile(argv[3]);
    const auto observation=failure_node["revalidation_evidence"];
    if(observation["status"].as<std::string>()!="present" || observation["inspected_plan_status"].as<std::string>()!="present")
      throw std::runtime_error("complete current request and inspected plan required; no substitution");
    auto request=mpcc_observation::read_request(observation["request"],std::filesystem::path(argv[3]).parent_path());
    if(request.decision_id!=failure->source.identity.source_context.decision_id ||
      request.now_sec!=failure->source.identity.snapshot_sec || request.control_origin_sec!=failure->source.control_prediction_origin_sec)
      throw std::runtime_error("failure decision/time association mismatch");
    request.plan=plan.plan;
    const auto current=retained::evaluate(request);
    const auto stop=retained::evaluate_stop_successor(request);
    output["failure_world_fingerprint"]=failure->interaction_fingerprint;
    output["failure_outcome"]=failure_node["failure_outcome"];
    output["failure_detail"]=failure_node["failure_detail"];
    auto n=output["current_revalidation"];
    n["decision_id"]=request.decision_id;n["inspected_sequence"]=value.identity.sequence;
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
    output["authority"]=false;

  }
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << output;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << output["current_revalidation"] << '\n';
}
