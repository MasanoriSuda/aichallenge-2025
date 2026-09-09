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
  if (node["schema"].as<std::string>() != "mpcc-published-execution-evidence/v1") return 3;
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
  output["trajectory_role"] = node["publication"]["source_kind"].as<std::string>() == "exact-executed" ? "exact-executed-source" : "bundle-source-only-not-the-published-bundle";
  output["reason"] = m::mpcc_rate_resolved_physical_adapter::to_string(result.reason);
  output["artifact_reason"] = m::mpcc_rate_resolved_execution_artifact::to_string(result.artifact_reason);
  output["publication"] = node["publication"];
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
    row["publication_aligned_time_sec"] = node["publication"]["publication_control_origin_sec"].as<double>() + t - node["publication"]["publication_artifact_elapsed_sec"].as<double>();
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
    const auto config = YAML::LoadFile(argv[3]);
    const auto current = m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(config["current_world"].as<std::string>(),&detail);
    if (!current || !current->source.replay_world) throw std::runtime_error("missing current world/"+detail);
    const auto & world = *current->source.replay_world;
    retained::Request request;
    request.plan = plan.plan;
    request.decision_id = node["publication"]["failure_decision_id"].as<uint64_t>();
    request.now_sec = node["publication"]["failure_observation_sec"].as<double>();
    request.control_origin_sec = node["publication"]["failure_control_origin_sec"].as<double>();
    if (current->source.identity.source_context.decision_id != request.decision_id ||
        current->interaction_fingerprint != node["publication"]["failure_interaction_fingerprint"].as<uint64_t>() ||
        current->source.identity.snapshot_sec != request.now_sec)
      throw std::runtime_error("failure world identity mismatch");
    request.execution_clock = {retained::ExecutionClockKind::PublishedPlan,
      node["publication"]["publication_control_origin_sec"].as<double>(),
      node["publication"]["publication_artifact_elapsed_sec"].as<double>()};
    request.current_intent = value.identity.source_context.intent;
    request.control_origin_physical_progress_m = config["physical_progress_m"].as<double>();
    request.path_length_m = config["path_length_m"].as<double>();
    request.progress_continuity_tolerance_m = 1.5;
    request.circular = true;
    request.measured_to_control_path = world.control_prefix;
    request.measured_to_control_elapsed_sec = world.control_prefix_elapsed_sec;
    request.control_pose = world.control_prefix.back();
    request.current_wall_grid = current->source.wall_grid;
    request.current_footprint = world.physical_footprint;
    request.obstacles.generation = world.observation_generation;
    request.obstacles.observed_sec = world.observed_sec;
    request.obstacles.current = world.current;
    for (const auto & item : world.obstacles) {
      retained::DynamicObstacle obstacle;
      obstacle.id=item.id; obstacle.circle={item.x_m,item.y_m,item.velocity_x_mps,item.velocity_y_mps,item.radius_m};
      request.obstacles.obstacles.push_back(obstacle);
    }
    request.current_speed_mps = config["current_speed_mps"].as<double>();
    request.control_origin_speed_mps = current->source.request.initial_state[3];
    request.current_time_steering_rad = config["current_time_steering_rad"].as<double>();
    request.current_steering_rad = current->source.request.current_steering_rad;
    request.current_response_steering_rad = current->source.request.current_response_steering_rad;
    request.previous_published_steering_rad = request.current_steering_rad;
    request.previous_published_command_age_sec = config["previous_published_command_age_sec"].as<double>();
    request.stop_lateral_policy = world.terminal_stop_lateral_policy;
    request.minimum_acceleration_mps2 = world.terminal_stop_minimum_acceleration_mps2;
    request.maximum_acceleration_mps2 = config["maximum_acceleration_mps2"].as<double>();
    output["replay_assumptions"] = config;
    const auto observe = [&](const retained::Request & r, const std::string & name) {
      const auto proof = retained::evaluate(r);
      const auto successor = retained::evaluate_stop_successor(r);
      auto n=output[name];
      n["reason"]=retained::to_string(proof.reason); n["accepted"]=proof.proof.has_value();
      n["successor_reason"]=retained::to_string(successor.reason);
      if(std::isfinite(successor.initial_course_progress_m)) {
        const auto frame=m::mpc_stage_geometry::sample_course_frame(r.plan->physical_snapshot->course_frame_knots,successor.initial_course_progress_m,1e-9);
        if(frame) {
          const auto pose=m::mpcc_execution_contract::reconstruct_planar_pose_from_frenet({frame->x_m,frame->y_m,frame->heading_rad},{successor.initial_lateral_m,successor.initial_lag_m,successor.initial_heading_offset_rad});
          if(pose)n["successor_initial_pose_error_m"]=std::hypot(pose->x_m-r.control_pose.x_m,pose->y_m-r.control_pose.y_m);
        }
      }

      if (successor.accepted()) {
        const auto bundle = m::mpcc_rate_resolved_stop_successor_bundle::build(r,successor,999999);
        n["successor_bundle_reason"]=m::mpcc_rate_resolved_stop_successor_bundle::to_string(bundle.reason);
        if (bundle.plan) {
          auto joined=r;joined.plan=bundle.plan;
          joined.execution_clock={retained::ExecutionClockKind::TimeAlignedCandidate,0.0,0.0};
          const auto reproof=retained::evaluate(joined);
          n["successor_bundle_join"]=retained::to_string(reproof.reason);
          const auto & art=*bundle.plan->execution_artifact;
          const auto & st=*art.semantic_initial_state;
          const auto frame=m::mpc_stage_geometry::sample_course_frame(art.course_frame.knots ? *art.course_frame.knots : bundle.plan->physical_snapshot->course_frame_knots,art.course_progress_origin_m+st.progress_m,1e-9);
          if(frame) {
            const auto pose=m::mpcc_execution_contract::reconstruct_planar_pose_from_frenet({frame->x_m,frame->y_m,frame->heading_rad},{st.lateral_m,st.lag_m,st.heading_offset_rad});
            if(pose){n["bundle_initial_pose_error_m"]=std::hypot(pose->x_m-r.control_pose.x_m,pose->y_m-r.control_pose.y_m);n["bundle_initial_yaw_error_rad"]=pose->yaw_rad-r.control_pose.yaw_rad;}
          }
          n["bundle_initial_progress_m"]=art.course_progress_origin_m+st.progress_m;
          n["bundle_initial_lag_m"]=st.lag_m;
          n["joined_initial_progress_m"]=art.course_progress_origin_m+reproof.current_control_state.progress_m;
          n["joined_initial_lag_m"]=reproof.current_control_state.lag_m;
          n["bundle_static_scope"]=retained::to_string(reproof.static_wall_scope);
          n["bundle_wall_index"]=reproof.continuation_path_clearance.rejected_path_index;

          n["successor_bundle_continuation"]=m::mpcc_rate_resolved_physical_adapter::to_string(reproof.continuation_reason);
          n["successor_bundle_scope"]=m::mpcc_rate_resolved_physical_adapter::to_string(reproof.continuation_scope);
          n["successor_bundle_wall_clear"]=reproof.continuation_path_clearance.valid && reproof.continuation_path_clearance.clear;
          n["successor_bundle_terminal_wall_clear"]=reproof.terminal_stop_path_clearance.valid && reproof.terminal_stop_path_clearance.clear;
          n["successor_bundle_terminal_wall_index"]=reproof.terminal_stop_path_clearance.rejected_path_index;
          n["successor_bundle_terminal_dynamic_min"]=reproof.terminal_stop_minimum_dynamic_clearance_m;
        }
      }

      n["cursor_elapsed_sec"]=proof.cursor_elapsed_sec;
      n["control_origin_speed_mps"]=r.control_origin_speed_mps;
      n["expected_speed_mps"]=proof.expected_speed_mps;
      n["pose_error_m"]=proof.control_pose_error_m;
      n["delay_clear"]=proof.delay_path_clearance.valid && proof.delay_path_clearance.clear;
      n["publisher_interval_clear"]=proof.publisher_interval_path_clearance.valid && proof.publisher_interval_path_clearance.clear;
      n["continuation_clear"]=proof.continuation_path_clearance.valid && proof.continuation_path_clearance.clear;
      n["terminal_stop_certified"]=proof.terminal_stop_certified;
      n["terminal_stop_reason"]=m::mpcc_rate_resolved_physical_adapter::to_string(proof.terminal_stop_reason);
      n["terminal_stop_reference_attempts"]=proof.terminal_stop_reference_attempts;
      n["terminal_wall_clear"]=proof.terminal_stop_path_clearance.valid && proof.terminal_stop_path_clearance.clear;
      n["minimum_dynamic_clearance_m"]=proof.minimum_dynamic_clearance_m;
      n["terminal_minimum_dynamic_clearance_m"]=proof.terminal_stop_minimum_dynamic_clearance_m;
      n["terminal_dynamic_blocker"]=proof.terminal_stop_blocking_obstacle_id;
      n["terminal_dynamic_checked_pose_count"]=proof.terminal_stop_dynamic_checked_pose_count;
    };
    observe(request,"recorded_current_world");
  }
  output["terminal_traces"]=mpcc_diagnostic_terminal_traces;
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << output;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << output["reason"].as<std::string>() << ", points=" << output["points"].size() << '\n';
  return result.exact_trajectory ? 0 : 5;
}
