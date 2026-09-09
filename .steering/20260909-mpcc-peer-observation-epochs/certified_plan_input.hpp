#pragma once
// Offline-only decoding. No solve, fabricated parent, or execution authority.
#include "../20260909-mpcc-publication-clock-handoff/revalidation_input.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
namespace mpcc_observation {
namespace physical=m::mpcc_rate_resolved_physical_wall;
namespace certified=m::mpcc_rate_resolved_certified_plan;
inline void require(bool ok,const char * why) { if(!ok)throw std::runtime_error(why); }
inline m::mpcc_execution_contract::ControlIntent read_intent(const std::string & name) {
  using I=m::mpcc_execution_contract::ControlIntent;
  for(auto i:{I::Track,I::Cruise,I::Follow,I::Hold,I::Stop,I::ShiftOut,I::Pass,I::Return,I::Rejoin})
    if(name==m::mpcc_execution_contract::to_string(i))return i;
  throw std::runtime_error("unknown recorded intent");
}
inline std::vector<m::mpc_stage_geometry::CourseFrameKnot> read_knots(const YAML::Node & n) {
  std::vector<m::mpc_stage_geometry::CourseFrameKnot> result;
  for(const auto & k:n) result.push_back({k["progress_m"].as<double>(),k["x_m"].as<double>(),k["y_m"].as<double>(),k["heading_rad"].as<double>(),k["waypoint"].as<int>()});
  return result;
}
inline m::recovery_footprint::Pose2D read_pose(const YAML::Node & n) {
  return {n["x_m"].as<double>(),n["y_m"].as<double>(),n["yaw_rad"].as<double>()};
}
inline std::shared_ptr<const certified::CertifiedPlan> read_source_free_plan(
  const YAML::Node & evidence,const std::filesystem::path & directory) {
  require(evidence["schema"].as<std::string>()=="mpcc-certified-plan-observation/v1" &&
    evidence["status"].as<std::string>()=="present" && !evidence["authority"].as<bool>(),"absent/invalid physical plan observation");
  require(evidence["solver_source_status"].as<std::string>()=="absent","source-free decoder cannot discard a present solver source");
  const auto node=evidence["artifact"];
  require(node["schema"].as<std::string>()=="mpcc-inspected-execution-artifact/v1","artifact schema");
  m::mpcc_rate_resolved_execution_artifact::ExecutionArtifact value;
  value.identity.sequence=node["source_sequence"].as<std::uint64_t>();
  value.identity.snapshot_sec=node["source_snapshot_sec"].as<double>();
  auto & context=value.identity.source_context;
  const auto problem_context=node["problem_context"];
  context.decision_id = problem_context["decision_id"].as<std::uint64_t>();
  context.intent = read_intent(problem_context["intent"].as<std::string>());
  context.intent_generation =
    problem_context["intent_generation"].as<std::uint64_t>();
  context.observation_generation =
    problem_context["observation_generation"].as<std::uint64_t>();
  context.stage_geometry_id =
    problem_context["stage_geometry_id"].as<std::uint64_t>();
  context.target_obstacle_generation =
    problem_context["target_obstacle_generation"].as<std::uint64_t>();
  context.target_id = problem_context["target_id"].as<std::string>();
  context.execution_side_sign = problem_context["execution_side_sign"].as<int>();
  if (problem_context["dynamic_obstacle_constraint_active"]) {
    context.dynamic_obstacle_constraint_active =
      problem_context["dynamic_obstacle_constraint_active"].as<bool>();
    context.dynamic_obstacle_generation =
      problem_context["dynamic_obstacle_generation"].as<std::uint64_t>();
    context.dynamic_obstacle_id =
      problem_context["dynamic_obstacle_id"].as<std::string>();
    context.dynamic_obstacle_side_sign =
      problem_context["dynamic_obstacle_side_sign"].as<int>();
  }
  context.horizon_steps = problem_context["horizon_steps"].as<std::size_t>();
  context.formulation = m::mpcc_execution_contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = problem_context["state_schema_id"].as<std::string>();
  context.input_schema_id = problem_context["input_schema_id"].as<std::string>();
  context.bounds_schema_id = problem_context["bounds_schema_id"].as<std::string>();
  context.cost_schema_id = problem_context["cost_schema_id"].as<std::string>();
  context.fingerprint = problem_context["fingerprint"].as<std::uint64_t>();
  require(problem_context["formulation"].as<std::string>()==m::mpcc_execution_contract::to_string(context.formulation),"unsupported formulation");
  require(m::mpcc_execution_contract::seal_problem_context(context).fingerprint==context.fingerprint && context.fingerprint==node["source_problem_fingerprint"].as<std::uint64_t>(),"context fingerprint mismatch");
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
  value.course_frame.progress_origin_m = node["course_frame_progress_origin_m"].as<double>();
  value.course_frame.knots = std::make_shared<const std::vector<m::mpc_stage_geometry::CourseFrameKnot>>(read_knots(node["course_frame_knots"]));
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

  const auto identity=[&](const YAML::Node & n) {
    physical::Identity result;
    result.artifact=value.identity;
    require(n["artifact_sequence"].as<std::uint64_t>()==value.identity.sequence &&
      n["artifact_problem_fingerprint"].as<std::uint64_t>()==context.fingerprint &&
      n["artifact_snapshot_sec"].as<double>()==value.identity.snapshot_sec,"physical/artifact identity mismatch");
    result.pose_snapshot_id=n["pose_snapshot_id"].as<std::uint64_t>();
    result.course_frame_window_id=n["course_frame_window_id"].as<std::uint64_t>();
    result.captured_sec=n["captured_sec"].as<double>();return result;
  };
  const auto n=evidence["physical_snapshot"];
  physical::Snapshot snapshot;
  snapshot.identity=identity(n["identity"]);
  snapshot.current_pose=read_pose(n["current_pose"]);
  for(const auto & pose:n["control_prefix"])snapshot.control_prefix.push_back(read_pose(pose));
  snapshot.course_frame_knots=read_knots(n["course_frame_knots"]);
  snapshot.footprint.front_extent_m=n["footprint"]["front_extent_m"].as<decltype(snapshot.footprint.front_extent_m)>();
  snapshot.footprint.rear_extent_m=n["footprint"]["rear_extent_m"].as<decltype(snapshot.footprint.rear_extent_m)>();
  snapshot.footprint.left_extent_m=n["footprint"]["left_extent_m"].as<decltype(snapshot.footprint.left_extent_m)>();
  snapshot.footprint.right_extent_m=n["footprint"]["right_extent_m"].as<decltype(snapshot.footprint.right_extent_m)>();
  snapshot.footprint.margin_m=n["footprint"]["margin_m"].as<decltype(snapshot.footprint.margin_m)>();
  snapshot.trajectory.progress_origin_m=n["trajectory"]["progress_origin_m"].as<decltype(snapshot.trajectory.progress_origin_m)>();
  snapshot.trajectory.elapsed_time_sec=n["trajectory"]["elapsed_time_sec"].as<decltype(snapshot.trajectory.elapsed_time_sec)>();
  snapshot.trajectory.path_distance_m=n["trajectory"]["path_distance_m"].as<decltype(snapshot.trajectory.path_distance_m)>();
  snapshot.trajectory.lateral_m=n["trajectory"]["lateral_m"].as<decltype(snapshot.trajectory.lateral_m)>();
  snapshot.trajectory.lag_m=n["trajectory"]["lag_m"].as<decltype(snapshot.trajectory.lag_m)>();
  snapshot.trajectory.heading_offset_rad=n["trajectory"]["heading_offset_rad"].as<decltype(snapshot.trajectory.heading_offset_rad)>();
  snapshot.trajectory.velocity_mps=n["trajectory"]["velocity_mps"].as<decltype(snapshot.trajectory.velocity_mps)>();
  snapshot.trajectory.progress_m=n["trajectory"]["progress_m"].as<decltype(snapshot.trajectory.progress_m)>();
  snapshot.trajectory.lateral_lower_m=n["trajectory"]["lateral_lower_m"].as<decltype(snapshot.trajectory.lateral_lower_m)>();
  snapshot.trajectory.lateral_upper_m=n["trajectory"]["lateral_upper_m"].as<decltype(snapshot.trajectory.lateral_upper_m)>();
  snapshot.trajectory.minimum_lateral_bound_reserve_m=n["trajectory"]["minimum_lateral_bound_reserve_m"].as<decltype(snapshot.trajectory.minimum_lateral_bound_reserve_m)>();
  snapshot.trajectory.progress_regression_tolerance_m=n["trajectory"]["progress_regression_tolerance_m"].as<decltype(snapshot.trajectory.progress_regression_tolerance_m)>();
  snapshot.trajectory.velocity_lower_bound_tolerance_mps=n["trajectory"]["velocity_lower_bound_tolerance_mps"].as<decltype(snapshot.trajectory.velocity_lower_bound_tolerance_mps)>();
  snapshot.trajectory.stationary_path_suffix_allowed=n["trajectory"]["stationary_path_suffix_allowed"].as<decltype(snapshot.trajectory.stationary_path_suffix_allowed)>();
  snapshot.trajectory.stationary_velocity_tolerance_mps=n["trajectory"]["stationary_velocity_tolerance_mps"].as<decltype(snapshot.trajectory.stationary_velocity_tolerance_mps)>();
  snapshot.trajectory.lateral_bound_tolerance_m=n["trajectory"]["lateral_bound_tolerance_m"].as<decltype(snapshot.trajectory.lateral_bound_tolerance_m)>();
  snapshot.terminal_stop_course_geometry.progress_m=n["terminal_stop_course_geometry"]["progress_m"].as<decltype(snapshot.terminal_stop_course_geometry.progress_m)>();
  snapshot.terminal_stop_course_geometry.curvature_radpm=n["terminal_stop_course_geometry"]["curvature_radpm"].as<decltype(snapshot.terminal_stop_course_geometry.curvature_radpm)>();
  snapshot.terminal_stop_course_geometry.lateral_lower_m=n["terminal_stop_course_geometry"]["lateral_lower_m"].as<decltype(snapshot.terminal_stop_course_geometry.lateral_lower_m)>();
  snapshot.terminal_stop_course_geometry.lateral_upper_m=n["terminal_stop_course_geometry"]["lateral_upper_m"].as<decltype(snapshot.terminal_stop_course_geometry.lateral_upper_m)>();
  snapshot.hard_wall_clearance_m=n["hard_wall_clearance_m"].as<double>();
  snapshot.bound_tolerance_m=n["bound_tolerance_m"].as<double>();
  snapshot.swept_step_m=n["swept_step_m"].as<double>();
  const auto g=n["wall_grid"];
  require(g["available"].as<bool>(),"missing physical grid");
  auto grid=std::make_shared<m::recovery_footprint::OccupancyGrid>();
  grid->width=g["width"].as<decltype(grid->width)>();
  grid->height=g["height"].as<decltype(grid->height)>();
  grid->resolution_m=g["resolution_m"].as<decltype(grid->resolution_m)>();
  grid->origin_x_m=g["origin_x_m"].as<decltype(grid->origin_x_m)>();
  grid->origin_y_m=g["origin_y_m"].as<decltype(grid->origin_y_m)>();
  const auto axis=g["y_axis"].as<std::string>();
  require(axis=="row-zero-at-minimum-y" || axis=="row-zero-at-maximum-y","unknown grid axis");
  grid->y_axis=axis=="row-zero-at-minimum-y" ? m::recovery_footprint::YAxisConvention::RowZeroAtMinimumY : m::recovery_footprint::YAxisConvention::RowZeroAtMaximumY;
  const auto payload=g["payload"].as<std::string>();
  require(!payload.empty() && std::filesystem::path(payload).filename()==payload,"nonlocal grid payload");
  std::ifstream input(directory/payload,std::ios::binary);std::int8_t byte;
  while(input.read(reinterpret_cast<char *>(&byte),sizeof(byte))) {
    require(byte>=-1 && byte<=1,"invalid grid cell");
    grid->cells.push_back(static_cast<m::recovery_footprint::CellState>(byte));
  }
  require(input.eof() && grid->cells.size()==g["cell_count"].as<std::size_t>() && grid->valid(),"invalid physical grid payload");
  snapshot.wall_grid=grid;
  snapshot.wall_grid_fingerprint=g["fingerprint"].as<std::uint64_t>();
  require(snapshot.wall_grid_fingerprint==m::recovery_footprint::occupancy_grid_fingerprint(*grid),"physical grid fingerprint mismatch");
  require(snapshot.identity.pose_snapshot_id==physical::fingerprint_control_pose_path(snapshot.control_prefix,snapshot.current_pose) &&
    snapshot.identity.course_frame_window_id==physical::fingerprint_course_frame_window(snapshot.course_frame_knots),"physical source fingerprint mismatch");
  const auto p=evidence["physical_proof"];
  physical::Result proof;
  proof.identity=identity(p["identity"]);
  require(p["outcome"].as<std::string>()==physical::to_string(physical::Outcome::Accepted),"recorded proof rejected");
  proof.outcome=physical::Outcome::Accepted;
  proof.completed_sec=p["completed_sec"].as<double>();
  const auto d=p["diagnostic"];
  proof.diagnostic.reason=m::mpcc_execution_contract::PhysicalWallCertificateReason::Accepted;
  require(d["reason"].as<std::string>()==m::mpcc_execution_contract::physical_wall_certificate_reason_name(proof.diagnostic.reason),"recorded diagnostic rejected");
  proof.diagnostic.stage_index=d["stage_index"].as<decltype(proof.diagnostic.stage_index)>();
  proof.diagnostic.waypoint_id=d["waypoint_id"].as<decltype(proof.diagnostic.waypoint_id)>();
  proof.diagnostic.path_distance_m=d["path_distance_m"].as<decltype(proof.diagnostic.path_distance_m)>();
  proof.diagnostic.lateral_m=d["lateral_m"].as<decltype(proof.diagnostic.lateral_m)>();
  proof.diagnostic.lag_m=d["lag_m"].as<decltype(proof.diagnostic.lag_m)>();
  proof.diagnostic.lower_bound_m=d["lower_bound_m"].as<decltype(proof.diagnostic.lower_bound_m)>();
  proof.diagnostic.upper_bound_m=d["upper_bound_m"].as<decltype(proof.diagnostic.upper_bound_m)>();
  proof.diagnostic.bound_reserve_m=d["bound_reserve_m"].as<decltype(proof.diagnostic.bound_reserve_m)>();
  proof.diagnostic.heading_offset_rad=d["heading_offset_rad"].as<decltype(proof.diagnostic.heading_offset_rad)>();
  proof.diagnostic.reference_progress_m=d["reference_progress_m"].as<decltype(proof.diagnostic.reference_progress_m)>();
  proof.diagnostic.solved_progress_m=d["solved_progress_m"].as<decltype(proof.diagnostic.solved_progress_m)>();
  proof.diagnostic.progress_delta_m=d["progress_delta_m"].as<decltype(proof.diagnostic.progress_delta_m)>();
  proof.diagnostic.pose_x_m=d["pose_x_m"].as<decltype(proof.diagnostic.pose_x_m)>();
  proof.diagnostic.pose_y_m=d["pose_y_m"].as<decltype(proof.diagnostic.pose_y_m)>();
  proof.diagnostic.pose_yaw_rad=d["pose_yaw_rad"].as<decltype(proof.diagnostic.pose_yaw_rad)>();
  proof.diagnostic.out_of_map=d["out_of_map"].as<decltype(proof.diagnostic.out_of_map)>();
  proof.diagnostic.contact_cell_count=d["contact_cell_count"].as<decltype(proof.diagnostic.contact_cell_count)>();
  proof.diagnostic.swept_rejected_path_index=d["swept_rejected_path_index"].as<decltype(proof.diagnostic.swept_rejected_path_index)>();
  proof.diagnostic.swept_checked_pose_count=d["swept_checked_pose_count"].as<decltype(proof.diagnostic.swept_checked_pose_count)>();
  proof.diagnostic.swept_rejected_substep=d["swept_rejected_substep"].as<decltype(proof.diagnostic.swept_rejected_substep)>();
  proof.diagnostic.swept_rejected_subdivision_count=d["swept_rejected_subdivision_count"].as<decltype(proof.diagnostic.swept_rejected_subdivision_count)>();
  proof.diagnostic.swept_rejected_segment_ratio=d["swept_rejected_segment_ratio"].as<decltype(proof.diagnostic.swept_rejected_segment_ratio)>();
  auto result=certified::build(std::make_shared<const m::mpcc_rate_resolved_execution_artifact::ExecutionArtifact>(value),snapshot,proof);
  require(result.plan!=nullptr,"recorded certified plan invalid");
  return result.plan;
}
} // namespace mpcc_observation
