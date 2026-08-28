#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
{
namespace
{

namespace architecture = mpcc_architecture_snapshot;
namespace contract = mpcc_execution_contract;
namespace dynamic = mpcc_rate_resolved_dynamic_proof;
namespace maneuver = mpcc_stateless_maneuver;
namespace physical = mpcc_rate_resolved_physical_adapter;
namespace shadow = mpcc_rate_resolved_shadow;
namespace wall = mpcc_rate_resolved_physical_wall;
namespace recovery = recovery_footprint;

struct DynamicModelDiagnostic
{
  bool complete{false};
  double minimum_full_disjunction_reserve_m{
    std::numeric_limits<double>::infinity()};
  int minimum_full_disjunction_reserve_stage{-1};
  double minimum_affine_node_clearance_m{std::numeric_limits<double>::infinity()};
  int minimum_affine_node_clearance_stage{-1};
  double minimum_nonlinear_node_clearance_m{std::numeric_limits<double>::infinity()};
  int minimum_nonlinear_node_clearance_stage{-1};
  double maximum_node_position_error_m{};
  int maximum_node_position_error_stage{-1};
  double preceding_node_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  int preceding_node_clearance_stage{-1};
  double following_node_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  int following_node_clearance_stage{-1};
};

dynamic::WorldObservation dynamic_world(
  const shadow::ReplayWorld & replay)
{
  dynamic::WorldObservation observation;
  observation.generation = replay.observation_generation;
  observation.observed_sec = replay.observed_sec;
  observation.current = replay.current;
  observation.obstacles.reserve(replay.obstacles.size());
  for (const auto & source : replay.obstacles) {
    observation.obstacles.push_back(dynamic::DynamicObstacle{
      source.id,
      recovery::CircleObstacle{
        source.x_m, source.y_m, source.velocity_x_mps,
        source.velocity_y_mps, source.radius_m}});
  }
  return observation;
}

std::optional<recovery::Pose2D> reconstruct_pose(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory,
  const std::size_t index,
  const double tolerance_m) noexcept
{
  if (
    index >= trajectory.progress_m.size() ||
    index >= trajectory.lateral_m.size() ||
    index >= trajectory.lag_m.size() ||
    index >= trajectory.heading_offset_rad.size())
  {
    return std::nullopt;
  }
  const auto frame = mpc_stage_geometry::sample_course_frame(
    knots, trajectory.progress_m[index], std::max(1e-9, tolerance_m));
  if (!frame.has_value()) {
    return std::nullopt;
  }
  const auto world = contract::reconstruct_planar_pose_from_frenet(
    contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
    contract::FrenetPose{
      trajectory.lateral_m[index], trajectory.lag_m[index],
      trajectory.heading_offset_rad[index]});
  if (!world.has_value()) {
    return std::nullopt;
  }
  return recovery::Pose2D{world->x_m, world->y_m, world->yaw_rad};
}

std::optional<recovery::Pose2D> reconstruct_affine_pose(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const std::size_t state_index,
  const double tolerance_m) noexcept
{
  if (state_index >= artifact.predicted_states.size()) {
    return std::nullopt;
  }
  const auto & state = artifact.predicted_states[state_index];
  const auto frame = mpc_stage_geometry::sample_course_frame(
    knots, artifact.course_progress_origin_m + state.progress_m,
    std::max(1e-9, tolerance_m));
  if (!frame.has_value()) {
    return std::nullopt;
  }
  const auto world = contract::reconstruct_planar_pose_from_frenet(
    contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
    contract::FrenetPose{
      state.lateral_m, state.lag_m, state.heading_offset_rad});
  if (!world.has_value()) {
    return std::nullopt;
  }
  return recovery::Pose2D{world->x_m, world->y_m, world->yaw_rad};
}

DynamicModelDiagnostic diagnose_dynamic_model(
  const shadow::Snapshot & candidate,
  const shadow::ReplayWorld & replay,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory,
  const std::string & obstacle_id, const int pass_side_sign,
  const double rejected_elapsed_sec) noexcept
{
  DynamicModelDiagnostic result;
  const auto obstacle = std::find_if(
    replay.obstacles.begin(), replay.obstacles.end(),
    [&obstacle_id](const shadow::ReplayDynamicObstacle & item) {
      return item.id == obstacle_id;
    });
  if (
    obstacle == replay.obstacles.end() ||
    artifact.predicted_states.size() != artifact.control_stages.size() + 1U)
  {
    return result;
  }
  const recovery::CircleObstacle circle{
    obstacle->x_m, obstacle->y_m, obstacle->velocity_x_mps,
    obstacle->velocity_y_mps, obstacle->radius_m};
  const double control_origin_age_sec =
    candidate.control_prediction_origin_sec - replay.observed_sec;
  double stage_end_sec = 0.0;
  for (std::size_t stage = 0U; stage < artifact.control_stages.size(); ++stage) {
    stage_end_sec += artifact.control_stages[stage].duration_sec;
    const auto affine_pose = reconstruct_affine_pose(
      candidate.wall_course_frame_knots, artifact, stage + 1U,
      replay.bound_tolerance_m);
    const auto exact_time = std::lower_bound(
      trajectory.elapsed_time_sec.begin(), trajectory.elapsed_time_sec.end(),
      stage_end_sec - 1e-9);
    if (
      !affine_pose.has_value() || exact_time == trajectory.elapsed_time_sec.end())
    {
      return result;
    }
    const auto exact_index = static_cast<std::size_t>(
      std::distance(trajectory.elapsed_time_sec.begin(), exact_time));
    if (std::abs(trajectory.elapsed_time_sec[exact_index] - stage_end_sec) > 1e-7) {
      return result;
    }
    const auto nonlinear_pose = reconstruct_pose(
      candidate.wall_course_frame_knots, trajectory, exact_index,
      replay.bound_tolerance_m);
    if (!nonlinear_pose.has_value()) {
      return result;
    }
    const double world_elapsed_sec = control_origin_age_sec + stage_end_sec;
    const auto affine_clearance = recovery::circle_obstacle_clearance_at_time(
      replay.physical_footprint, affine_pose.value(), circle, world_elapsed_sec);
    const auto nonlinear_clearance = recovery::circle_obstacle_clearance_at_time(
      replay.physical_footprint, nonlinear_pose.value(), circle, world_elapsed_sec);
    if (!affine_clearance.has_value() || !nonlinear_clearance.has_value()) {
      return result;
    }
    if (affine_clearance.value() < result.minimum_affine_node_clearance_m) {
      result.minimum_affine_node_clearance_m = affine_clearance.value();
      result.minimum_affine_node_clearance_stage = static_cast<int>(stage);
    }
    if (nonlinear_clearance.value() < result.minimum_nonlinear_node_clearance_m) {
      result.minimum_nonlinear_node_clearance_m = nonlinear_clearance.value();
      result.minimum_nonlinear_node_clearance_stage = static_cast<int>(stage);
    }
    if (world_elapsed_sec <= rejected_elapsed_sec + 1e-9) {
      result.preceding_node_clearance_m = nonlinear_clearance.value();
      result.preceding_node_clearance_stage = static_cast<int>(stage);
    } else if (result.following_node_clearance_stage < 0) {
      result.following_node_clearance_m = nonlinear_clearance.value();
      result.following_node_clearance_stage = static_cast<int>(stage);
    }
    if (
      (pass_side_sign == -1 || pass_side_sign == 1) &&
      stage < candidate.dynamic_obstacle_stages.size() &&
      candidate.dynamic_obstacle_stages[stage].valid)
    {
      const auto & target = candidate.dynamic_obstacle_stages[stage];
      const auto & state = artifact.predicted_states[stage + 1U];
      const double behind_reserve_m =
        target.target_progress_m - target.longitudinal_overlap_m -
        (state.progress_m + state.lag_m);
      const double side_reserve_m =
        static_cast<double>(pass_side_sign) *
        (state.lateral_m - target.target_lateral_m) -
        target.lateral_center_separation_m;
      const double disjunction_reserve_m = std::max(
        behind_reserve_m, side_reserve_m);
      if (
        disjunction_reserve_m < result.minimum_full_disjunction_reserve_m)
      {
        result.minimum_full_disjunction_reserve_m = disjunction_reserve_m;
        result.minimum_full_disjunction_reserve_stage =
          static_cast<int>(stage);
      }
    }
    const double position_error_m = std::hypot(
      affine_pose->x_m - nonlinear_pose->x_m,
      affine_pose->y_m - nonlinear_pose->y_m);
    if (position_error_m > result.maximum_node_position_error_m) {
      result.maximum_node_position_error_m = position_error_m;
      result.maximum_node_position_error_stage = static_cast<int>(stage);
    }
  }
  result.complete = true;
  return result;
}

wall::Snapshot wall_snapshot(
  const shadow::Snapshot & candidate,
  const shadow::ReplayWorld & replay,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory)
{
  wall::Snapshot result;
  result.identity.artifact = candidate.identity;
  result.identity.captured_sec = candidate.identity.snapshot_sec;
  result.identity.pose_snapshot_id = wall::fingerprint_control_pose_path(
    replay.control_prefix, replay.control_prefix.back());
  result.identity.course_frame_window_id =
    wall::fingerprint_course_frame_window(candidate.wall_course_frame_knots);
  result.wall_grid = candidate.wall_grid;
  result.wall_grid_fingerprint = replay.wall_grid_fingerprint;
  result.footprint = replay.physical_footprint;
  result.current_pose = replay.current_pose;
  result.control_prefix = replay.control_prefix;
  result.trajectory = trajectory;
  result.course_frame_knots = candidate.wall_course_frame_knots;
  result.hard_wall_clearance_m = replay.hard_wall_clearance_m;
  result.bound_tolerance_m = replay.bound_tolerance_m;
  result.swept_step_m = replay.swept_step_m;
  return result;
}

dynamic::Result prove_dynamic(
  const shadow::Snapshot & candidate,
  const shadow::ReplayWorld & replay,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory)
{
  dynamic::Result result;
  const auto observation = dynamic_world(replay);
  if (!dynamic::observation_valid(observation) || !observation.current) {
    result.valid = false;
    result.clear = false;
    return result;
  }
  dynamic::observe_timed_path(
    replay.physical_footprint, replay.control_prefix,
    replay.control_prefix_elapsed_sec, replay.swept_step_m,
    observation, result);
  if (!result.valid || !result.clear) {
    return result;
  }
  auto previous_pose = replay.control_prefix.back();
  double previous_time_sec = replay.control_prefix_elapsed_sec.back();
  for (std::size_t index = 0U; index < trajectory.elapsed_time_sec.size(); ++index) {
    const auto pose = reconstruct_pose(
      candidate.wall_course_frame_knots, trajectory, index,
      replay.bound_tolerance_m);
    if (!pose.has_value()) {
      result.valid = false;
      result.clear = false;
      return result;
    }
    const double time_sec =
      candidate.control_prediction_origin_sec - replay.observed_sec +
      trajectory.elapsed_time_sec[index];
    dynamic::observe_segment(
      replay.physical_footprint, previous_pose, pose.value(),
      previous_time_sec, time_sec, replay.swept_step_m,
      observation, result);
    if (!result.valid || !result.clear) {
      return result;
    }
    previous_pose = pose.value();
    previous_time_sec = time_sec;
  }
  dynamic::finalize(observation, result);
  return result;
}

ArmResult evaluate_arm(
  const Arm arm,
  shadow::Snapshot candidate,
  const std::uint64_t source_fingerprint,
  const std::uint64_t candidate_fingerprint,
  const maneuver::TerminalResolution & successor)
{
  ArmResult arm_result;
  arm_result.arm = arm;
  arm_result.source_interaction_fingerprint = source_fingerprint;
  arm_result.candidate_fingerprint = candidate_fingerprint;
  if (!successor.accepted) {
    arm_result.stage = Stage::TerminalSuccessorRejected;
    arm_result.detail = successor.detail;
    return arm_result;
  }
  if (!candidate.replay_world.has_value()) {
    arm_result.stage = Stage::SourceRejected;
    arm_result.detail = "replay world unavailable";
    return arm_result;
  }

  // A local context is intentional: architecture arms may not share a
  // receding warm start or any solver lifecycle state.
  shadow::SolverContext solver;
  const auto solved = solver.evaluate(candidate);
  arm_result.solver_outcome = solved.outcome;
  arm_result.solver_compute_ms = solved.compute_ms;
  arm_result.terminal_progress_m = solved.terminal_progress_m;
  arm_result.terminal_velocity_mps = solved.terminal_velocity_mps;
  if (
    solved.outcome != shadow::Outcome::Solved ||
    solved.execution_artifact == nullptr)
  {
    arm_result.stage = Stage::SolverRejected;
    arm_result.detail = solved.detail;
    return arm_result;
  }
  const auto adapted = physical::build(
    *solved.execution_artifact,
    candidate.identity.source_context.intent,
    candidate.identity.source_context.stage_geometry_id);
  if (!adapted.exact_trajectory.has_value()) {
    arm_result.stage = Stage::ExactTrajectoryRejected;
    std::ostringstream detail;
    detail << "adapter=" << physical::to_string(adapted.reason)
           << "/stage=" << adapted.rejected_stage;
    arm_result.detail = detail.str();
    return arm_result;
  }
  auto exact = adapted.exact_trajectory.value();
  arm_result.minimum_lateral_bound_reserve_m =
    exact.minimum_lateral_bound_reserve_m;
  auto physical_snapshot = wall_snapshot(
    candidate, candidate.replay_world.value(), exact);
  auto wall_result = wall::evaluate(physical_snapshot);
  if (wall_result.outcome != wall::Outcome::Accepted) {
    arm_result.stage = Stage::WallProofRejected;
    arm_result.detail = wall_result.detail;
    return arm_result;
  }
  auto dynamic_result = prove_dynamic(
    candidate, candidate.replay_world.value(), exact);
  if (!dynamic_result.valid || !dynamic_result.clear) {
    arm_result.stage = Stage::DynamicProofRejected;
    const auto & replay = candidate.replay_world.value();
    const double control_origin_age_sec =
      candidate.control_prediction_origin_sec - replay.observed_sec;
    const bool prefix_scope =
      std::isfinite(dynamic_result.rejected_elapsed_sec) &&
      dynamic_result.rejected_elapsed_sec <= control_origin_age_sec + 1e-9;
    int qp_stage = -1;
    if (
      std::isfinite(dynamic_result.rejected_elapsed_sec) && !prefix_scope)
    {
      const double solver_elapsed_sec =
        dynamic_result.rejected_elapsed_sec - control_origin_age_sec;
      double stage_end_sec = 0.0;
      for (std::size_t stage = 0U; stage < candidate.request.inputs.size(); ++stage) {
        stage_end_sec += candidate.request.inputs[stage].stage_dt_sec;
        if (solver_elapsed_sec <= stage_end_sec + 1e-9) {
          qp_stage = static_cast<int>(stage);
          break;
        }
      }
    }
    const auto model_diagnostic = diagnose_dynamic_model(
      candidate, replay, *solved.execution_artifact, exact,
      dynamic_result.blocking_obstacle_id,
      solved.dynamic_obstacle_resolved_side_sign,
      dynamic_result.rejected_elapsed_sec);
    std::ostringstream detail;
    detail << "obstacle=" << dynamic_result.blocking_obstacle_id
           << "/checked=" << dynamic_result.checked_pose_count
           << "/dynamic_side=" << solved.dynamic_obstacle_resolved_side_sign
           << "/first_side_stage="
           << solved.dynamic_obstacle_first_pass_side_stage
           << "/behind_rows="
           << solved.dynamic_obstacle_stay_behind_row_count
           << "/side_rows=" << solved.dynamic_obstacle_pass_side_row_count
           << "/partial_rows="
           << solved.dynamic_obstacle_partial_escape_row_count
           << "/reason=" << recovery::to_string(dynamic_result.rejection_reason)
           << "/scope=" << (prefix_scope ? "control-prefix" : "candidate")
           << "/qp_stage=" << qp_stage
           << "/rejected_t=" << dynamic_result.rejected_elapsed_sec
           << "/rejected_clearance=" << dynamic_result.rejected_clearance_m
           << "/rejected_pose=[" << dynamic_result.rejected_pose.x_m << ','
           << dynamic_result.rejected_pose.y_m << ','
           << dynamic_result.rejected_pose.yaw_rad << ']'
           << "/minimum_obstacle="
           << dynamic_result.minimum_clearance_obstacle_id
           << "/minimum_t=" << dynamic_result.minimum_clearance_elapsed_sec
           << "/minimum_clearance=" << dynamic_result.minimum_clearance_m;
    if (model_diagnostic.complete) {
      detail << "/affine_node_min="
             << model_diagnostic.minimum_affine_node_clearance_m
             << "@" << model_diagnostic.minimum_affine_node_clearance_stage
             << "/nonlinear_node_min="
             << model_diagnostic.minimum_nonlinear_node_clearance_m
             << "@" << model_diagnostic.minimum_nonlinear_node_clearance_stage
             << "/node_pose_error_max="
             << model_diagnostic.maximum_node_position_error_m
             << "@" << model_diagnostic.maximum_node_position_error_stage
             << "/full_disjunction_min="
             << model_diagnostic.minimum_full_disjunction_reserve_m
             << "@" << model_diagnostic.minimum_full_disjunction_reserve_stage
             << "/rejection_node_bracket="
             << model_diagnostic.preceding_node_clearance_m
             << "@" << model_diagnostic.preceding_node_clearance_stage
             << "->" << model_diagnostic.following_node_clearance_m
             << "@" << model_diagnostic.following_node_clearance_stage;
    } else {
      detail << "/node_model_diagnostic=unavailable";
    }
    arm_result.detail = detail.str();
    return arm_result;
  }

  ManeuverBundle bundle;
  bundle.target_id = candidate.identity.source_context.target_id;
  bundle.pass_side_sign = candidate.identity.source_context.execution_side_sign;
  bundle.source_interaction_fingerprint = source_fingerprint;
  bundle.candidate_fingerprint = candidate_fingerprint;
  bundle.exact_trajectory = std::move(exact);
  bundle.wall_certificate = std::move(wall_result);
  bundle.dynamic_certificate = std::move(dynamic_result);
  bundle.terminal_successor = successor.successor;
  bundle.stop_suffix = successor.stop_suffix;
  arm_result.stage = Stage::Accepted;
  arm_result.detail = "accepted";
  arm_result.bundle = std::move(bundle);
  return arm_result;
}

ArmResult rejected_arm(
  const Arm arm, const Stage stage, const std::uint64_t source_fingerprint,
  std::string detail)
{
  ArmResult result;
  result.arm = arm;
  result.stage = stage;
  result.source_interaction_fingerprint = source_fingerprint;
  result.detail = std::move(detail);
  return result;
}

}  // namespace

const char * to_string(const Arm arm) noexcept
{
  switch (arm) {
    case Arm::PersistentA: return "persistent-a";
    case Arm::StatelessLeftB: return "stateless-left-b";
    case Arm::StatelessRightB: return "stateless-right-b";
  }
  return "unknown";
}

const char * to_string(const Stage stage) noexcept
{
  switch (stage) {
    case Stage::Accepted: return "accepted";
    case Stage::SourceRejected: return "source-rejected";
    case Stage::CandidateRejected: return "candidate-rejected";
    case Stage::SolverRejected: return "solver-rejected";
    case Stage::ExactTrajectoryRejected: return "exact-trajectory-rejected";
    case Stage::WallProofRejected: return "wall-proof-rejected";
    case Stage::DynamicProofRejected: return "dynamic-proof-rejected";
    case Stage::TerminalSuccessorRejected:
      return "terminal-successor-rejected";
  }
  return "unknown";
}

Report compare(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  Report report;
  try {
    const auto & source = recorded.source;
    const auto source_fingerprint = recorded.interaction_fingerprint;
    report.source_interaction_fingerprint = source_fingerprint;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      report.detail = "source interaction snapshot rejected";
      for (const auto arm :
        {Arm::PersistentA, Arm::StatelessLeftB, Arm::StatelessRightB})
      {
        report.arms.push_back(rejected_arm(
          arm, Stage::SourceRejected, source_fingerprint, report.detail));
      }
      return report;
    }
    report.source_accepted = true;
    report.detail = "accepted";
    const auto persistent_successor = maneuver::resolve_terminal_successor(source);
    report.arms.push_back(evaluate_arm(
      Arm::PersistentA, source, source_fingerprint, source_fingerprint,
      persistent_successor));

    for (const auto & [arm, side] :
      {std::pair{Arm::StatelessLeftB, 1},
       std::pair{Arm::StatelessRightB, -1}})
    {
      const auto built = maneuver::build(source, source_fingerprint, side);
      if (!built.seed.has_value()) {
        const auto stage =
          built.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
          Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
        report.arms.push_back(rejected_arm(
          arm, stage, source_fingerprint,
          std::string{maneuver::to_string(built.reason)} + ": " + built.detail));
        continue;
      }
      const auto successor = maneuver::resolve_terminal_successor(
        built.seed->solver_snapshot);
      report.arms.push_back(evaluate_arm(
        arm, built.seed->solver_snapshot, source_fingerprint,
        built.seed->candidate_fingerprint, successor));
    }
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown architecture comparison exception";
  }
  return report;
}

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
