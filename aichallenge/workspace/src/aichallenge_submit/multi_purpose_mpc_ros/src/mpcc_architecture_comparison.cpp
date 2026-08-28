#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
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
    std::ostringstream detail;
    detail << "obstacle=" << dynamic_result.blocking_obstacle_id
           << "/checked=" << dynamic_result.checked_pose_count
           << "/minimum_clearance=" << dynamic_result.minimum_clearance_m;
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
