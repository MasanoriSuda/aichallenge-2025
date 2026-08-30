#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_lattice_shadow.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_lattice_shadow
{
namespace
{

namespace physical_adapter = mpcc_rate_resolved_physical_adapter;

using SteadyClock = std::chrono::steady_clock;

physical::Snapshot build_wall_snapshot(
  const shadow::Snapshot & candidate,
  const shadow::ReplayWorld & replay,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory)
{
  physical::Snapshot result;
  result.identity.artifact = candidate.identity;
  result.identity.captured_sec = candidate.identity.snapshot_sec;
  result.identity.pose_snapshot_id = physical::fingerprint_control_pose_path(
    replay.control_prefix, replay.control_prefix.back());
  result.identity.course_frame_window_id =
    physical::fingerprint_course_frame_window(
    candidate.wall_course_frame_knots);
  result.wall_grid = candidate.wall_grid;
  result.wall_grid_fingerprint = replay.wall_grid_fingerprint;
  result.footprint = replay.physical_footprint;
  result.current_pose = replay.current_pose;
  result.control_prefix = replay.control_prefix;
  result.trajectory = trajectory;
  result.course_frame_knots = candidate.wall_course_frame_knots;
  result.terminal_stop_course_geometry.progress_m =
    candidate.wall_reference_progress_m;
  result.terminal_stop_course_geometry.lateral_lower_m =
    candidate.wall_lower_m;
  result.terminal_stop_course_geometry.lateral_upper_m =
    candidate.wall_upper_m;
  result.terminal_stop_course_geometry.curvature_radpm.reserve(
    candidate.request.inputs.size());
  for (const auto & input : candidate.request.inputs) {
    result.terminal_stop_course_geometry.curvature_radpm.push_back(
      input.path_curvature_radpm);
  }
  result.hard_wall_clearance_m = replay.hard_wall_clearance_m;
  result.bound_tolerance_m = replay.bound_tolerance_m;
  result.swept_step_m = replay.swept_step_m;
  return result;
}

bool source_identity_matches(
  const shadow::Snapshot & source,
  const artifact::ExecutionArtifact & execution) noexcept
{
  return
    artifact::validate(execution) == artifact::RejectReason::None &&
    artifact::same_identity(source.identity, execution.identity);
}

}  // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Accepted: return "accepted";
    case Reason::InvalidSource: return "invalid-source";
    case Reason::CandidateBuildRejected: return "candidate-build-rejected";
    case Reason::ScheduleRejected: return "schedule-rejected";
    case Reason::SolverRejected: return "solver-rejected";
    case Reason::ExactTrajectoryRejected:
      return "exact-trajectory-rejected";
    case Reason::StopNotReached: return "stop-not-reached";
    case Reason::WallProofRejected: return "wall-proof-rejected";
    case Reason::DynamicProofRejected: return "dynamic-proof-rejected";
    case Reason::CertifiedPlanRejected: return "certified-plan-rejected";
    case Reason::Exception: return "exception";
    case Reason::Count: break;
  }
  return "unknown";
}

Result evaluate(
  const shadow::Snapshot & selected_source,
  const artifact::ExecutionArtifact & selected_normal_execution,
  shadow::SolverContext & private_solver_context) noexcept
{
  const auto started = SteadyClock::now();
  Result result;
  result.source_normal_identity = selected_normal_execution.identity;
  const auto finish = [&]() {
      result.total_compute_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - started).count();
      return result;
    };
  try {
    if (!source_identity_matches(selected_source, selected_normal_execution)) {
      result.detail = "selected snapshot/artifact identity mismatch";
      return finish();
    }
    const auto stop = lattice::build_maximum_braking_candidate(
      selected_source, selected_normal_execution,
      private_solver_context.physical_constraint_tolerance());
    if (!stop.accepted()) {
      result.reason = Reason::CandidateBuildRejected;
      result.detail = std::string{lattice::to_string(stop.reason)} + '/' +
      stop.detail;
      return finish();
    }
    if (!stop.candidate.replay_world.has_value()) {
      result.reason = Reason::InvalidSource;
      result.detail = "rebased Stop replay world unavailable";
      return finish();
    }

    const auto population = lattice::build_population(
      stop.candidate,
      private_solver_context.physical_constraint_tolerance());
    if (population.empty()) {
      result.reason = Reason::ScheduleRejected;
      result.detail = "empty Stop control lattice";
      return finish();
    }
    for (const auto & candidate : population) {
      ++result.attempted_candidate_count;
      result.initial_rate_sign = candidate.schedule.initial_rate_sign;
      result.first_switch_stage = candidate.schedule.first_switch_stage;
      result.second_switch_stage = candidate.schedule.second_switch_stage;
      if (!candidate.accepted()) {
        result.reason = Reason::ScheduleRejected;
        result.detail = candidate.detail;
        continue;
      }
      const auto solved =
        private_solver_context.evaluate_fixed_steering_rate_shadow(
        stop.candidate, candidate.schedule.steering_rate_radps);
      result.solver_outcome = solved.outcome;
      result.selected_solver_ms = solved.compute_ms;
      if (
        solved.outcome != shadow::Outcome::Solved ||
        solved.execution_artifact == nullptr)
      {
        result.reason = Reason::SolverRejected;
        result.detail = solved.detail;
        continue;
      }
      const auto adapted = physical_adapter::build(
        *solved.execution_artifact,
        stop.candidate.identity.source_context.intent,
        stop.candidate.identity.source_context.stage_geometry_id);
      if (!adapted.exact_trajectory.has_value()) {
        result.reason = Reason::ExactTrajectoryRejected;
        std::ostringstream detail;
        detail << physical_adapter::to_string(adapted.reason)
               << "/stage=" << adapted.rejected_stage;
        result.detail = detail.str();
        continue;
      }
      const auto & exact = adapted.exact_trajectory.value();
      result.minimum_lateral_bound_reserve_m =
        exact.minimum_lateral_bound_reserve_m;
      if (
        exact.velocity_mps.empty() ||
        exact.velocity_mps.back() > std::max(
          1e-9, solved.execution_artifact->physical_global_tolerance))
      {
        result.reason = Reason::StopNotReached;
        result.detail = "seven-state lattice did not reach rest";
        continue;
      }
      auto wall_snapshot = build_wall_snapshot(
        stop.candidate, stop.candidate.replay_world.value(), exact);
      const auto wall_result = physical::evaluate(wall_snapshot);
      result.wall_outcome = wall_result.outcome;
      if (wall_result.outcome != physical::Outcome::Accepted) {
        result.reason = Reason::WallProofRejected;
        result.detail = wall_result.detail;
        continue;
      }
      const auto dynamic_result = dynamic::evaluate_current_world(
        stop.candidate, wall_snapshot);
      result.dynamic_valid = dynamic_result.valid;
      result.dynamic_clear = dynamic_result.clear;
      result.minimum_dynamic_clearance_m =
        dynamic_result.minimum_clearance_m;
      if (!dynamic_result.valid || !dynamic_result.clear) {
        result.reason = Reason::DynamicProofRejected;
        std::ostringstream detail;
        detail << "obstacle=" << dynamic_result.blocking_obstacle_id
               << "/minimum=" << dynamic_result.minimum_clearance_m;
        result.detail = detail.str();
        continue;
      }
      const auto built = certified::build(
        solved.execution_artifact, wall_snapshot, wall_result);
      if (built.reason != certified::RejectReason::None || built.plan == nullptr) {
        result.reason = Reason::CertifiedPlanRejected;
        result.detail = certified::to_string(built.reason);
        continue;
      }
      result.reason = Reason::Accepted;
      result.certified_stop_plan = built.plan;
      result.detail = "accepted/observation-only";
      return finish();
    }
    return finish();
  } catch (const std::exception & error) {
    result.reason = Reason::Exception;
    result.detail = error.what();
    return finish();
  } catch (...) {
    result.reason = Reason::Exception;
    result.detail = "unknown Stop lattice shadow exception";
    return finish();
  }
}

const char * to_string(const PublishReason reason) noexcept
{
  switch (reason) {
    case PublishReason::Accepted: return "accepted";
    case PublishReason::InvalidResult: return "invalid-result";
    case PublishReason::SequenceRollback: return "sequence-rollback";
  }
  return "unknown";
}

PublishReason Mailbox::publish(Result result)
{
  if (
    result.source_normal_identity.sequence == 0U ||
    !std::isfinite(result.total_compute_ms) || result.total_compute_ms < 0.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++state_.invalid_result_count;
    return PublishReason::InvalidResult;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (result.source_normal_identity.sequence <= state_.latest_sequence) {
    ++state_.sequence_rollback_count;
    return PublishReason::SequenceRollback;
  }
  state_.latest_sequence = result.source_normal_identity.sequence;
  latest_ = std::move(result);
  ++state_.accepted_count;
  return PublishReason::Accepted;
}

std::optional<Result> Mailbox::latest_after(const std::uint64_t sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !latest_.has_value() ||
    latest_->source_normal_identity.sequence <= sequence)
  {
    return std::nullopt;
  }
  return latest_;
}

MailboxState Mailbox::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_lattice_shadow
