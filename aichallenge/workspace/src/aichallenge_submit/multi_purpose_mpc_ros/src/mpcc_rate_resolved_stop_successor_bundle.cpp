#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_successor_bundle.hpp"

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_successor_bundle
{
namespace
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace contract = mpcc_execution_contract;
namespace physical = mpcc_rate_resolved_physical_wall;
namespace recovery = recovery_footprint;

bool finite(const double value) noexcept
{
  return std::isfinite(value);
}

bool same_command(
  const mpcc_rate_resolved_physical_adapter::StopContingencyResult::
  ActuationSample & lhs,
  const mpcc_rate_resolved_physical_adapter::StopContingencyResult::
  ActuationSample & rhs,
  const double tolerance) noexcept
{
  return lhs.command_interval_index == rhs.command_interval_index &&
         std::abs(lhs.acceleration_mps2 - rhs.acceleration_mps2) <= tolerance &&
         std::abs(lhs.steering_rate_radps - rhs.steering_rate_radps) <=
         tolerance;
}

}  // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Available: return "available";
    case Reason::StopSuccessorUnavailable:
      return "stop-successor-unavailable";
    case Reason::InvalidSource: return "invalid-source";
    case Reason::UnsupportedTerminalIntent:
      return "unsupported-terminal-intent";
    case Reason::InvalidIdentity: return "invalid-identity";
    case Reason::InvalidActuationSequence:
      return "invalid-actuation-sequence";
    case Reason::InvalidExecutionArtifact:
      return "invalid-execution-artifact";
    case Reason::PhysicalWallRejected: return "physical-wall-rejected";
    case Reason::CertifiedPlanRejected: return "certified-plan-rejected";
    case Reason::Count: break;
  }
  return "unknown";
}

const char * to_string(const ActuationRejectDetail detail) noexcept
{
  switch (detail) {
    case ActuationRejectDetail::None: return "none";
    case ActuationRejectDetail::ExactTrajectoryShape:
      return "exact-trajectory-shape";
    case ActuationRejectDetail::InitialLateralBounds:
      return "initial-lateral-bounds";
    case ActuationRejectDetail::CommandIndexDiscontinuity:
      return "command-index-discontinuity";
    case ActuationRejectDetail::CommandChangedWithinInterval:
      return "command-changed-within-interval";
    case ActuationRejectDetail::InvalidDenseSample:
      return "invalid-dense-sample";
    case ActuationRejectDetail::InvalidCommandDuration:
      return "invalid-command-duration";
    case ActuationRejectDetail::PublicationIntervalNotCovered:
      return "publication-interval-not-covered";
    case ActuationRejectDetail::ProgressRegressed:
      return "progress-regressed";
    case ActuationRejectDetail::Count: break;
  }
  return "unknown";
}

Result build(
  const retained::Request & request,
  const retained::StopSuccessorResult & stop_successor,
  const std::uint64_t artifact_sequence)
{
  Result result;
  const auto reject_actuation = [&] (
      const ActuationRejectDetail detail, const std::size_t index,
      const double observed = std::numeric_limits<double>::quiet_NaN(),
      const double required = std::numeric_limits<double>::quiet_NaN(),
      const double certificate_tolerance =
      std::numeric_limits<double>::quiet_NaN()) {
      result.reason = Reason::InvalidActuationSequence;
      result.actuation_detail = detail;
      result.rejected_index = index;
      result.observed_value = observed;
      result.required_bound = required;
      result.certificate_tolerance = certificate_tolerance;
      return result;
    };
  if (!stop_successor.accepted()) {
    return result;
  }
  if (
    request.plan == nullptr || request.plan->execution_artifact == nullptr ||
    request.plan->physical_snapshot == nullptr ||
    certified::validate(*request.plan) != certified::RejectReason::None)
  {
    result.reason = Reason::InvalidSource;
    return result;
  }
  const auto & source_artifact = *request.plan->execution_artifact;
  const auto & source_physical = *request.plan->physical_snapshot;
  const auto & source_context = source_artifact.identity.source_context;
  if (source_context.intent == contract::ControlIntent::Return) {
    // A maximum-braking endpoint is not automatically a semantically valid
    // Return endpoint.  Keep this case on the existing fail-closed owner.
    result.reason = Reason::UnsupportedTerminalIntent;
    return result;
  }
  if (
    artifact_sequence == 0U || request.decision_id == 0U ||
    request.decision_id != stop_successor.decision_id ||
    request.current_intent != source_context.intent ||
    stop_successor.source_sequence != source_artifact.identity.sequence ||
    !request.obstacles.current ||
    request.obstacles.generation != stop_successor.obstacle_generation ||
    !finite(request.now_sec) || !finite(request.control_origin_sec) ||
    request.control_origin_sec < request.now_sec ||
    !finite(request.control_origin_speed_mps) ||
    request.control_origin_speed_mps <=
    std::max(1e-9, source_artifact.physical_global_tolerance) ||
    !finite(request.current_steering_rad) ||
    !finite(request.current_response_steering_rad))
  {
    result.reason = Reason::InvalidIdentity;
    return result;
  }

  const auto & exact = stop_successor.exact_trajectory;
  const auto & samples = stop_successor.actuation_samples;
  const std::size_t dense_count = exact.elapsed_time_sec.size();
  if (
    !race_mpcc_foundation::exact_physical_execution_trajectory_complete(exact) ||
    dense_count == 0U || samples.size() != dense_count)
  {
    return reject_actuation(
      ActuationRejectDetail::ExactTrajectoryShape, samples.size(),
      static_cast<double>(samples.size()), static_cast<double>(dense_count));
  }
  if (
    !finite(stop_successor.initial_lateral_m) ||
    !finite(stop_successor.initial_lag_m) ||
    !finite(stop_successor.initial_heading_offset_rad) ||
    !finite(stop_successor.lifted_control_origin_progress_m) ||
    !finite(stop_successor.initial_lateral_lower_m) ||
    !finite(stop_successor.initial_lateral_upper_m) ||
    stop_successor.initial_lateral_lower_m >
    stop_successor.initial_lateral_upper_m)
  {
    return reject_actuation(
      ActuationRejectDetail::InitialLateralBounds, 0U,
      stop_successor.initial_lateral_m,
      stop_successor.initial_lateral_lower_m);
  }

  const double tolerance =
    std::max(1e-9, source_artifact.physical_global_tolerance);
  std::vector<std::size_t> command_end_indices;
  std::vector<double> command_durations_sec;
  std::vector<double> command_acceleration_mps2;
  std::vector<double> command_steering_rate_radps;
  std::vector<double> command_curvature_radpm;
  std::size_t begin = 0U;
  std::size_t expected_command_index = 0U;
  while (begin < samples.size()) {
    if (samples[begin].command_interval_index != expected_command_index) {
      return reject_actuation(
        ActuationRejectDetail::CommandIndexDiscontinuity, begin,
        static_cast<double>(samples[begin].command_interval_index),
        static_cast<double>(expected_command_index), tolerance);
    }
    std::size_t end = begin;
    double duration_sec{};
    while (
      end < samples.size() &&
      samples[end].command_interval_index == expected_command_index)
    {
      if (!same_command(samples[begin], samples[end], tolerance)) {
        return reject_actuation(
          ActuationRejectDetail::CommandChangedWithinInterval, end,
          samples[end].acceleration_mps2 - samples[begin].acceleration_mps2,
          tolerance, tolerance);
      }
      if (
        !finite(samples[end].elapsed_time_sec) ||
        !finite(samples[end].duration_sec) || samples[end].duration_sec <= 0.0 ||
        !finite(samples[end].acceleration_mps2) ||
        !finite(samples[end].effective_acceleration_mps2) ||
        !finite(samples[end].steering_rate_radps) ||
        !finite(samples[end].end_velocity_mps) ||
        !finite(samples[end].end_steering_rad) ||
        !finite(samples[end].end_response_steering_rad) ||
        !finite(samples[end].path_curvature_radpm) ||
        !finite(samples[end].virtual_progress_speed_mps))
      {
        return reject_actuation(
          ActuationRejectDetail::InvalidDenseSample, end,
          samples[end].duration_sec, 0.0, tolerance);
      }
      duration_sec += samples[end].duration_sec;
      ++end;
    }
    if (!finite(duration_sec) || duration_sec <= 0.0) {
      return reject_actuation(
        ActuationRejectDetail::InvalidCommandDuration,
        command_end_indices.size(), duration_sec, 0.0, tolerance);
    }
    command_end_indices.push_back(end - 1U);
    command_durations_sec.push_back(duration_sec);
    command_acceleration_mps2.push_back(samples[begin].acceleration_mps2);
    command_steering_rate_radps.push_back(samples[begin].steering_rate_radps);
    command_curvature_radpm.push_back(samples[begin].path_curvature_radpm);
    begin = end;
    ++expected_command_index;
  }
  if (
    command_end_indices.empty() ||
    command_durations_sec.front() + tolerance <
    source_artifact.publication_interval_sec)
  {
    return reject_actuation(
      ActuationRejectDetail::PublicationIntervalNotCovered, 0U,
      command_durations_sec.empty() ? 0.0 : command_durations_sec.front(),
      source_artifact.publication_interval_sec, tolerance);
  }

  auto execution = std::make_shared<artifact::ExecutionArtifact>();
  auto current_context = source_context;
  current_context.decision_id = request.decision_id;
  current_context.observation_generation = request.obstacles.generation;
  if (contract::canonical_normal_intent_requires_target_observation(
      current_context.intent))
  {
    current_context.target_obstacle_generation = request.obstacles.generation;
  }
  if (current_context.dynamic_obstacle_constraint_active) {
    current_context.dynamic_obstacle_generation = request.obstacles.generation;
  }
  current_context.horizon_steps = command_end_indices.size();
  current_context.fingerprint = 0U;
  current_context = contract::seal_problem_context(std::move(current_context));
  execution->identity = artifact::Identity{
    artifact_sequence, current_context, request.now_sec};
  execution->prediction_origin_sec = request.control_origin_sec;
  execution->publication_interval_sec =
    source_artifact.publication_interval_sec;
  execution->completed_sec = request.control_origin_sec;
  execution->course_progress_origin_m =
    source_artifact.course_progress_origin_m;
  execution->semantic_initial_steering_rad = request.current_steering_rad;
  execution->semantic_initial_response_steering_rad =
    request.current_response_steering_rad;
  execution->wheelbase_m = source_artifact.wheelbase_m;
  execution->yaw_response_gain = source_artifact.yaw_response_gain;
  execution->yaw_response_time_constant_sec =
    source_artifact.yaw_response_time_constant_sec;
  execution->minimum_frenet_denominator =
    source_artifact.minimum_frenet_denominator;
  execution->maximum_abs_steering_rad =
    source_artifact.maximum_abs_steering_rad;
  execution->maximum_abs_steering_rate_radps =
    source_artifact.maximum_abs_steering_rate_radps;
  execution->physical_global_tolerance =
    source_artifact.physical_global_tolerance;
  execution->maximum_constraint_violation =
    source_artifact.maximum_constraint_violation;
  execution->maximum_normalized_constraint_violation =
    source_artifact.maximum_normalized_constraint_violation;

  const double initial_course_progress_m =
    stop_successor.lifted_control_origin_progress_m -
    stop_successor.initial_lag_m;
  execution->predicted_states.push_back(artifact::PredictedState{
    stop_successor.initial_lateral_m,
    stop_successor.initial_lag_m,
    stop_successor.initial_heading_offset_rad,
    request.control_origin_speed_mps,
    initial_course_progress_m - execution->course_progress_origin_m,
    request.current_steering_rad,
    request.current_response_steering_rad});
  execution->nominal_path_distance_m.push_back(0.0);
  execution->lateral_lower_m.push_back(
    stop_successor.initial_lateral_lower_m);
  execution->lateral_upper_m.push_back(
    stop_successor.initial_lateral_upper_m);

  double previous_progress_m = initial_course_progress_m;
  for (std::size_t command = 0U; command < command_end_indices.size(); ++command) {
    const std::size_t dense_end = command_end_indices[command];
    const double duration_sec = command_durations_sec[command];
    const double endpoint_progress_m = exact.progress_m[dense_end];
    const double progress_delta_m = endpoint_progress_m - previous_progress_m;
    const double progress_speed_mps =
      progress_delta_m / duration_sec;
    if (!finite(progress_speed_mps) || progress_speed_mps < -tolerance) {
      return reject_actuation(
        ActuationRejectDetail::ProgressRegressed, command,
        progress_delta_m, -tolerance * duration_sec, tolerance);
    }
    artifact::ControlStage control;
    control.acceleration_mps2 = command_acceleration_mps2[command];
    control.steering_rate_radps = command_steering_rate_radps[command];
    control.virtual_progress_speed_mps = std::max(0.0, progress_speed_mps);
    control.duration_sec = duration_sec;
    control.virtual_progress_lower_mps = 0.0;
    control.virtual_progress_upper_mps =
      std::max(control.virtual_progress_speed_mps, tolerance);
    control.acceleration_lower_mps2 = request.minimum_acceleration_mps2;
    control.acceleration_upper_mps2 = request.maximum_acceleration_mps2;
    control.path_curvature_radpm = command_curvature_radpm[command];
    execution->control_stages.push_back(control);
    execution->predicted_states.push_back(artifact::PredictedState{
      exact.lateral_m[dense_end], exact.lag_m[dense_end],
      exact.heading_offset_rad[dense_end], exact.velocity_mps[dense_end],
      endpoint_progress_m - execution->course_progress_origin_m,
      samples[dense_end].end_steering_rad,
      samples[dense_end].end_response_steering_rad});
    execution->nominal_path_distance_m.push_back(
      exact.path_distance_m[dense_end]);
    execution->lateral_lower_m.push_back(exact.lateral_lower_m[dense_end]);
    execution->lateral_upper_m.push_back(exact.lateral_upper_m[dense_end]);
    previous_progress_m = endpoint_progress_m;
  }
  if (artifact::validate(*execution) != artifact::RejectReason::None) {
    result.reason = Reason::InvalidExecutionArtifact;
    return result;
  }

  physical::Snapshot snapshot;
  snapshot.identity.artifact = execution->identity;
  snapshot.identity.pose_snapshot_id = physical::fingerprint_control_pose_path(
    {request.control_pose}, request.control_pose);
  snapshot.identity.course_frame_window_id =
    physical::fingerprint_course_frame_window(source_physical.course_frame_knots);
  snapshot.identity.captured_sec = request.control_origin_sec;
  snapshot.wall_grid = request.current_wall_grid;
  if (snapshot.wall_grid != nullptr) {
    snapshot.wall_grid_fingerprint =
      recovery::occupancy_grid_fingerprint(*snapshot.wall_grid);
  }
  snapshot.footprint = request.current_footprint;
  snapshot.current_pose = request.control_pose;
  snapshot.control_prefix = {request.control_pose};
  snapshot.trajectory = exact;
  snapshot.course_frame_knots = source_physical.course_frame_knots;
  snapshot.terminal_stop_course_geometry =
    source_physical.terminal_stop_course_geometry;
  snapshot.hard_wall_clearance_m = source_physical.hard_wall_clearance_m;
  snapshot.bound_tolerance_m = source_physical.bound_tolerance_m;
  snapshot.swept_step_m = source_physical.swept_step_m;
  if (!physical::snapshot_valid(snapshot)) {
    result.reason = Reason::PhysicalWallRejected;
    return result;
  }
  const auto physical_result = physical::evaluate(snapshot);
  if (
    physical_result.outcome != physical::Outcome::Accepted ||
    physical_result.diagnostic.reason !=
    contract::PhysicalWallCertificateReason::Accepted)
  {
    result.reason = Reason::PhysicalWallRejected;
    return result;
  }
  const auto built = certified::build(execution, snapshot, physical_result);
  if (built.reason != certified::RejectReason::None || built.plan == nullptr) {
    result.reason = Reason::CertifiedPlanRejected;
    return result;
  }
  result.reason = Reason::Available;
  result.plan = built.plan;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_successor_bundle
