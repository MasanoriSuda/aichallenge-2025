#include "multi_purpose_mpc_ros/mpcc_on_trajectory_connector.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_on_trajectory_connector
{
namespace
{

constexpr double kIdentityTolerance = 1e-9;

double wrap_to_pi(double angle) noexcept
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2.0 * pi;
  while (angle > pi) {
    angle -= two_pi;
  }
  while (angle < -pi) {
    angle += two_pi;
  }
  return angle;
}

double interpolate(
  const double start, const double end, const double fraction) noexcept
{
  return start + fraction * (end - start);
}

double interpolate_angle(
  const double start, const double end, const double fraction) noexcept
{
  return wrap_to_pi(start + fraction * wrap_to_pi(end - start));
}

bool finite_state(const State & state) noexcept
{
  return std::isfinite(state.lateral_m) && std::isfinite(state.lag_m) &&
         std::isfinite(state.heading_offset_rad) &&
         std::isfinite(state.velocity_mps) &&
         std::isfinite(state.absolute_progress_m) &&
         std::isfinite(state.steering_rad) &&
         std::isfinite(state.response_steering_rad);
}

std::optional<State> sample(
  const certified::CertifiedPlan & plan,
  const artifact::Cursor & cursor) noexcept
{
  if (
    certified::validate(plan) != certified::RejectReason::None ||
    !cursor.available || plan.execution_artifact == nullptr ||
    plan.physical_snapshot == nullptr)
  {
    return std::nullopt;
  }
  const auto & execution = *plan.execution_artifact;
  const auto & trajectory = plan.physical_snapshot->trajectory;
  const std::size_t stage = cursor.control_stage_index;
  if (
    stage >= execution.control_stages.size() ||
    stage + 1U >= execution.predicted_states.size() ||
    trajectory.elapsed_time_sec.empty() ||
    trajectory.elapsed_time_sec.size() != trajectory.lateral_m.size() ||
    trajectory.elapsed_time_sec.size() != trajectory.lag_m.size() ||
    trajectory.elapsed_time_sec.size() !=
    trajectory.heading_offset_rad.size() ||
    trajectory.elapsed_time_sec.size() != trajectory.velocity_mps.size() ||
    trajectory.elapsed_time_sec.size() != trajectory.progress_m.size())
  {
    return std::nullopt;
  }

  const double stage_duration = execution.control_stages[stage].duration_sec;
  if (!std::isfinite(stage_duration) || stage_duration <= 0.0) {
    return std::nullopt;
  }
  const double stage_fraction = cursor.stage_elapsed_sec / stage_duration;
  if (
    !std::isfinite(stage_fraction) || stage_fraction < 0.0 ||
    stage_fraction >= 1.0)
  {
    return std::nullopt;
  }
  const auto & affine_start = execution.predicted_states[stage];
  const auto & affine_end = execution.predicted_states[stage + 1U];

  State physical_start{
    execution.predicted_states.front().lateral_m,
    execution.predicted_states.front().lag_m,
    execution.predicted_states.front().heading_offset_rad,
    execution.predicted_states.front().velocity_mps,
    execution.course_progress_origin_m +
    execution.predicted_states.front().progress_m,
    interpolate(
      affine_start.steering_rad, affine_end.steering_rad, stage_fraction),
    interpolate(
      affine_start.response_steering_rad,
      affine_end.response_steering_rad, stage_fraction)};
  State physical_end = physical_start;
  double physical_start_sec{};

  const auto upper = std::lower_bound(
    trajectory.elapsed_time_sec.begin(), trajectory.elapsed_time_sec.end(),
    cursor.elapsed_sec);
  if (upper == trajectory.elapsed_time_sec.end()) {
    return std::nullopt;
  }
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(trajectory.elapsed_time_sec.begin(), upper));
  const auto load_physical = [&trajectory](const std::size_t index) {
      return State{
        trajectory.lateral_m[index],
        trajectory.lag_m[index],
        trajectory.heading_offset_rad[index],
        trajectory.velocity_mps[index],
        trajectory.progress_m[index],
        0.0,
        0.0};
    };
  physical_end = load_physical(upper_index);
  if (upper_index > 0U) {
    physical_start = load_physical(upper_index - 1U);
    physical_start_sec = trajectory.elapsed_time_sec[upper_index - 1U];
  }
  const double physical_end_sec = trajectory.elapsed_time_sec[upper_index];
  const double physical_duration_sec = physical_end_sec - physical_start_sec;
  if (!std::isfinite(physical_duration_sec) || physical_duration_sec <= 0.0) {
    return std::nullopt;
  }
  const double physical_fraction =
    (cursor.elapsed_sec - physical_start_sec) / physical_duration_sec;
  if (
    !std::isfinite(physical_fraction) || physical_fraction < 0.0 ||
    physical_fraction > 1.0 + kIdentityTolerance)
  {
    return std::nullopt;
  }
  const double bounded_fraction = std::clamp(physical_fraction, 0.0, 1.0);
  State state{
    interpolate(
      physical_start.lateral_m, physical_end.lateral_m, bounded_fraction),
    interpolate(physical_start.lag_m, physical_end.lag_m, bounded_fraction),
    interpolate_angle(
      physical_start.heading_offset_rad,
      physical_end.heading_offset_rad, bounded_fraction),
    interpolate(
      physical_start.velocity_mps, physical_end.velocity_mps,
      bounded_fraction),
    interpolate(
      physical_start.absolute_progress_m,
      physical_end.absolute_progress_m, bounded_fraction),
    interpolate(
      affine_start.steering_rad, affine_end.steering_rad, stage_fraction),
    interpolate(
      affine_start.response_steering_rad,
      affine_end.response_steering_rad, stage_fraction)};
  if (!finite_state(state)) {
    return std::nullopt;
  }
  return state;
}

double progress_difference(
  const double candidate, const double parent, const double path_length_m,
  const bool circular) noexcept
{
  double difference = candidate - parent;
  if (!circular) {
    return difference;
  }
  const double half = 0.5 * path_length_m;
  while (difference > half) {
    difference -= path_length_m;
  }
  while (difference < -half) {
    difference += path_length_m;
  }
  return difference;
}

bool compatible_model(
  const artifact::ExecutionArtifact & parent,
  const artifact::ExecutionArtifact & candidate) noexcept
{
  const auto & lhs = parent.identity.source_context;
  const auto & rhs = candidate.identity.source_context;
  const double tolerance = std::max(
    {kIdentityTolerance, parent.physical_global_tolerance,
      candidate.physical_global_tolerance});
  return lhs.formulation == rhs.formulation &&
         lhs.state_schema_id == rhs.state_schema_id &&
         lhs.input_schema_id == rhs.input_schema_id &&
         std::abs(parent.wheelbase_m - candidate.wheelbase_m) <= tolerance &&
         std::abs(parent.yaw_response_gain - candidate.yaw_response_gain) <=
         tolerance &&
         std::abs(
           parent.yaw_response_time_constant_sec -
           candidate.yaw_response_time_constant_sec) <= tolerance;
}

}  // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Accepted: return "accepted";
    case Reason::InvalidRequest: return "invalid-request";
    case Reason::InvalidParent: return "invalid-parent";
    case Reason::InvalidCandidate: return "invalid-candidate";
    case Reason::IncompatibleModel: return "incompatible-model";
    case Reason::ParentClockInvalid: return "parent-clock-invalid";
    case Reason::SwitchBeforeCandidateOrigin:
      return "switch-before-candidate-origin";
    case Reason::ParentCursorUnavailable: return "parent-cursor-unavailable";
    case Reason::CandidateCursorUnavailable:
      return "candidate-cursor-unavailable";
    case Reason::StateMismatch: return "state-mismatch";
  }
  return "unknown";
}

Result evaluate(const Request & request) noexcept
{
  Result result;
  if (
    request.parent == nullptr || request.candidate == nullptr ||
    !std::isfinite(request.switch_control_origin_sec) ||
    request.switch_control_origin_sec < 0.0 ||
    (request.circular &&
    (!std::isfinite(request.path_length_m) || request.path_length_m <= 0.0)))
  {
    return result;
  }
  if (certified::validate(*request.parent) != certified::RejectReason::None) {
    result.reason = Reason::InvalidParent;
    return result;
  }
  if (
    certified::validate(*request.candidate) != certified::RejectReason::None)
  {
    result.reason = Reason::InvalidCandidate;
    return result;
  }
  const auto & parent = *request.parent->execution_artifact;
  const auto & candidate = *request.candidate->execution_artifact;
  result.parent_sequence = parent.identity.sequence;
  result.candidate_sequence = candidate.identity.sequence;
  if (!compatible_model(parent, candidate)) {
    result.reason = Reason::IncompatibleModel;
    return result;
  }
  if (
    !std::isfinite(request.parent_first_published_control_origin_sec) ||
    request.parent_first_published_control_origin_sec < 0.0 ||
    !std::isfinite(request.parent_first_published_artifact_elapsed_sec) ||
    request.parent_first_published_artifact_elapsed_sec < 0.0 ||
    request.switch_control_origin_sec + kIdentityTolerance <
    request.parent_first_published_control_origin_sec)
  {
    result.reason = Reason::ParentClockInvalid;
    return result;
  }
  if (
    request.switch_control_origin_sec + kIdentityTolerance <
    candidate.prediction_origin_sec)
  {
    result.reason = Reason::SwitchBeforeCandidateOrigin;
    return result;
  }

  result.parent_elapsed_sec =
    request.parent_first_published_artifact_elapsed_sec +
    request.switch_control_origin_sec -
    request.parent_first_published_control_origin_sec;
  result.candidate_elapsed_sec =
    request.switch_control_origin_sec - candidate.prediction_origin_sec;
  const auto parent_cursor = artifact::resolve_cursor(
    parent, parent.prediction_origin_sec + result.parent_elapsed_sec);
  const auto candidate_cursor = artifact::resolve_cursor(
    candidate, candidate.prediction_origin_sec + result.candidate_elapsed_sec);
  result.parent_cursor_reason = parent_cursor.reason;
  result.candidate_cursor_reason = candidate_cursor.reason;
  if (!parent_cursor.available) {
    result.reason = Reason::ParentCursorUnavailable;
    return result;
  }
  if (!candidate_cursor.available) {
    result.reason = Reason::CandidateCursorUnavailable;
    return result;
  }
  const auto parent_state = sample(*request.parent, parent_cursor);
  if (!parent_state.has_value()) {
    result.reason = Reason::ParentCursorUnavailable;
    return result;
  }
  const auto candidate_state = sample(*request.candidate, candidate_cursor);
  if (!candidate_state.has_value()) {
    result.reason = Reason::CandidateCursorUnavailable;
    return result;
  }
  result.parent_state = parent_state.value();
  result.candidate_state = candidate_state.value();
  result.lateral_difference_m =
    candidate_state->lateral_m - parent_state->lateral_m;
  result.lag_difference_m = candidate_state->lag_m - parent_state->lag_m;
  result.heading_difference_rad = wrap_to_pi(
    candidate_state->heading_offset_rad - parent_state->heading_offset_rad);
  result.velocity_difference_mps =
    candidate_state->velocity_mps - parent_state->velocity_mps;
  result.progress_difference_m = progress_difference(
    candidate_state->absolute_progress_m,
    parent_state->absolute_progress_m, request.path_length_m,
    request.circular);
  result.steering_difference_rad =
    candidate_state->steering_rad - parent_state->steering_rad;
  result.response_steering_difference_rad =
    candidate_state->response_steering_rad -
    parent_state->response_steering_rad;
  result.position_tolerance_m = std::max(
    {kIdentityTolerance,
      request.parent->physical_snapshot->bound_tolerance_m,
      request.candidate->physical_snapshot->bound_tolerance_m,
      artifact::physical_lateral_bound_tolerance_m(parent),
      artifact::physical_lateral_bound_tolerance_m(candidate)});
  result.model_tolerance = std::max(
    {kIdentityTolerance,
      parent.physical_global_tolerance + parent.maximum_constraint_violation,
      candidate.physical_global_tolerance +
      candidate.maximum_constraint_violation});
  const bool position_matches =
    std::abs(result.lateral_difference_m) <= result.position_tolerance_m &&
    std::abs(result.lag_difference_m) <= result.position_tolerance_m &&
    std::abs(result.progress_difference_m) <= result.position_tolerance_m;
  const bool model_matches =
    std::abs(result.heading_difference_rad) <= result.model_tolerance &&
    std::abs(result.velocity_difference_mps) <= result.model_tolerance &&
    std::abs(result.steering_difference_rad) <= result.model_tolerance &&
    std::abs(result.response_steering_difference_rad) <=
    result.model_tolerance;
  result.reason = position_matches && model_matches ?
    Reason::Accepted : Reason::StateMismatch;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_on_trajectory_connector
