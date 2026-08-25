#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation
{
namespace
{

constexpr double kIdentityTolerance = 1e-9;

bool finite_pose(const recovery::Pose2D & pose) noexcept
{
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

bool same_pose(
  const recovery::Pose2D & lhs, const recovery::Pose2D & rhs) noexcept
{
  return finite_pose(lhs) && finite_pose(rhs) &&
         std::abs(lhs.x_m - rhs.x_m) <= kIdentityTolerance &&
         std::abs(lhs.y_m - rhs.y_m) <= kIdentityTolerance &&
         std::abs(lhs.yaw_rad - rhs.yaw_rad) <= kIdentityTolerance;
}

bool same_footprint(
  const recovery::FootprintExtents & lhs,
  const recovery::FootprintExtents & rhs) noexcept
{
  return lhs.valid() && rhs.valid() &&
         lhs.front_extent_m == rhs.front_extent_m &&
         lhs.rear_extent_m == rhs.rear_extent_m &&
         lhs.left_extent_m == rhs.left_extent_m &&
         lhs.right_extent_m == rhs.right_extent_m &&
         lhs.margin_m == rhs.margin_m;
}

bool track_cruise(const contract::ControlIntent intent) noexcept
{
  return intent == contract::ControlIntent::Track ||
         intent == contract::ControlIntent::Cruise;
}

artifact::PredictedState interpolate_expected_state(
  const artifact::ExecutionArtifact & execution,
  const artifact::Cursor & cursor) noexcept
{
  const auto & start =
    execution.predicted_states[cursor.control_stage_index];
  const auto & end =
    execution.predicted_states[cursor.control_stage_index + 1U];
  const double duration =
    execution.control_stages[cursor.control_stage_index].duration_sec;
  const double fraction = cursor.stage_elapsed_sec / duration;
  const auto interpolate = [fraction](const double lhs, const double rhs) {
      return lhs + fraction * (rhs - lhs);
    };
  return artifact::PredictedState{
    interpolate(start.lateral_m, end.lateral_m),
    interpolate(start.lag_m, end.lag_m),
    interpolate(start.heading_offset_rad, end.heading_offset_rad),
    interpolate(start.velocity_mps, end.velocity_mps),
    interpolate(start.progress_m, end.progress_m),
    interpolate(start.steering_rad, end.steering_rad)};
}

struct LiftResult
{
  bool accepted{false};
  double progress_m{};
  long lap_offset{};
};

LiftResult lift_progress(
  const double measured_progress_m, const double retained_progress_m,
  const double path_length_m, const double tolerance_m,
  const bool circular) noexcept
{
  LiftResult result;
  if (!std::isfinite(measured_progress_m) ||
    !std::isfinite(retained_progress_m) ||
    !std::isfinite(path_length_m) || path_length_m <= 0.0 ||
    !std::isfinite(tolerance_m) || tolerance_m < 0.0)
  {
    return result;
  }
  if (!circular) {
    if (std::abs(measured_progress_m - retained_progress_m) >
      tolerance_m + kIdentityTolerance)
    {
      return result;
    }
    result.accepted = true;
    result.progress_m = measured_progress_m;
    return result;
  }
  if (tolerance_m >= 0.5 * path_length_m) {
    return result;
  }
  const double raw_offset =
    (retained_progress_m - measured_progress_m) / path_length_m;
  if (raw_offset < static_cast<double>(std::numeric_limits<long>::min()) ||
    raw_offset > static_cast<double>(std::numeric_limits<long>::max()))
  {
    return result;
  }
  const long lap_offset = std::lround(raw_offset);
  const double lifted = measured_progress_m +
    static_cast<double>(lap_offset) * path_length_m;
  if (std::abs(lifted - retained_progress_m) >
    tolerance_m + kIdentityTolerance)
  {
    return result;
  }
  result.accepted = true;
  result.progress_m = lifted;
  result.lap_offset = lap_offset;
  return result;
}

std::optional<recovery::Pose2D> reconstruct_pose(
  const physical::Snapshot & source,
  const artifact::PredictedState & state) noexcept
{
  const double absolute_progress_m =
    source.trajectory.progress_origin_m + state.progress_m;
  const auto frame = mpc_stage_geometry::sample_course_frame(
    source.course_frame_knots, absolute_progress_m,
    std::max(kIdentityTolerance, source.bound_tolerance_m));
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

double wrap_to_pi(const double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct DynamicPathResult
{
  bool valid{true};
  bool clear{true};
  std::string blocking_obstacle_id;
  std::size_t checked_pose_count{};
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
};

bool dynamic_observation_valid(
  const DynamicWorldObservation & observation) noexcept
{
  if (
    observation.generation == 0U || !std::isfinite(observation.observed_sec) ||
    observation.observed_sec < 0.0)
  {
    return false;
  }
  std::unordered_set<std::string> ids;
  for (const auto & obstacle : observation.obstacles) {
    if (
      obstacle.id.empty() || !ids.emplace(obstacle.id).second ||
      !std::isfinite(obstacle.circle.x_m) ||
      !std::isfinite(obstacle.circle.y_m) ||
      !std::isfinite(obstacle.circle.velocity_x_mps) ||
      !std::isfinite(obstacle.circle.velocity_y_mps) ||
      !std::isfinite(obstacle.circle.radius_m) ||
      obstacle.circle.radius_m < 0.0)
    {
      return false;
    }
  }
  return true;
}

void evaluate_dynamic_pose(
  const recovery::FootprintExtents & footprint,
  const recovery::Pose2D & pose, const double elapsed_time_sec,
  const DynamicWorldObservation & observation,
  DynamicPathResult & result)
{
  if (!result.valid || !result.clear) {
    return;
  }
  for (const auto & obstacle : observation.obstacles) {
    const auto clearance = recovery::circle_obstacle_clearance_at_time(
      footprint, pose, obstacle.circle, elapsed_time_sec);
    if (!clearance.has_value()) {
      result.valid = false;
      result.clear = false;
      result.blocking_obstacle_id = obstacle.id;
      return;
    }
    ++result.checked_pose_count;
    result.minimum_clearance_m = std::min(
      result.minimum_clearance_m, clearance.value());
    if (clearance.value() < -kIdentityTolerance) {
      result.clear = false;
      result.blocking_obstacle_id = obstacle.id;
      return;
    }
  }
}

void evaluate_dynamic_segment(
  const recovery::FootprintExtents & footprint,
  const recovery::Pose2D & start, const recovery::Pose2D & end,
  const double start_time_sec, const double end_time_sec,
  const double swept_step_m, const DynamicWorldObservation & observation,
  DynamicPathResult & result)
{
  // A preceding segment owns the first failure.  Do not reinterpret an
  // already proven physical overlap as malformed geometry when a caller
  // evaluates the next portion of the joined path.
  if (!result.valid || !result.clear) {
    return;
  }
  if (
    !finite_pose(start) || !finite_pose(end) || !std::isfinite(start_time_sec) ||
    !std::isfinite(end_time_sec) || end_time_sec < start_time_sec ||
    !std::isfinite(swept_step_m) || swept_step_m <= 0.0)
  {
    result.valid = false;
    result.clear = false;
    return;
  }
  const double yaw_delta = wrap_to_pi(end.yaw_rad - start.yaw_rad);
  const double corner_radius_m = std::hypot(
    std::max(footprint.front_extent_m, footprint.rear_extent_m) +
    footprint.margin_m,
    std::max(footprint.left_extent_m, footprint.right_extent_m) +
    footprint.margin_m);
  double maximum_obstacle_motion_m = 0.0;
  const double duration_sec = end_time_sec - start_time_sec;
  for (const auto & obstacle : observation.obstacles) {
    maximum_obstacle_motion_m = std::max(
      maximum_obstacle_motion_m,
      std::hypot(
        obstacle.circle.velocity_x_mps,
        obstacle.circle.velocity_y_mps) * duration_sec);
  }
  const double relative_motion_bound_m =
    std::hypot(end.x_m - start.x_m, end.y_m - start.y_m) +
    corner_radius_m * std::abs(yaw_delta) + maximum_obstacle_motion_m;
  const double raw_subdivisions = std::ceil(
    relative_motion_bound_m / swept_step_m);
  if (!std::isfinite(raw_subdivisions) || raw_subdivisions > 1000000.0) {
    result.valid = false;
    result.clear = false;
    return;
  }
  const std::size_t subdivisions = std::max<std::size_t>(
    1U, static_cast<std::size_t>(raw_subdivisions));
  for (std::size_t index = 0U; index <= subdivisions; ++index) {
    const double fraction =
      static_cast<double>(index) / static_cast<double>(subdivisions);
    const recovery::Pose2D pose{
      start.x_m + fraction * (end.x_m - start.x_m),
      start.y_m + fraction * (end.y_m - start.y_m),
      start.yaw_rad + fraction * yaw_delta};
    const double elapsed_time_sec =
      start_time_sec + fraction * duration_sec;
    evaluate_dynamic_pose(
      footprint, pose, elapsed_time_sec, observation, result);
    if (!result.valid || !result.clear) {
      return;
    }
  }
}

void evaluate_timed_dynamic_path(
  const recovery::FootprintExtents & footprint,
  const std::vector<recovery::Pose2D> & path,
  const std::vector<double> & elapsed_sec, const double swept_step_m,
  const DynamicWorldObservation & observation, DynamicPathResult & result)
{
  if (!result.valid || !result.clear) {
    return;
  }
  if (
    path.empty() || path.size() != elapsed_sec.size() ||
    !std::isfinite(elapsed_sec.front()) ||
    std::abs(elapsed_sec.front()) > kIdentityTolerance)
  {
    result.valid = false;
    result.clear = false;
    return;
  }
  if (path.size() == 1U) {
    evaluate_dynamic_pose(
      footprint, path.front(), elapsed_sec.front(), observation, result);
    return;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    evaluate_dynamic_segment(
      footprint, path[index - 1U], path[index],
      elapsed_sec[index - 1U], elapsed_sec[index],
      swept_step_m, observation, result);
    if (!result.valid || !result.clear) {
      return;
    }
  }
}

}  // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Accepted: return "accepted";
    case Reason::MissingPlan: return "missing-plan";
    case Reason::InvalidPlan: return "invalid-plan";
    case Reason::CursorUnavailable: return "cursor-unavailable";
    case Reason::IntentMismatch: return "intent-mismatch";
    case Reason::DynamicObservationUnavailable:
      return "dynamic-observation-unavailable";
    case Reason::DynamicObservationInvalid:
      return "dynamic-observation-invalid";
    case Reason::DynamicPathInvalid: return "dynamic-path-invalid";
    case Reason::DynamicPathBlocked: return "dynamic-path-blocked";
    case Reason::StaticWorldMismatch: return "static-world-mismatch";
    case Reason::InvalidCurrentState: return "invalid-current-state";
    case Reason::ProgressLiftRejected: return "progress-lift-rejected";
    case Reason::CourseFrameUnavailable: return "course-frame-unavailable";
    case Reason::ActuationRejected: return "actuation-rejected";
    case Reason::SteeringUnreachable: return "steering-unreachable";
    case Reason::VelocityUnreachable: return "velocity-unreachable";
    case Reason::ControlPathInvalid: return "control-path-invalid";
    case Reason::DelayPrefixBlocked: return "delay-prefix-blocked";
    case Reason::ConnectorBlocked: return "connector-blocked";
    case Reason::Count: break;
  }
  return "unknown";
}

Result evaluate(const Request & request)
{
  Result result;
  if (request.plan == nullptr) {
    return result;
  }
  if (certified::validate(*request.plan) != certified::RejectReason::None) {
    result.reason = Reason::InvalidPlan;
    return result;
  }
  const auto & execution = *request.plan->execution_artifact;
  const auto & source = *request.plan->physical_snapshot;
  const auto cursor = artifact::resolve_cursor(
    execution, request.control_origin_sec);
  result.cursor_reason = cursor.reason;
  if (!cursor.available) {
    result.reason = Reason::CursorUnavailable;
    return result;
  }
  if (!track_cruise(request.current_intent) ||
    request.current_intent != execution.identity.source_context.intent)
  {
    result.reason = Reason::IntentMismatch;
    return result;
  }
  if (!request.obstacles.current) {
    result.reason = Reason::DynamicObservationUnavailable;
    return result;
  }
  if (!dynamic_observation_valid(request.obstacles) ||
    request.obstacles.observed_sec > request.now_sec + kIdentityTolerance)
  {
    result.reason = Reason::DynamicObservationInvalid;
    return result;
  }
  if (request.current_wall_grid == nullptr ||
    request.current_wall_grid.get() != source.wall_grid.get() ||
    !same_footprint(request.current_footprint, source.footprint))
  {
    result.reason = Reason::StaticWorldMismatch;
    return result;
  }
  if (request.decision_id == 0U || !std::isfinite(request.now_sec) ||
    !std::isfinite(request.current_speed_mps) ||
    request.current_speed_mps < 0.0 ||
    !std::isfinite(request.current_steering_rad) ||
    !std::isfinite(request.minimum_acceleration_mps2) ||
    !std::isfinite(request.maximum_acceleration_mps2) ||
    request.minimum_acceleration_mps2 > request.maximum_acceleration_mps2 ||
    !std::isfinite(request.publication_interval_sec) ||
    request.publication_interval_sec <= 0.0 ||
    !std::isfinite(request.control_origin_sec) ||
    request.control_origin_sec < request.now_sec ||
    request.measured_to_control_path.empty() ||
    request.measured_to_control_path.size() !=
    request.measured_to_control_elapsed_sec.size() ||
    !std::isfinite(request.measured_to_control_elapsed_sec.front()) ||
    std::abs(request.measured_to_control_elapsed_sec.front()) >
    kIdentityTolerance ||
    !std::isfinite(request.measured_to_control_elapsed_sec.back()) ||
    std::abs(
      request.measured_to_control_elapsed_sec.back() -
      (request.control_origin_sec - request.now_sec)) > kIdentityTolerance ||
    !same_pose(request.measured_to_control_path.back(), request.control_pose))
  {
    result.reason = Reason::InvalidCurrentState;
    return result;
  }

  const auto expected = interpolate_expected_state(execution, cursor);
  const double expected_absolute_progress_m =
    execution.course_progress_origin_m + expected.progress_m;
  const auto lift = lift_progress(
    request.measured_course_progress_m, expected_absolute_progress_m,
    request.path_length_m, request.progress_continuity_tolerance_m,
    request.circular);
  if (!lift.accepted) {
    result.reason = Reason::ProgressLiftRejected;
    return result;
  }
  const auto expected_pose = reconstruct_pose(source, expected);
  if (!expected_pose.has_value()) {
    result.reason = Reason::CourseFrameUnavailable;
    return result;
  }
  const auto actuation = artifact::extract_actuation(execution, cursor);
  result.actuation_reason = actuation.reason;
  if (!actuation.actuation.has_value()) {
    result.reason = Reason::ActuationRejected;
    return result;
  }

  const double steering_difference_rad =
    actuation.actuation->steering_rad - request.current_steering_rad;
  const double maximum_steering_step_rad =
    execution.maximum_abs_steering_rate_radps *
    request.publication_interval_sec + execution.physical_global_tolerance;
  result.steering_difference_rad = steering_difference_rad;
  result.maximum_steering_step_rad = maximum_steering_step_rad;
  if (std::abs(steering_difference_rad) > maximum_steering_step_rad) {
    result.reason = Reason::SteeringUnreachable;
    return result;
  }
  const double velocity_reachability_duration_sec =
    request.control_origin_sec - request.now_sec;
  const double velocity_lower_mps = std::max(
    0.0, request.current_speed_mps +
    request.minimum_acceleration_mps2 * velocity_reachability_duration_sec -
    execution.physical_global_tolerance);
  const double velocity_upper_mps = std::max(
    0.0, request.current_speed_mps +
    request.maximum_acceleration_mps2 * velocity_reachability_duration_sec +
    execution.physical_global_tolerance);
  result.velocity_difference_mps =
    actuation.actuation->predicted_speed_mps - request.current_speed_mps;
  result.reachable_velocity_lower_mps = velocity_lower_mps;
  result.reachable_velocity_upper_mps = velocity_upper_mps;
  result.velocity_reachability_duration_sec =
    velocity_reachability_duration_sec;
  if (actuation.actuation->predicted_speed_mps < velocity_lower_mps ||
    actuation.actuation->predicted_speed_mps > velocity_upper_mps)
  {
    result.reason = Reason::VelocityUnreachable;
    return result;
  }

  auto clearance_footprint = source.footprint;
  clearance_footprint.left_extent_m += source.hard_wall_clearance_m;
  clearance_footprint.right_extent_m += source.hard_wall_clearance_m;
  const auto delay = recovery::evaluate_clear_footprint_path(
    *source.wall_grid, clearance_footprint,
    request.measured_to_control_path, source.swept_step_m);
  if (!delay.valid) {
    result.reason = Reason::ControlPathInvalid;
    return result;
  }
  if (!delay.clear) {
    result.reason = Reason::DelayPrefixBlocked;
    return result;
  }
  const std::vector<recovery::Pose2D> connector{
    request.control_pose, expected_pose.value()};
  const auto connector_result = recovery::evaluate_clear_footprint_path(
    *source.wall_grid, clearance_footprint, connector,
    source.swept_step_m);
  if (!connector_result.valid) {
    result.reason = Reason::ControlPathInvalid;
    return result;
  }
  if (!connector_result.clear) {
    result.reason = Reason::ConnectorBlocked;
    return result;
  }

  DynamicPathResult dynamic;
  evaluate_timed_dynamic_path(
    source.footprint, request.measured_to_control_path,
    request.measured_to_control_elapsed_sec, source.swept_step_m,
    request.obstacles, dynamic);
  const double prediction_delay_sec =
    request.control_origin_sec - request.now_sec;
  evaluate_dynamic_segment(
    source.footprint, connector.front(), connector.back(),
    prediction_delay_sec, prediction_delay_sec, source.swept_step_m,
    request.obstacles, dynamic);
  auto previous_dynamic_pose = expected_pose.value();
  double dynamic_time_sec = prediction_delay_sec;
  for (
    std::size_t stage_index = cursor.control_stage_index;
    stage_index < execution.control_stages.size(); ++stage_index)
  {
    const auto endpoint_pose = reconstruct_pose(
      source, execution.predicted_states[stage_index + 1U]);
    if (!endpoint_pose.has_value()) {
      result.reason = Reason::CourseFrameUnavailable;
      return result;
    }
    const double segment_duration_sec = stage_index == cursor.control_stage_index ?
      execution.control_stages[stage_index].duration_sec -
      cursor.stage_elapsed_sec :
      execution.control_stages[stage_index].duration_sec;
    evaluate_dynamic_segment(
      source.footprint, previous_dynamic_pose, endpoint_pose.value(),
      dynamic_time_sec, dynamic_time_sec + segment_duration_sec,
      source.swept_step_m, request.obstacles, dynamic);
    dynamic_time_sec += segment_duration_sec;
    previous_dynamic_pose = endpoint_pose.value();
    if (!dynamic.valid || !dynamic.clear) {
      break;
    }
  }
  result.blocking_obstacle_id = dynamic.blocking_obstacle_id;
  result.dynamic_checked_pose_count = dynamic.checked_pose_count;
  result.minimum_dynamic_clearance_m = dynamic.minimum_clearance_m;
  if (!dynamic.valid) {
    result.reason = Reason::DynamicPathInvalid;
    return result;
  }
  if (!dynamic.clear) {
    result.reason = Reason::DynamicPathBlocked;
    return result;
  }

  Proof proof;
  proof.plan = request.plan;
  proof.decision_id = request.decision_id;
  proof.obstacle_generation = request.obstacles.generation;
  proof.observed_sec = request.obstacles.observed_sec;
  proof.observation_origin_sec = request.now_sec;
  proof.control_origin_sec = request.control_origin_sec;
  proof.prediction_delay_sec = prediction_delay_sec;
  proof.cursor = cursor;
  proof.actuation = actuation.actuation.value();
  proof.expected_current_state = expected;
  proof.expected_current_pose = expected_pose.value();
  proof.expected_absolute_progress_m = expected_absolute_progress_m;
  proof.lifted_measured_progress_m = lift.progress_m;
  proof.lap_offset = lift.lap_offset;
  proof.steering_difference_rad = steering_difference_rad;
  proof.maximum_steering_step_rad = maximum_steering_step_rad;
  proof.velocity_difference_mps = result.velocity_difference_mps;
  proof.reachable_velocity_lower_mps = result.reachable_velocity_lower_mps;
  proof.reachable_velocity_upper_mps = result.reachable_velocity_upper_mps;
  proof.velocity_reachability_duration_sec =
    result.velocity_reachability_duration_sec;
  proof.delay_checked_pose_count = delay.checked_pose_count;
  proof.connector_checked_pose_count = connector_result.checked_pose_count;
  proof.dynamic_checked_pose_count = dynamic.checked_pose_count;
  proof.minimum_dynamic_clearance_m = dynamic.minimum_clearance_m;
  result.reason = Reason::Accepted;
  result.proof = std::move(proof);
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation
