#include "multi_purpose_mpc_ros/v2x_overtake_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace multi_purpose_mpc_ros::v2x_overtake_core
{
namespace
{

bool is_configured_side(const PassSide side) noexcept
{
  return side == PassSide::Left || side == PassSide::Right;
}

bool is_feasible(const SideSelectionRequest & request, const PassSide side) noexcept
{
  if (side == PassSide::Left) {
    return request.left_feasible;
  }
  if (side == PassSide::Right) {
    return request.right_feasible;
  }
  return false;
}

void validate_speed(const double speed_mps, const char * name)
{
  if (!std::isfinite(speed_mps) || speed_mps < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

}  // namespace

SpeedLimitResolution resolve_effective_speed_limit(const SpeedLimitRequest & request)
{
  validate_speed(request.normal_speed_mps, "normal speed");
  validate_speed(request.global_hard_cap_mps, "global hard cap");
  if (!std::isfinite(request.start_window_duration_sec) ||
    request.start_window_duration_sec < 0.0)
  {
    throw std::invalid_argument("Start window duration must be finite and non-negative");
  }
  if (request.start_speed_mps.has_value()) {
    validate_speed(*request.start_speed_mps, "Start speed");
  }

  const double normal_speed_mps =
    std::min(request.normal_speed_mps, request.global_hard_cap_mps);
  if (!request.start_speed_mps.has_value() || request.start_window_duration_sec == 0.0) {
    return {normal_speed_mps, StartWindowStatus::NotConfigured};
  }
  if (!request.elapsed_since_start_sec.has_value()) {
    return {normal_speed_mps, StartWindowStatus::AwaitingStart};
  }

  const double elapsed_sec = *request.elapsed_since_start_sec;
  if (!std::isfinite(elapsed_sec) || elapsed_sec < 0.0) {
    return {normal_speed_mps, StartWindowStatus::InvalidElapsed};
  }
  if (elapsed_sec >= request.start_window_duration_sec) {
    return {normal_speed_mps, StartWindowStatus::Expired};
  }

  return {
    std::min(*request.start_speed_mps, request.global_hard_cap_mps),
    StartWindowStatus::Applied};
}

const char * to_string(const StartWindowStatus status) noexcept
{
  switch (status) {
    case StartWindowStatus::NotConfigured:
      return "normal";
    case StartWindowStatus::AwaitingStart:
      return "awaiting_start";
    case StartWindowStatus::InvalidElapsed:
      return "invalid_elapsed";
    case StartWindowStatus::Applied:
      return "start_window";
    case StartWindowStatus::Expired:
      return "expired_normal";
  }
  return "unknown";
}

OvertakeSpeedReferenceResolution resolve_overtake_speed_reference(
  const OvertakeSpeedReferenceRequest & request)
{
  validate_speed(request.base_reference_speed_mps, "base overtake reference");
  validate_speed(request.hard_cap_mps, "overtake hard cap");
  validate_speed(request.front_speed_mps, "front speed");
  validate_speed(request.entry_speed_mps, "overtake entry speed");
  validate_speed(request.shiftout_max_closing_speed_mps, "ShiftOut closing speed");

  const double base_reference =
    std::min(request.base_reference_speed_mps, request.hard_cap_mps);
  if (request.stage == OvertakeSpeedStage::Pass) {
    return {base_reference, false};
  }

  const double front_cap = std::min(
    request.hard_cap_mps,
    std::max(
      request.entry_speed_mps,
      request.front_speed_mps + request.shiftout_max_closing_speed_mps));
  return {std::min(base_reference, front_cap), true};
}

double advance_prediction_time(const PredictionTimeRequest & request)
{
  if (!std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0) {
    throw std::invalid_argument("Prediction elapsed time must be finite and non-negative");
  }
  if (!std::isfinite(request.segment_distance_m) || request.segment_distance_m < 0.0) {
    throw std::invalid_argument("Prediction segment distance must be finite and non-negative");
  }
  validate_speed(request.predicted_speed_mps, "predicted path speed");
  if (!std::isfinite(request.minimum_speed_mps) || request.minimum_speed_mps <= 0.0) {
    throw std::invalid_argument("Prediction minimum speed must be finite and positive");
  }
  if (!std::isfinite(request.maximum_time_sec) || request.maximum_time_sec < 0.0) {
    throw std::invalid_argument("Prediction maximum time must be finite and non-negative");
  }

  const double segment_time = request.segment_distance_m /
    std::max(request.predicted_speed_mps, request.minimum_speed_mps);
  return std::min(request.maximum_time_sec, request.elapsed_sec + segment_time);
}

ForwardCourseProjection project_forward_course_progress(
  const std::vector<CoursePoint> & path, const ForwardCourseProjectionRequest & request)
{
  if (path.size() < 2U) {
    throw std::invalid_argument("Course projection requires at least two path points");
  }
  const auto finite = [](const double value) {return std::isfinite(value);};
  if (
    !finite(request.origin_x_m) || !finite(request.origin_y_m) ||
    !finite(request.target_x_m) || !finite(request.target_y_m) ||
    !finite(request.target_vx_mps) || !finite(request.target_vy_mps))
  {
    throw std::invalid_argument("Course projection pose and velocity must be finite");
  }
  if (
    !finite(request.lookbehind_distance_m) || request.lookbehind_distance_m < 0.0 ||
    !finite(request.lookahead_distance_m) || request.lookahead_distance_m <= 0.0 ||
    !finite(request.max_cross_track_distance_m) ||
    request.max_cross_track_distance_m <= 0.0)
  {
    throw std::invalid_argument(
            "Course projection distances must be finite with positive lookahead/cross-track");
  }
  for (const auto & point : path) {
    if (!finite(point.x_m) || !finite(point.y_m)) {
      throw std::invalid_argument("Course projection path points must be finite");
    }
  }

  const std::size_t point_count = path.size();
  const auto next_index = [&](const std::size_t index) {
      if (index + 1U < point_count) {
        return index + 1U;
      }
      return request.circular ? 0U : index;
    };
  const auto previous_index = [&](const std::size_t index) {
      if (index > 0U) {
        return index - 1U;
      }
      return request.circular ? point_count - 1U : index;
    };
  const auto segment_length = [&](const std::size_t from, const std::size_t to) {
      return std::hypot(path[to].x_m - path[from].x_m, path[to].y_m - path[from].y_m);
    };

  std::size_t anchor = request.circular ?
    request.start_index % point_count : std::min(request.start_index, point_count - 1U);
  double distance_before_anchor_m = 0.0;
  for (std::size_t step = 0U;
    step + 1U < point_count &&
    distance_before_anchor_m + 1e-9 < request.lookbehind_distance_m; ++step)
  {
    const std::size_t previous = previous_index(anchor);
    if (previous == anchor) {
      break;
    }
    const double length_m = segment_length(previous, anchor);
    anchor = previous;
    // Runtime ReferencePath may contain an appended circular closure point.
    // Treat a zero-length closure or repeated interior sample as no progress;
    // rejecting the entire control cycle would turn a harmless path encoding
    // detail into a permanent MPC fallback at the lap seam.
    if (!finite(length_m) || length_m <= 1e-9) {
      continue;
    }
    distance_before_anchor_m += length_m;
  }

  struct Candidate
  {
    bool valid{false};
    double score{std::numeric_limits<double>::infinity()};
    double progress_m{};
    double lateral_m{};
    double cross_track_m{};
    double tangent_x{};
    double tangent_y{};
    std::size_t segment_index{};
  };
  Candidate origin;
  Candidate target;
  const double target_speed_mps = std::hypot(request.target_vx_mps, request.target_vy_mps);
  double segment_start_progress_m = -distance_before_anchor_m;
  std::size_t current = anchor;
  const std::size_t maximum_segments = request.circular ? point_count : point_count - 1U;

  const auto consider = [&](
      Candidate & best, const double px, const double py, const std::size_t from,
      const std::size_t to, const double length_m, const double start_progress_m,
      const bool target_candidate) {
      const double tangent_x = (path[to].x_m - path[from].x_m) / length_m;
      const double tangent_y = (path[to].y_m - path[from].y_m) / length_m;
      const double dx = px - path[from].x_m;
      const double dy = py - path[from].y_m;
      const double along_m = std::clamp(dx * tangent_x + dy * tangent_y, 0.0, length_m);
      const double projected_x = path[from].x_m + along_m * tangent_x;
      const double projected_y = path[from].y_m + along_m * tangent_y;
      const double residual_x = px - projected_x;
      const double residual_y = py - projected_y;
      const double lateral_m = -tangent_y * residual_x + tangent_x * residual_y;
      const double cross_track_m = std::hypot(residual_x, residual_y);
      if (cross_track_m > request.max_cross_track_distance_m) {
        return;
      }
      double score = cross_track_m * cross_track_m;
      if (target_candidate && target_speed_mps > 0.25) {
        const double direction_alignment =
          (request.target_vx_mps * tangent_x + request.target_vy_mps * tangent_y) /
          target_speed_mps;
        if (direction_alignment < -0.2) {
          return;
        }
        score += 0.04 * (1.0 - std::clamp(direction_alignment, -1.0, 1.0));
      }
      const double progress_m = start_progress_m + along_m;
      if (
        !best.valid || score + 1e-12 < best.score ||
        (std::abs(score - best.score) <= 1e-12 && progress_m < best.progress_m))
      {
        best.valid = true;
        best.score = score;
        best.progress_m = progress_m;
        best.lateral_m = lateral_m;
        best.cross_track_m = cross_track_m;
        best.tangent_x = tangent_x;
        best.tangent_y = tangent_y;
        best.segment_index = from;
      }
    };

  for (std::size_t step = 0U; step < maximum_segments; ++step) {
    const std::size_t next = next_index(current);
    if (next == current) {
      break;
    }
    const double length_m = segment_length(current, next);
    if (!finite(length_m) || length_m <= 1e-9) {
      current = next;
      continue;
    }
    consider(
      origin, request.origin_x_m, request.origin_y_m, current, next, length_m,
      segment_start_progress_m, false);
    consider(
      target, request.target_x_m, request.target_y_m, current, next, length_m,
      segment_start_progress_m, true);
    segment_start_progress_m += length_m;
    current = next;
    if (segment_start_progress_m > request.lookahead_distance_m + length_m) {
      break;
    }
  }

  ForwardCourseProjection result;
  if (!origin.valid || !target.valid) {
    return result;
  }
  result.forward_distance_m = target.progress_m - origin.progress_m;
  if (
    result.forward_distance_m < -request.lookbehind_distance_m - 1e-9 ||
    result.forward_distance_m > request.lookahead_distance_m + 1e-9)
  {
    return ForwardCourseProjection{};
  }
  result.valid = true;
  result.lateral_m = target.lateral_m;
  result.along_track_speed_mps = std::max(
    0.0,
    request.target_vx_mps * target.tangent_x +
    request.target_vy_mps * target.tangent_y);
  result.cross_track_distance_m = target.cross_track_m;
  result.segment_index = target.segment_index;
  return result;
}

PassCompletionResolution resolve_pass_completion(const PassCompletionRequest & request)
{
  if (
    std::isnan(request.distance_to_hard_curve_m) || request.distance_to_hard_curve_m < 0.0)
  {
    throw std::invalid_argument("Hard-curve distance must be non-negative or positive infinity");
  }
  validate_speed(request.curve_buffer_m, "hard-curve buffer");
  validate_speed(request.front_distance_m, "front distance");
  validate_speed(request.front_speed_mps, "front speed");
  if (!std::isfinite(request.planned_ego_speed_mps) || request.planned_ego_speed_mps <= 0.0) {
    throw std::invalid_argument("Planned ego speed must be finite and positive");
  }
  validate_speed(request.return_clear_distance_m, "return clear distance");
  validate_speed(request.minimum_shift_distance_m, "minimum ShiftOut distance");
  validate_speed(request.merge_buffer_m, "merge buffer");
  if (
    !std::isfinite(request.minimum_relative_speed_mps) ||
    request.minimum_relative_speed_mps <= 0.0)
  {
    throw std::invalid_argument("Minimum relative speed must be finite and positive");
  }

  PassCompletionResolution resolution;
  resolution.available_distance_m = std::isinf(request.distance_to_hard_curve_m) ?
    request.distance_to_hard_curve_m :
    std::max(0.0, request.distance_to_hard_curve_m - request.curve_buffer_m);
  resolution.relative_speed_mps = request.planned_ego_speed_mps - request.front_speed_mps;
  resolution.required_distance_m = std::numeric_limits<double>::infinity();
  if (resolution.relative_speed_mps < request.minimum_relative_speed_mps) {
    return resolution;
  }

  const double relative_gain = request.front_distance_m + request.return_clear_distance_m;
  const double pass_time = relative_gain / resolution.relative_speed_mps;
  const double pass_distance = request.planned_ego_speed_mps * pass_time;
  resolution.required_distance_m =
    std::max(request.minimum_shift_distance_m, pass_distance) + request.merge_buffer_m;
  resolution.feasible =
    resolution.available_distance_m + 1e-9 >= resolution.required_distance_m;
  return resolution;
}

SideSelection select_pass_side(const SideSelectionRequest & request) noexcept
{
  if (is_configured_side(request.locked)) {
    if (is_feasible(request, request.locked)) {
      return {request.locked, SideSelectionReason::Locked};
    }
    return {PassSide::None, SideSelectionReason::LockedUnavailable};
  }

  if (!is_configured_side(request.preferred)) {
    return {PassSide::None, SideSelectionReason::InvalidPreference};
  }
  if (is_feasible(request, request.preferred)) {
    return {request.preferred, SideSelectionReason::Preferred};
  }

  const PassSide alternate = opposite_side(request.preferred);
  if (request.allow_alternate && is_feasible(request, alternate)) {
    return {alternate, SideSelectionReason::Alternate};
  }
  if (is_feasible(request, alternate)) {
    return {PassSide::None, SideSelectionReason::PreferredUnavailable};
  }
  return {PassSide::None, SideSelectionReason::NoFeasibleSide};
}

PassSide opposite_side(const PassSide side) noexcept
{
  if (side == PassSide::Left) {
    return PassSide::Right;
  }
  if (side == PassSide::Right) {
    return PassSide::Left;
  }
  return PassSide::None;
}

ContinuityAction resolve_target_continuity(const ContinuityRequest & request)
{
  if (std::isnan(request.target_age_sec) || request.target_age_sec < 0.0) {
    throw std::invalid_argument("target age must be finite and non-negative");
  }
  if (!std::isfinite(request.target_hold_sec) || request.target_hold_sec < 0.0) {
    throw std::invalid_argument("target hold must be finite and non-negative");
  }
  if (request.solver_recovery_requested || request.target_position_jump) {
    return ContinuityAction::Recovery;
  }
  if (!std::isfinite(request.target_age_sec)) {
    return ContinuityAction::Recovery;
  }
  if (request.rear_clear_confirmed && !request.side_vehicle_present) {
    return ContinuityAction::Return;
  }
  if (
    request.rear_clear_observed ||
    (request.target_seen && request.target_not_ahead) ||
    (!request.target_seen && request.target_age_sec <= request.target_hold_sec))
  {
    return ContinuityAction::Hold;
  }
  return ContinuityAction::Recovery;
}

bool can_reacquire_during_return(const ReacquireRequest & request) noexcept
{
  return request.enabled && request.stable_target_id && request.same_target &&
         request.same_side && request.gap_available && request.execution_allowed &&
         std::isfinite(request.return_elapsed_sec) && request.return_elapsed_sec >= 0.0 &&
         std::isfinite(request.reacquire_window_sec) && request.reacquire_window_sec >= 0.0 &&
         request.return_elapsed_sec <= request.reacquire_window_sec &&
         std::isfinite(request.return_progress) && request.return_progress >= 0.0 &&
         std::isfinite(request.max_return_progress) && request.max_return_progress >= 0.0 &&
         request.return_progress <= request.max_return_progress;
}

ForwardDistanceResolution integrate_forward_distance(const ForwardDistanceRequest & request)
{
  if (
    !std::isfinite(request.accumulated_distance_m) ||
    request.accumulated_distance_m < 0.0)
  {
    throw std::invalid_argument("accumulated forward distance must be finite and non-negative");
  }
  if (
    !std::isfinite(request.max_observation_gap_sec) ||
    request.max_observation_gap_sec <= 0.0)
  {
    throw std::invalid_argument("maximum observation gap must be finite and positive");
  }

  ForwardDistanceResolution resolution{request.accumulated_distance_m, false};
  if (
    !std::isfinite(request.forward_speed_mps) || !std::isfinite(request.delta_sec) ||
    request.delta_sec < 0.0 || request.delta_sec > request.max_observation_gap_sec)
  {
    return resolution;
  }

  resolution.accumulated_distance_m +=
    std::max(0.0, request.forward_speed_mps) * request.delta_sec;
  resolution.observation_accepted = true;
  return resolution;
}

RecoveryPolicyResolution resolve_recovery_policy(const RecoveryPolicyRequest & request)
{
  validate_speed(request.configured_velocity_limit_mps, "Recovery velocity limit");
  if (request.configured_velocity_limit_mps <= 0.0) {
    throw std::invalid_argument("Recovery velocity limit must be positive");
  }
  if (!std::isfinite(request.target_distance_m) || request.target_distance_m <= 0.0) {
    throw std::invalid_argument("Recovery target distance must be finite and positive");
  }
  if (!std::isfinite(request.lateral_completion_m) || request.lateral_completion_m < 0.0) {
    throw std::invalid_argument("Recovery lateral completion must be finite and non-negative");
  }
  if (!std::isfinite(request.stall_timeout_sec) || request.stall_timeout_sec <= 0.0) {
    throw std::invalid_argument("Recovery stall timeout must be finite and positive");
  }
  if (!std::isfinite(request.timeout_sec) || request.timeout_sec <= 0.0) {
    throw std::invalid_argument("Recovery timeout must be finite and positive");
  }
  if (request.stall_timeout_sec > request.timeout_sec) {
    throw std::invalid_argument("Recovery stall timeout must not exceed total timeout");
  }

  RecoveryPolicyResolution resolution{request.configured_velocity_limit_mps,
    RecoveryExitReason::Active};
  if (
    !std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0 ||
    !std::isfinite(request.traveled_distance_m) || request.traveled_distance_m < 0.0 ||
    !std::isfinite(request.lateral_error_m) ||
    !std::isfinite(request.stalled_sec) || request.stalled_sec < 0.0)
  {
    resolution.exit_reason = RecoveryExitReason::InvalidObservation;
    return resolution;
  }
  if (std::abs(request.lateral_error_m) <= request.lateral_completion_m) {
    resolution.exit_reason = RecoveryExitReason::LateralComplete;
    return resolution;
  }
  if (request.traveled_distance_m >= request.target_distance_m) {
    resolution.exit_reason = RecoveryExitReason::DistanceComplete;
    return resolution;
  }
  if (request.stalled_sec >= request.stall_timeout_sec) {
    resolution.exit_reason = RecoveryExitReason::Stalled;
    return resolution;
  }
  if (request.elapsed_sec >= request.timeout_sec) {
    resolution.exit_reason = RecoveryExitReason::TimedOut;
  }
  return resolution;
}

const char * to_string(const RecoveryExitReason reason) noexcept
{
  switch (reason) {
    case RecoveryExitReason::Active:
      return "active";
    case RecoveryExitReason::DistanceComplete:
      return "distance complete";
    case RecoveryExitReason::LateralComplete:
      return "lateral complete";
    case RecoveryExitReason::Stalled:
      return "stalled";
    case RecoveryExitReason::TimedOut:
      return "timed out";
    case RecoveryExitReason::InvalidObservation:
      return "invalid observation";
  }
  return "unknown";
}

double arm_solver_cooldown(const SolverCooldownRequest & request)
{
  if (!std::isfinite(request.now_sec)) {
    throw std::invalid_argument("Solver cooldown time must be finite");
  }
  if (!std::isfinite(request.duration_sec) || request.duration_sec < 0.0) {
    throw std::invalid_argument("Solver cooldown duration must be finite and non-negative");
  }
  if (
    std::isnan(request.current_until_sec) ||
    request.current_until_sec == std::numeric_limits<double>::infinity())
  {
    throw std::invalid_argument("Solver cooldown deadline must be finite or negative infinity");
  }
  if (request.duration_sec == 0.0) {
    return request.current_until_sec;
  }

  const double candidate_until_sec = request.now_sec + request.duration_sec;
  if (!std::isfinite(candidate_until_sec)) {
    throw std::invalid_argument("Solver cooldown deadline overflowed");
  }
  return std::max(request.current_until_sec, candidate_until_sec);
}

bool is_solver_cooldown_active(
  const double now_sec, const double cooldown_until_sec) noexcept
{
  return std::isfinite(now_sec) && std::isfinite(cooldown_until_sec) &&
         now_sec < cooldown_until_sec;
}

}  // namespace multi_purpose_mpc_ros::v2x_overtake_core
