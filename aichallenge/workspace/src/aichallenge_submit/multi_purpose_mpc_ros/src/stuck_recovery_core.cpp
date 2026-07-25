#include "multi_purpose_mpc_ros/stuck_recovery_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace multi_purpose_mpc_ros::stuck_recovery
{
namespace
{

constexpr double kTimestampEpsilon = 1.0e-9;
constexpr double kDistanceComparisonEpsilon = 1.0e-9;

void validate_nonnegative(const double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

void validate_detector_config(const DetectorConfig & config)
{
  validate_nonnegative(config.stopped_speed_mps, "stopped speed");
  validate_nonnegative(config.moving_speed_mps, "moving speed");
  validate_nonnegative(
    config.solver_fallback_duration_sec,
    "solver fallback duration");
  validate_nonnegative(
    config.solver_evidence_free_duration_sec,
    "solver evidence-free recovery duration");
  validate_nonnegative(
    config.evidence_free_duration_sec,
    "evidence-free recovery duration");
  validate_nonnegative(
    config.coordinated_stop_duration_sec,
    "coordinated stop recovery duration");
  validate_nonnegative(
    config.max_observation_gap_sec,
    "maximum observation gap");
  if (config.solver_fallback_recovery_enabled &&
    config.solver_fallback_duration_sec <= 0.0)
  {
    throw std::invalid_argument(
            "enabled solver fallback recovery requires a positive duration");
  }
  if (
    config.solver_evidence_free_recovery_enabled &&
    (!config.solver_fallback_recovery_enabled ||
    config.solver_evidence_free_duration_sec <= 0.0 ||
    config.solver_evidence_free_duration_sec < config.stationary_duration_sec ||
    config.solver_evidence_free_duration_sec < config.solver_fallback_duration_sec))
  {
    throw std::invalid_argument(
            "enabled solver evidence-free recovery requires solver fallback recovery and a "
            "duration no shorter than stationary and solver fallback durations");
  }
  if (
    config.evidence_free_recovery_enabled &&
    (config.evidence_free_duration_sec <= 0.0 ||
    config.evidence_free_duration_sec < config.stationary_duration_sec))
  {
    throw std::invalid_argument(
            "enabled evidence-free recovery requires a duration no shorter than stationary duration");
  }
  if (
    config.coordinated_stop_recovery_enabled &&
    (config.coordinated_stop_duration_sec <= 0.0 ||
    config.coordinated_stop_duration_sec < config.stationary_duration_sec))
  {
    throw std::invalid_argument(
            "enabled coordinated stop recovery requires a duration no shorter than stationary duration");
  }
  if (config.max_observation_gap_sec <= 0.0) {
    throw std::invalid_argument("maximum observation gap must be positive");
  }
  validate_nonnegative(config.forward_intent_speed_mps, "forward intent speed");
  validate_nonnegative(
    config.forward_intent_acceleration_mps2, "forward intent acceleration");
  validate_nonnegative(config.stationary_duration_sec, "stationary duration");
  validate_nonnegative(config.max_pose_displacement_m, "maximum pose displacement");
  validate_nonnegative(config.max_progress_delta_m, "maximum progress delta");
  if (config.moving_speed_mps < config.stopped_speed_mps) {
    throw std::invalid_argument("moving speed must be greater than or equal to stopped speed");
  }
  if (
    config.forward_intent_speed_mps == 0.0 &&
    config.forward_intent_acceleration_mps2 == 0.0)
  {
    throw std::invalid_argument("at least one forward intent threshold must be positive");
  }
}

void validate_supervisor_config(const SupervisorConfig & config)
{
  validate_nonnegative(config.awsim_recovery_wait_sec, "AWSIM recovery wait");
  validate_nonnegative(config.stop_speed_mps, "stop speed");
  validate_nonnegative(config.stop_confirm_sec, "stop confirmation duration");
  validate_nonnegative(config.clearance_wait_timeout_sec, "clearance wait timeout");
  validate_nonnegative(
    config.safe_stop_clear_confirm_sec,
    "safe-stop clearance confirmation duration");
  validate_nonnegative(config.aggressive_retry_delay_sec, "aggressive retry delay");
  validate_nonnegative(config.gear_report_timeout_sec, "gear report timeout");
  validate_nonnegative(
    config.gear_command_resend_interval_sec,
    "gear command resend interval");
  validate_nonnegative(
    config.max_reverse_distance_m,
    "maximum reverse distance");
  validate_nonnegative(
    config.max_reverse_duration_sec,
    "maximum reverse duration");
  validate_nonnegative(config.max_reverse_speed_mps, "maximum reverse speed");
  validate_nonnegative(
    config.reverse_acceleration_magnitude_mps2,
    "reverse acceleration magnitude");
  validate_nonnegative(config.max_forward_distance_m, "maximum forward distance");
  validate_nonnegative(config.max_forward_duration_sec, "maximum forward duration");
  validate_nonnegative(config.max_forward_speed_mps, "maximum forward speed");
  validate_nonnegative(
    config.forward_acceleration_magnitude_mps2,
    "forward acceleration magnitude");
  validate_nonnegative(config.escape_step_distance_m, "escape step distance");
  if (config.escape_step_distance_m <= 0.0) {
    throw std::invalid_argument("escape step distance must be positive");
  }
  validate_nonnegative(config.rejoin_speed_limit_mps, "rejoin speed limit");
  if (config.rejoin_speed_limit_mps <= 0.0) {
    throw std::invalid_argument("rejoin speed limit must be positive");
  }
  validate_nonnegative(config.max_rejoin_lateral_error_m, "maximum rejoin lateral error");
  validate_nonnegative(config.max_rejoin_heading_error_rad, "maximum rejoin heading error");
  validate_nonnegative(config.rejoin_confirm_sec, "rejoin confirmation duration");
  validate_nonnegative(config.rejoin_timeout_sec, "rejoin timeout");
  validate_nonnegative(
    config.rejoin_solver_recovery_timeout_sec,
    "rejoin solver recovery timeout");
  validate_nonnegative(config.cooldown_sec, "recovery cooldown");
}

bool aggressive_retry_reason_is_recoverable(const RecoveryReason reason) noexcept
{
  switch (reason) {
    case RecoveryReason::ClearanceWaitTimedOut:
    case RecoveryReason::ManeuverDirectionUnknown:
    case RecoveryReason::AttemptLimitReached:
    case RecoveryReason::GearReportMissing:
    case RecoveryReason::GearReportTimedOut:
    case RecoveryReason::GearCommandLimitReached:
    case RecoveryReason::ReverseDistanceLimit:
    case RecoveryReason::ReverseDurationLimit:
    case RecoveryReason::ReverseSpeedLimit:
    case RecoveryReason::CollisionWorsening:
    case RecoveryReason::RearHazardAppeared:
    case RecoveryReason::ReverseGearLost:
    case RecoveryReason::ForwardDistanceLimit:
    case RecoveryReason::ForwardDurationLimit:
    case RecoveryReason::ForwardSpeedLimit:
    case RecoveryReason::ForwardHazardAppeared:
    case RecoveryReason::EscapeStepLimitReached:
    case RecoveryReason::ContactNotImproving:
    case RecoveryReason::DriveGearLost:
    case RecoveryReason::EscapeNotConfirmed:
    case RecoveryReason::RejoinTimedOut:
    case RecoveryReason::RejoinUnsafe:
    case RecoveryReason::RejoinPathBlocked:
    case RecoveryReason::SolverUnsafe:
      return true;
    case RecoveryReason::None:
    case RecoveryReason::Disabled:
    case RecoveryReason::ShadowMode:
    case RecoveryReason::SimulationOnlyBlocked:
    case RecoveryReason::RaceInactive:
    case RecoveryReason::AwaitingStuckConfirmation:
    case RecoveryReason::StuckSuspected:
    case RecoveryReason::StuckConfirmed:
    case RecoveryReason::AwsimRecoveryWaiting:
    case RecoveryReason::AwsimRecoveryResolved:
    case RecoveryReason::StopConfirmationPending:
    case RecoveryReason::ClearanceCheck:
    case RecoveryReason::RearStaticBlocked:
    case RecoveryReason::RearVehicleBlocked:
    case RecoveryReason::RearInformationIncomplete:
    case RecoveryReason::ReverseGearRequested:
    case RecoveryReason::ReverseGearConfirmed:
    case RecoveryReason::GearReportInvalid:
    case RecoveryReason::ReverseInProgress:
    case RecoveryReason::ReverseEscapeBraking:
    case RecoveryReason::ReverseEscapeConfirmed:
    case RecoveryReason::ForwardInProgress:
    case RecoveryReason::ForwardEscapeConfirmed:
    case RecoveryReason::EscapeStepComplete:
    case RecoveryReason::DriveGearRequested:
    case RecoveryReason::DriveGearConfirmed:
    case RecoveryReason::RejoinInProgress:
    case RecoveryReason::RejoinComplete:
    case RecoveryReason::SolverRecoveryPending:
    case RecoveryReason::AggressiveRetry:
    case RecoveryReason::CooldownActive:
    case RecoveryReason::OdometryUnsafe:
    case RecoveryReason::ControlInterrupted:
    case RecoveryReason::InvalidInput:
    case RecoveryReason::NonMonotonicTime:
    case RecoveryReason::SessionReset:
      return false;
  }
  return false;
}

bool finite_nonnegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

bool is_shadow_candidate(const StuckVerdict verdict) noexcept
{
  return verdict == StuckVerdict::Suspected || verdict == StuckVerdict::Confirmed;
}

DetectorDecision disabled_decision(const DetectorInput & input) noexcept
{
  DetectorDecision decision;
  decision.verdict = StuckVerdict::NotEligible;
  decision.reject_reason = StuckRejectReason::FeatureDisabled;
  decision.pose_displacement_m =
    std::isfinite(input.pose_displacement_m) ? input.pose_displacement_m : 0.0;
  decision.progress_delta_m =
    std::isfinite(input.unwrapped_progress_delta_m) ?
    input.unwrapped_progress_delta_m : 0.0;
  return decision;
}

}  // namespace

bool should_override_deliberate_stop_for_collision(
  const CollisionDeliberateStopOverrideRequest & request) noexcept
{
  return request.enabled && request.simulation_environment && request.collision_hint &&
         request.has_front_vehicle && request.forward_intent &&
         std::isfinite(request.signed_speed_mps) &&
         std::isfinite(request.stopped_speed_mps) && request.stopped_speed_mps >= 0.0 &&
         std::abs(request.signed_speed_mps) <= request.stopped_speed_mps;
}

bool recovery_escape_distance_confirmed(
  const bool current_footprint_clear, const double traveled_distance_m,
  const double target_distance_m, const double tolerance_m) noexcept
{
  return current_footprint_clear && std::isfinite(traveled_distance_m) &&
         std::isfinite(target_distance_m) && std::isfinite(tolerance_m) &&
         traveled_distance_m >= 0.0 && target_distance_m >= 0.0 && tolerance_m >= 0.0 &&
         traveled_distance_m + tolerance_m + kDistanceComparisonEpsilon >= target_distance_m;
}

bool solver_fallback_requires_reverse_only(
  const bool solver_fallback, const bool evidence_free_recovery_enabled,
  const bool wall_evidence, const double heading_error_rad,
  const double reverse_only_heading_error_rad) noexcept
{
  if (!solver_fallback) {
    return false;
  }
  const bool large_heading_error =
    std::isfinite(heading_error_rad) &&
    std::isfinite(reverse_only_heading_error_rad) &&
    reverse_only_heading_error_rad > 0.0 &&
    std::abs(heading_error_rad) >= reverse_only_heading_error_rad;
  return large_heading_error ||
         (evidence_free_recovery_enabled && !wall_evidence);
}

bool source_timestamp_is_monotonic(
  const double stamp_sec, const std::optional<double> & previous_stamp_sec) noexcept
{
  return std::isfinite(stamp_sec) && stamp_sec > 0.0 &&
         (!previous_stamp_sec.has_value() ||
         stamp_sec + kTimestampEpsilon >= previous_stamp_sec.value());
}

bool source_sample_is_current(
  const double array_stamp_sec, const double sample_stamp_sec,
  const double timeout_sec) noexcept
{
  if (
    !std::isfinite(array_stamp_sec) || array_stamp_sec <= 0.0 ||
    !std::isfinite(sample_stamp_sec) || sample_stamp_sec <= 0.0 ||
    !finite_nonnegative(timeout_sec))
  {
    return false;
  }
  const double source_age_sec = array_stamp_sec - sample_stamp_sec;
  return source_age_sec >= -kTimestampEpsilon &&
         source_age_sec <= timeout_sec + kTimestampEpsilon;
}

FaultRetryGate::FaultRetryGate(FaultRetryConfig config)
: config_(std::move(config))
{
  validate_nonnegative(config_.clear_confirm_sec, "fault retry clear confirmation");
  validate_nonnegative(config_.max_observation_gap_sec, "fault retry maximum observation gap");
  if (config_.max_observation_gap_sec <= 0.0) {
    throw std::invalid_argument("fault retry maximum observation gap must be positive");
  }
}

bool FaultRetryGate::update(const FaultRetryInput & input)
{
  const bool healthy = config_.enabled && input.simulation_environment &&
    input.race_started && input.control_enabled && input.odometry_fresh_and_finite &&
    input.command_finite && input.drive_gear_fresh && input.boost_inactive &&
    input.v2x_complete && input.bounded_maneuver_available && !input.collision_worsening;
  if (!std::isfinite(input.now_sec) || !healthy) {
    reset();
    return false;
  }
  if (
    last_update_sec_.has_value() &&
    (input.now_sec < last_update_sec_.value() ||
    input.now_sec - last_update_sec_.value() > config_.max_observation_gap_sec))
  {
    healthy_since_sec_ = input.now_sec;
  }
  if (!healthy_since_sec_.has_value()) {
    healthy_since_sec_ = input.now_sec;
  }
  last_update_sec_ = input.now_sec;
  return input.now_sec - healthy_since_sec_.value() >= config_.clear_confirm_sec;
}

void FaultRetryGate::reset() noexcept
{
  healthy_since_sec_.reset();
  last_update_sec_.reset();
}

std::optional<double> compute_rejoin_steering_tire_angle(
  const RejoinSteeringRequest & request) noexcept
{
  if (
    !std::isfinite(request.path_curvature_radpm) ||
    !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
    !std::isfinite(request.lateral_error_m) ||
    !std::isfinite(request.heading_error_rad) ||
    !finite_nonnegative(request.lateral_error_gain_rad_per_m) ||
    !finite_nonnegative(request.heading_error_gain) ||
    !std::isfinite(request.max_steering_tire_angle_rad) ||
    request.max_steering_tire_angle_rad <= 0.0 ||
    request.max_steering_tire_angle_rad >= 0.5 * std::acos(-1.0))
  {
    return std::nullopt;
  }

  const double feedforward_rad = std::atan(
    request.wheelbase_m * request.path_curvature_radpm);
  const double target_rad =
    feedforward_rad -
    request.lateral_error_gain_rad_per_m * request.lateral_error_m -
    request.heading_error_gain * request.heading_error_rad;
  if (!std::isfinite(target_rad)) {
    return std::nullopt;
  }
  return std::clamp(
    target_rad,
    -request.max_steering_tire_angle_rad,
    request.max_steering_tire_angle_rad);
}

StuckDetector::StuckDetector(DetectorConfig config)
: config_(std::move(config))
{
  validate_detector_config(config_);
}

bool reverse_actuation_calibration_is_valid(
  const ReverseActuationCalibration & calibration, const double acceleration_min_mps2,
  const double acceleration_max_mps2) noexcept
{
  return std::isfinite(acceleration_min_mps2) && acceleration_min_mps2 < 0.0 &&
         std::isfinite(acceleration_max_mps2) && acceleration_max_mps2 >= 0.0 &&
         std::isfinite(calibration.drive_acceleration_mps2) &&
         std::abs(calibration.drive_acceleration_mps2) > 0.0 &&
         calibration.drive_acceleration_mps2 >= acceleration_min_mps2 &&
         calibration.drive_acceleration_mps2 <= acceleration_max_mps2 &&
         std::isfinite(calibration.stop_acceleration_mps2) &&
         std::abs(calibration.stop_acceleration_mps2) > 0.0 &&
         calibration.stop_acceleration_mps2 >= acceleration_min_mps2 &&
         calibration.stop_acceleration_mps2 <= acceleration_max_mps2 &&
         calibration.drive_acceleration_mps2 * calibration.stop_acceleration_mps2 < 0.0 &&
         std::isfinite(calibration.verified_stop_deceleration_mps2) &&
         calibration.verified_stop_deceleration_mps2 > 0.0 &&
         std::isfinite(calibration.control_latency_sec) && calibration.control_latency_sec >= 0.0;
}

double reverse_stopping_distance_reserve_m(
  const ReverseActuationCalibration & calibration, const double signed_speed_mps,
  const double control_period_sec) noexcept
{
  if (
    !std::isfinite(signed_speed_mps) || !std::isfinite(control_period_sec) ||
    control_period_sec < 0.0 ||
    !std::isfinite(calibration.verified_stop_deceleration_mps2) ||
    calibration.verified_stop_deceleration_mps2 <= 0.0 ||
    !std::isfinite(calibration.control_latency_sec) || calibration.control_latency_sec < 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double speed_mps = std::abs(signed_speed_mps);
  return
    speed_mps * speed_mps / (2.0 * calibration.verified_stop_deceleration_mps2) +
    speed_mps * (calibration.control_latency_sec + control_period_sec);
}

DetectorDecision StuckDetector::update(const DetectorInput & input)
{
  const bool numeric_input_valid =
    std::isfinite(input.now_sec) && std::isfinite(input.signed_speed_mps) &&
    std::isfinite(input.requested_forward_speed_mps) &&
    std::isfinite(input.requested_acceleration_mps2) &&
    finite_nonnegative(input.pose_displacement_m) &&
    std::isfinite(input.unwrapped_progress_delta_m);
  if (!numeric_input_valid) {
    reset_observation();
    return reject(input, StuckVerdict::NotEligible, StuckRejectReason::InvalidInput);
  }

  if (last_update_sec_.has_value() && input.now_sec < *last_update_sec_) {
    reset_observation();
    last_update_sec_ = input.now_sec;
    return reject(input, StuckVerdict::NotEligible, StuckRejectReason::NonMonotonicTime);
  }
  if (last_update_sec_.has_value() &&
    input.now_sec - *last_update_sec_ > config_.max_observation_gap_sec)
  {
    // Do not count executor/odometry outages as continuous stopped or fallback
    // observation.
    reset_observation();
  }
  last_update_sec_ = input.now_sec;

  const bool forward_intent =
    input.requested_forward_speed_mps >= config_.forward_intent_speed_mps ||
    input.requested_acceleration_mps2 >=
    config_.forward_intent_acceleration_mps2;
  // A solver-fallback recovery normally requires current footprint/wall
  // evidence. The separately configured evidence-free route has a longer
  // no-motion window and is constrained to reverse-only actuation by the ROS
  // adapter. A legacy collision hint alone remains intentionally insufficient.
  const bool evidence = input.solver_fallback ?
    input.wall_evidence :
    (input.wall_evidence || input.collision_hint);

  const auto reject_and_reset =
    [this, &input, forward_intent, evidence](
    const StuckVerdict verdict, const StuckRejectReason reason)
    {
      reset_observation();
      return reject(input, verdict, reason, forward_intent, evidence);
    };

  if (!input.race_started) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::RaceNotStarted);
  }
  if (!input.control_enabled) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::ControlDisabled);
  }
  if (!input.odometry_fresh) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::OdometryStale);
  }
  const bool coordinated_stop_candidate =
    input.deliberate_stop && input.coordinated_stop &&
    config_.coordinated_stop_recovery_enabled;
  const bool solver_evidence_free_candidate =
    input.solver_fallback && !input.wall_evidence &&
    config_.solver_evidence_free_recovery_enabled;
  if (input.deliberate_stop && !coordinated_stop_candidate) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::DeliberateStop);
  }
  if (input.solver_fallback && !config_.solver_fallback_recovery_enabled) {
    return reject_and_reset(
      StuckVerdict::NotEligible,
      StuckRejectReason::SolverFallback);
  }
  if (input.solver_fallback && !input.wall_evidence &&
    !solver_evidence_free_candidate && !coordinated_stop_candidate)
  {
    return reject_and_reset(
      StuckVerdict::NotEligible,
      StuckRejectReason::SolverFallbackMissingWallEvidence);
  }
  if (input.gear_transition_active) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::GearTransition);
  }
  if (input.awsim_recovery_settling) {
    return reject_and_reset(
      StuckVerdict::NotEligible, StuckRejectReason::AwsimRecoverySettling);
  }
  if (!forward_intent) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::NoForwardIntent);
  }

  if (input.solver_fallback) {
    if (!solver_fallback_since_sec_.has_value()) {
      solver_fallback_since_sec_ = input.now_sec;
    }
  } else {
    solver_fallback_since_sec_.reset();
  }

  const double absolute_speed_mps = std::abs(input.signed_speed_mps);
  const double speed_limit_mps = stationary_since_sec_.has_value() ?
    config_.moving_speed_mps : config_.stopped_speed_mps;
  if (absolute_speed_mps > speed_limit_mps) {
    return reject_and_reset(StuckVerdict::Moving, StuckRejectReason::VehicleMoving);
  }
  if (input.pose_displacement_m > config_.max_pose_displacement_m) {
    return reject_and_reset(StuckVerdict::Moving, StuckRejectReason::PoseProgressing);
  }
  if (std::abs(input.unwrapped_progress_delta_m) > config_.max_progress_delta_m) {
    return reject_and_reset(StuckVerdict::Moving, StuckRejectReason::PathProgressing);
  }

  if (!stationary_since_sec_.has_value()) {
    stationary_since_sec_ = input.now_sec;
  }
  const double stationary_duration_sec = input.now_sec - *stationary_since_sec_;
  const double solver_fallback_duration_sec =
    solver_fallback_since_sec_.has_value() ?
    input.now_sec - *solver_fallback_since_sec_ :
    0.0;
  double required_stationary_duration_sec = config_.stationary_duration_sec;
  if (coordinated_stop_candidate) {
    required_stationary_duration_sec = std::max(
      required_stationary_duration_sec, config_.coordinated_stop_duration_sec);
  }
  if (solver_evidence_free_candidate) {
    required_stationary_duration_sec = std::max(
      required_stationary_duration_sec, config_.solver_evidence_free_duration_sec);
  }
  if (stationary_duration_sec < required_stationary_duration_sec ||
    (input.solver_fallback &&
    solver_fallback_duration_sec < config_.solver_fallback_duration_sec))
  {
    return reject(
      input, StuckVerdict::Moving,
      StuckRejectReason::ObservationWindowIncomplete,
      forward_intent, evidence);
  }
  if (!evidence) {
    if (coordinated_stop_candidate) {
      return reject(
        input, StuckVerdict::Confirmed, StuckRejectReason::None,
        forward_intent, false);
    }
    if (solver_evidence_free_candidate) {
      return reject(
        input, StuckVerdict::Confirmed, StuckRejectReason::None,
        forward_intent, false);
    }
    if (
      config_.evidence_free_recovery_enabled && !input.solver_fallback &&
      stationary_duration_sec >= config_.evidence_free_duration_sec)
    {
      return reject(
        input, StuckVerdict::Confirmed, StuckRejectReason::None,
        forward_intent, false);
    }
    return reject(
      input, StuckVerdict::Suspected, StuckRejectReason::MissingCorroboratingEvidence,
      forward_intent, false);
  }
  return reject(
    input, StuckVerdict::Confirmed, StuckRejectReason::None, forward_intent, true);
}

void StuckDetector::reset() noexcept
{
  stationary_since_sec_.reset();
  solver_fallback_since_sec_.reset();
  last_update_sec_.reset();
}

const DetectorConfig & StuckDetector::config() const noexcept
{
  return config_;
}

DetectorDecision StuckDetector::reject(
  const DetectorInput & input, const StuckVerdict verdict, const StuckRejectReason reason,
  const bool forward_intent, const bool evidence) noexcept
{
  DetectorDecision decision;
  decision.verdict = verdict;
  decision.reject_reason = reason;
  if (
    stationary_since_sec_.has_value() && std::isfinite(input.now_sec) &&
    input.now_sec >= *stationary_since_sec_)
  {
    decision.stationary_duration_sec = input.now_sec - *stationary_since_sec_;
  }
  decision.pose_displacement_m = std::isfinite(input.pose_displacement_m) ?
    input.pose_displacement_m :
    0.0;
  decision.progress_delta_m = std::isfinite(input.unwrapped_progress_delta_m) ?
    input.unwrapped_progress_delta_m :
    0.0;
  if (solver_fallback_since_sec_.has_value() && std::isfinite(input.now_sec) &&
    input.now_sec >= *solver_fallback_since_sec_)
  {
    decision.solver_fallback_duration_sec =
      input.now_sec - *solver_fallback_since_sec_;
  }
  decision.forward_intent = forward_intent;
  decision.corroborating_evidence = evidence;
  decision.solver_fallback_qualified =
    input.solver_fallback && config_.solver_fallback_recovery_enabled &&
    verdict == StuckVerdict::Confirmed && forward_intent && evidence;
  decision.solver_evidence_free_qualified =
    input.solver_fallback && config_.solver_fallback_recovery_enabled &&
    config_.solver_evidence_free_recovery_enabled &&
    verdict == StuckVerdict::Confirmed && forward_intent && !evidence &&
    decision.stationary_duration_sec >= config_.solver_evidence_free_duration_sec &&
    decision.solver_fallback_duration_sec >= config_.solver_fallback_duration_sec;
  decision.evidence_free_qualified =
    !input.solver_fallback && !input.deliberate_stop &&
    config_.evidence_free_recovery_enabled &&
    verdict == StuckVerdict::Confirmed && forward_intent && !evidence &&
    decision.stationary_duration_sec >= config_.evidence_free_duration_sec;
  decision.coordinated_stop_qualified =
    input.deliberate_stop && input.coordinated_stop &&
    config_.coordinated_stop_recovery_enabled && verdict == StuckVerdict::Confirmed &&
    forward_intent && !evidence &&
    decision.stationary_duration_sec >= config_.coordinated_stop_duration_sec;
  return decision;
}

void StuckDetector::reset_observation() noexcept
{
  stationary_since_sec_.reset();
  solver_fallback_since_sec_.reset();
}

RecoverySupervisor::RecoverySupervisor(SupervisorConfig config)
: config_(std::move(config))
{
  validate_supervisor_config(config_);
}

RecoveryAction RecoverySupervisor::update(const RecoveryInput & input)
{
  if (!input_is_finite(input)) {
    const double transition_time = std::isfinite(input.now_sec) ? input.now_sec : 0.0;
    transition(RecoveryState::SafeStop, RecoveryReason::InvalidInput, transition_time);
    return safe_stop_action(RecoveryReason::InvalidInput);
  }
  if (last_update_sec_.has_value() && input.now_sec < *last_update_sec_) {
    transition(RecoveryState::SafeStop, RecoveryReason::NonMonotonicTime, input.now_sec);
    last_update_sec_ = input.now_sec;
    return safe_stop_action(RecoveryReason::NonMonotonicTime);
  }
  if (!state_entered_sec_.has_value()) {
    state_entered_sec_ = input.now_sec;
  }
  last_update_sec_ = input.now_sec;

  if (!input.race_active) {
    reset_session();
    last_update_sec_ = input.now_sec;
    state_entered_sec_ = input.now_sec;
    state_reason_ = RecoveryReason::RaceInactive;
    return normal_action(RecoveryReason::RaceInactive);
  }
  if (!input.odometry_valid) {
    if (state_ == RecoveryState::Normal || state_ == RecoveryState::SuspectStuck) {
      transition(RecoveryState::Normal, RecoveryReason::OdometryUnsafe, input.now_sec);
      return normal_action(RecoveryReason::OdometryUnsafe);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::OdometryUnsafe, input.now_sec);
    return safe_stop_action(RecoveryReason::OdometryUnsafe);
  }
  const bool aggressive_solver_independent_rejoin =
    config_.aggressive_sim_recovery_enabled && state_ == RecoveryState::LowSpeedRejoin;
  if (!input.solver_healthy && !aggressive_solver_independent_rejoin) {
    if (
      state_ == RecoveryState::LowSpeedRejoin &&
      config_.rejoin_solver_recovery_timeout_sec > 0.0)
    {
      if (!solver_unhealthy_since_sec_.has_value()) {
        solver_unhealthy_since_sec_ = input.now_sec;
      }
      aligned_since_sec_.reset();
      if (
        input.now_sec - *solver_unhealthy_since_sec_ <
        config_.rejoin_solver_recovery_timeout_sec)
      {
        transition(
          RecoveryState::LowSpeedRejoin,
          RecoveryReason::SolverRecoveryPending, input.now_sec);
        return hold_action(RecoveryReason::SolverRecoveryPending);
      }
    }
    if (state_ == RecoveryState::Normal || state_ == RecoveryState::SuspectStuck) {
      transition(RecoveryState::Normal, RecoveryReason::SolverUnsafe, input.now_sec);
      return normal_action(RecoveryReason::SolverUnsafe);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::SolverUnsafe, input.now_sec);
    return safe_stop_action(RecoveryReason::SolverUnsafe);
  }
  solver_unhealthy_since_sec_.reset();
  if (!input.control_enabled) {
    if (state_ == RecoveryState::Normal || state_ == RecoveryState::SuspectStuck) {
      transition(RecoveryState::Normal, RecoveryReason::ControlInterrupted, input.now_sec);
      return normal_action(RecoveryReason::ControlInterrupted);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::ControlInterrupted, input.now_sec);
    return safe_stop_action(RecoveryReason::ControlInterrupted);
  }
  if (input.hard_stop_requested) {
    if (state_ == RecoveryState::Normal || state_ == RecoveryState::SuspectStuck) {
      transition(RecoveryState::Normal, RecoveryReason::ControlInterrupted, input.now_sec);
      return normal_action(RecoveryReason::ControlInterrupted);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::ControlInterrupted, input.now_sec);
    return safe_stop_action(RecoveryReason::ControlInterrupted);
  }

  switch (state_) {
    case RecoveryState::Normal:
      return update_normal(input);
    case RecoveryState::SuspectStuck:
      return update_suspect(input);
    case RecoveryState::WaitAwsimRecovery:
      return update_wait_awsim(input);
    case RecoveryState::StopAndConfirm:
      return update_stop_and_confirm(input);
    case RecoveryState::CheckClearance:
      return update_check_clearance(input);
    case RecoveryState::WaitForClear:
      return update_wait_for_clear(input);
    case RecoveryState::ShiftToReverse:
      transition(
        RecoveryState::WaitReverseReport, RecoveryReason::ReverseGearRequested,
        input.now_sec);
      return update_wait_reverse_report(input);
    case RecoveryState::WaitReverseReport:
      return update_wait_reverse_report(input);
    case RecoveryState::ReverseManeuver:
      return update_reverse_maneuver(input);
    case RecoveryState::ForwardManeuver:
      return update_forward_maneuver(input);
    case RecoveryState::StopAndReassess:
      return update_stop_and_reassess(input);
    case RecoveryState::StopBeforeDrive:
      return update_stop_before_drive(input);
    case RecoveryState::ShiftToDrive:
      transition(
        RecoveryState::WaitDriveReport, RecoveryReason::DriveGearRequested,
        input.now_sec);
      return update_wait_drive_report(input);
    case RecoveryState::WaitDriveReport:
      return update_wait_drive_report(input);
    case RecoveryState::LowSpeedRejoin:
      return update_low_speed_rejoin(input);
    case RecoveryState::SafeStop:
      return update_safe_stop(input);
  }
  transition(RecoveryState::SafeStop, RecoveryReason::InvalidInput, input.now_sec);
  return safe_stop_action(RecoveryReason::InvalidInput);
}

void RecoverySupervisor::reset_session() noexcept
{
  state_ = RecoveryState::Normal;
  state_reason_ = RecoveryReason::SessionReset;
  attempt_count_ = 0U;
  escape_step_count_ = 0U;
  gear_command_request_count_ = 0U;
  state_entered_sec_.reset();
  last_update_sec_.reset();
  stopped_since_sec_.reset();
  aligned_since_sec_.reset();
  solver_unhealthy_since_sec_.reset();
  clearance_safe_since_sec_.reset();
  last_gear_request_sec_.reset();
  cooldown_until_sec_.reset();
  normal_reset_pending_ = false;
  active_stepwise_escape_ = false;
  reassess_after_drive_ = false;
  safe_stop_after_drive_ = false;
  escape_confirmed_before_drive_ = false;
}

RecoveryState RecoverySupervisor::state() const noexcept
{
  return state_;
}

RecoveryReason RecoverySupervisor::state_reason() const noexcept
{
  return state_reason_;
}

std::size_t RecoverySupervisor::attempt_count() const noexcept
{
  return attempt_count_;
}

std::size_t RecoverySupervisor::escape_step_count() const noexcept
{
  return escape_step_count_;
}

bool RecoverySupervisor::safe_stop_latched() const noexcept
{
  return state_ == RecoveryState::SafeStop;
}

bool RecoverySupervisor::drive_report_will_reassess_or_stop() const noexcept
{
  return reassess_after_drive_ || safe_stop_after_drive_;
}

const SupervisorConfig & RecoverySupervisor::config() const noexcept
{
  return config_;
}

RecoveryAction RecoverySupervisor::update_normal(const RecoveryInput & input)
{
  if (cooldown_until_sec_.has_value() && input.now_sec < *cooldown_until_sec_) {
    return normal_action(RecoveryReason::CooldownActive);
  }
  cooldown_until_sec_.reset();

  if (input.detector.verdict == StuckVerdict::Confirmed) {
    // Maneuver and attempt limits bound one recovery episode. A completed
    // rejoin must not consume the budget of a later, independent obstruction.
    attempt_count_ = 0U;
    escape_step_count_ = 0U;
    transition(RecoveryState::SuspectStuck, RecoveryReason::StuckConfirmed, input.now_sec);
    return hold_action(RecoveryReason::StuckConfirmed);
  }
  if (input.detector.verdict == StuckVerdict::Suspected) {
    attempt_count_ = 0U;
    escape_step_count_ = 0U;
    transition(RecoveryState::SuspectStuck, RecoveryReason::StuckSuspected, input.now_sec);
    return normal_action(RecoveryReason::StuckSuspected);
  }
  return normal_action(RecoveryReason::AwaitingStuckConfirmation);
}

RecoveryAction RecoverySupervisor::update_suspect(const RecoveryInput & input)
{
  if (input.detector.verdict == StuckVerdict::Confirmed) {
    transition(
      RecoveryState::WaitAwsimRecovery, RecoveryReason::AwsimRecoveryWaiting,
      input.now_sec);
    return hold_action(RecoveryReason::AwsimRecoveryWaiting);
  }
  if (input.detector.verdict == StuckVerdict::Suspected) {
    return normal_action(RecoveryReason::StuckSuspected);
  }
  transition(RecoveryState::Normal, RecoveryReason::AwaitingStuckConfirmation, input.now_sec);
  return normal_action(RecoveryReason::AwaitingStuckConfirmation);
}

RecoveryAction RecoverySupervisor::update_wait_awsim(const RecoveryInput & input)
{
  if (!input.awsim_recovery_settled) {
    return hold_action(RecoveryReason::AwsimRecoveryWaiting);
  }
  if (state_elapsed(input.now_sec) < config_.awsim_recovery_wait_sec) {
    return hold_action(RecoveryReason::AwsimRecoveryWaiting);
  }
  // AWSIM wall recovery can rotate or nudge a still-colliding vehicle enough
  // to reset the detector's observation window. Only leave Recovery when the
  // current footprint is actually clear; otherwise continue with the bounded
  // maneuver after the AWSIM settling interval.
  if (
    input.detector.verdict != StuckVerdict::Confirmed &&
    input.awsim_recovery_resolved)
  {
    transition(RecoveryState::Normal, RecoveryReason::AwsimRecoveryResolved, input.now_sec);
    cooldown_until_sec_ = input.now_sec + config_.cooldown_sec;
    return normal_action(RecoveryReason::AwsimRecoveryResolved);
  }
  transition(
    RecoveryState::StopAndConfirm, RecoveryReason::StopConfirmationPending,
    input.now_sec);
  return hold_action(RecoveryReason::StopConfirmationPending);
}

RecoveryAction RecoverySupervisor::update_stop_and_confirm(const RecoveryInput & input)
{
  if (!stopped_confirmed(input)) {
    return hold_action(RecoveryReason::StopConfirmationPending);
  }
  transition(RecoveryState::CheckClearance, RecoveryReason::ClearanceCheck, input.now_sec);
  return hold_action(RecoveryReason::ClearanceCheck);
}

RecoveryAction RecoverySupervisor::update_check_clearance(const RecoveryInput & input)
{
  if (input.maneuver_direction == ManeuverDirection::Unknown) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::ManeuverDirectionUnknown, input.now_sec);
    return safe_stop_action(RecoveryReason::ManeuverDirectionUnknown);
  }
  if (!clearance_is_safe(input)) {
    const RecoveryReason reason = clearance_reason(input);
    transition(RecoveryState::WaitForClear, reason, input.now_sec);
    return hold_action(reason);
  }
  if (input.stepwise_escape && escape_step_count_ >= config_.max_escape_steps) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::EscapeStepLimitReached, input.now_sec);
    return safe_stop_action(RecoveryReason::EscapeStepLimitReached);
  }
  if (!input.stepwise_escape && attempt_count_ >= config_.max_attempts) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::AttemptLimitReached, input.now_sec);
    return safe_stop_action(RecoveryReason::AttemptLimitReached);
  }

  if (input.maneuver_direction == ManeuverDirection::Forward) {
    if (
      !input.gear_report_fresh || !gear_report_is_valid(input.reported_gear) ||
      input.reported_gear != Gear::Drive)
    {
      transition(RecoveryState::SafeStop, RecoveryReason::DriveGearLost, input.now_sec);
      return safe_stop_action(RecoveryReason::DriveGearLost);
    }
    if (!input.stepwise_escape || attempt_count_ == 0U) {
      ++attempt_count_;
    }
    if (input.stepwise_escape) {
      ++escape_step_count_;
    }
    active_stepwise_escape_ = input.stepwise_escape;
    transition(
      RecoveryState::ForwardManeuver, RecoveryReason::ForwardInProgress, input.now_sec);
    return update_forward_maneuver(input);
  }
  if (config_.max_gear_command_requests == 0U) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::GearCommandLimitReached, input.now_sec);
    return safe_stop_action(RecoveryReason::GearCommandLimitReached);
  }

  if (!input.stepwise_escape || attempt_count_ == 0U) {
    ++attempt_count_;
  }
  if (input.stepwise_escape) {
    ++escape_step_count_;
  }
  active_stepwise_escape_ = input.stepwise_escape;
  transition(
    RecoveryState::ShiftToReverse, RecoveryReason::ReverseGearRequested, input.now_sec);
  return request_gear_action(Gear::Reverse, RecoveryReason::ReverseGearRequested, input.now_sec);
}

RecoveryAction RecoverySupervisor::update_wait_for_clear(const RecoveryInput & input)
{
  if (clearance_is_safe(input)) {
    transition(RecoveryState::CheckClearance, RecoveryReason::ClearanceCheck, input.now_sec);
    // Consume the same complete clearance snapshot immediately. Waiting for a
    // second control cycle can lose a valid V2X/candidate sample and leave a
    // stopped vehicle oscillating between WAIT_FOR_CLEAR and CHECK_CLEARANCE.
    return update_check_clearance(input);
  }
  if (state_elapsed(input.now_sec) >= config_.clearance_wait_timeout_sec) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::ClearanceWaitTimedOut, input.now_sec);
    return safe_stop_action(RecoveryReason::ClearanceWaitTimedOut);
  }
  return hold_action(clearance_reason(input));
}

RecoveryAction RecoverySupervisor::update_safe_stop(const RecoveryInput & input)
{
  if (
    config_.clearance_safe_stop_recovery_enabled &&
    state_reason_ == RecoveryReason::ClearanceWaitTimedOut)
  {
    if (!clearance_is_safe(input)) {
      clearance_safe_since_sec_.reset();
      // A blocked rear corridor is external state, not a failed maneuver.
      // Stay stopped and re-evaluate the same complete snapshot instead of
      // entering the aggressive retry loop while the blocker is unchanged.
      return safe_stop_action(state_reason_);
    } else {
      if (!clearance_safe_since_sec_.has_value()) {
        clearance_safe_since_sec_ = input.now_sec;
      }
      if (input.now_sec - *clearance_safe_since_sec_ >= config_.safe_stop_clear_confirm_sec) {
        transition(RecoveryState::CheckClearance, RecoveryReason::ClearanceCheck, input.now_sec);
        return hold_action(RecoveryReason::ClearanceCheck);
      }
      return safe_stop_action(state_reason_);
    }
  } else {
    clearance_safe_since_sec_.reset();
  }

  if (
    !config_.aggressive_sim_recovery_enabled ||
    !aggressive_retry_reason_is_recoverable(state_reason_) ||
    state_elapsed(input.now_sec) < config_.aggressive_retry_delay_sec)
  {
    return safe_stop_action(state_reason_);
  }

  // A new simulation-race recovery cycle must not inherit a consumed budget
  // or an escape confirmation from the failed primitive. The ROS adapter also
  // resets its pose/contact/candidate anchors on this transition.
  attempt_count_ = 0U;
  escape_step_count_ = 0U;
  active_stepwise_escape_ = false;
  reassess_after_drive_ = false;
  safe_stop_after_drive_ = false;
  escape_confirmed_before_drive_ = false;
  transition(RecoveryState::StopAndConfirm, RecoveryReason::AggressiveRetry, input.now_sec);
  return hold_action(RecoveryReason::AggressiveRetry);
}

RecoveryAction RecoverySupervisor::update_wait_reverse_report(const RecoveryInput & input)
{
  if (input.gear_report_fresh && !gear_report_is_valid(input.reported_gear)) {
    transition(RecoveryState::SafeStop, RecoveryReason::GearReportInvalid, input.now_sec);
    return safe_stop_action(RecoveryReason::GearReportInvalid);
  }
  if (input.gear_report_fresh && input.reported_gear == Gear::Reverse) {
    if (!clearance_is_safe(input)) {
      // Missing V2X/Boost information is not evidence of a rear hazard and
      // must never complete or abandon a recovery. Remain stopped in Reverse
      // and resume the same bounded maneuver when the complete snapshot
      // returns. A positively observed static/vehicle blockage still returns
      // to Drive after the bounded wait below.
      if (!input.rear_information_complete) {
        return hold_action(RecoveryReason::RearInformationIncomplete);
      }
      if (state_elapsed(input.now_sec) >= config_.clearance_wait_timeout_sec) {
        if (active_stepwise_escape_) {
          // The current primitive is no longer safe, but this bounded
          // stepwise episode can still select a different candidate after
          // returning to Drive. Preserve the episode distance/step budget and
          // reuse the normal stop-and-reassess path.
          reassess_after_drive_ = true;
        }
        transition(
          RecoveryState::StopBeforeDrive, RecoveryReason::RearHazardAppeared,
          input.now_sec);
        return hold_action(RecoveryReason::RearHazardAppeared);
      }
      // Gear changes and V2X callbacks can become visible on adjacent control
      // cycles. Stay stopped in Reverse until a complete safe corridor is
      // observed; never emit ReverseCreep on incomplete information.
      return hold_action(clearance_reason(input));
    }
    transition(
      RecoveryState::ReverseManeuver, RecoveryReason::ReverseGearConfirmed,
      input.now_sec);
    return update_reverse_maneuver(input);
  }
  if (state_elapsed(input.now_sec) >= config_.gear_report_timeout_sec) {
    transition(RecoveryState::SafeStop, RecoveryReason::GearReportTimedOut, input.now_sec);
    return safe_stop_action(RecoveryReason::GearReportTimedOut);
  }
  if (
    gear_command_request_count_ < config_.max_gear_command_requests &&
    last_gear_request_sec_.has_value() &&
    input.now_sec - *last_gear_request_sec_ >= config_.gear_command_resend_interval_sec)
  {
    return request_gear_action(
      Gear::Reverse, RecoveryReason::ReverseGearRequested, input.now_sec);
  }
  return hold_action(
    input.gear_report_fresh ? RecoveryReason::ReverseGearRequested :
    RecoveryReason::GearReportMissing);
}

RecoveryAction RecoverySupervisor::update_reverse_maneuver(const RecoveryInput & input)
{
  if (
    !input.gear_report_fresh || !gear_report_is_valid(input.reported_gear) ||
    input.reported_gear != Gear::Reverse)
  {
    transition(RecoveryState::StopBeforeDrive, RecoveryReason::ReverseGearLost, input.now_sec);
    return hold_action(RecoveryReason::ReverseGearLost);
  }
  if (input.collision_worsening) {
    // A stepwise contact escape owns only one short bounded motion. Stop that
    // motion immediately, return to Drive, and select a new statically checked
    // primitive instead of abandoning the whole episode after one map-cell
    // boundary change. The escape-step limit still bounds repeated retries.
    if (active_stepwise_escape_) {
      reassess_after_drive_ = true;
    }
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::CollisionWorsening,
      input.now_sec);
    return hold_action(RecoveryReason::CollisionWorsening);
  }
  if (config_.max_reverse_speed_mps <= 0.0) {
    transition(
      RecoveryState::StopBeforeDrive,
      RecoveryReason::ReverseSpeedLimit, input.now_sec);
    return hold_action(RecoveryReason::ReverseSpeedLimit);
  }
  if (
    active_stepwise_escape_ &&
    input.traveled_distance_m >= config_.escape_step_distance_m)
  {
    if (!input.step_contact_improved) {
      safe_stop_after_drive_ = true;
      transition(
        RecoveryState::StopBeforeDrive, RecoveryReason::ContactNotImproving,
        input.now_sec);
      return hold_action(RecoveryReason::ContactNotImproving);
    }
    reassess_after_drive_ = true;
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::EscapeStepComplete,
      input.now_sec);
    return hold_action(RecoveryReason::EscapeStepComplete);
  }
  if (!clearance_is_safe(input)) {
    // Static occupancy and V2X callbacks can briefly become inconsistent with
    // the control cycle while Reverse remains engaged. Stop immediately, but
    // keep Reverse and wait for a complete safe corridor instead of abandoning
    // an otherwise improving escape step after one incomplete sample.
    transition(
      RecoveryState::WaitReverseReport, clearance_reason(input), input.now_sec);
    return hold_action(clearance_reason(input));
  }
  if (!active_stepwise_escape_ && input.recovery_escape_confirmed) {
    escape_confirmed_before_drive_ = true;
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::ReverseEscapeConfirmed,
      input.now_sec);
    return hold_action(RecoveryReason::ReverseEscapeConfirmed);
  }
  if (input.episode_traveled_distance_m >= config_.max_reverse_distance_m) {
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::ReverseDistanceLimit,
      input.now_sec);
    return hold_action(RecoveryReason::ReverseDistanceLimit);
  }
  if (state_elapsed(input.now_sec) >= config_.max_reverse_duration_sec) {
    // A stepwise escape may hit its per-maneuver time budget just before the
    // short distance target when acceleration is intentionally small.  Treat
    // that boundary like a completed bounded step: stop, return to Drive, and
    // run the full static/V2X clearance selection again.  The episode distance
    // and max_escape_steps limits still bound the total recovery motion.
    if (active_stepwise_escape_ || attempt_count_ < config_.max_attempts) {
      reassess_after_drive_ = true;
    }
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::ReverseDurationLimit,
      input.now_sec);
    return hold_action(RecoveryReason::ReverseDurationLimit);
  }
  if (!active_stepwise_escape_ && input.reverse_escape_brake_required) {
    // Do not leave Reverse merely to regulate stopping distance. The adapter
    // applies the calibrated Reverse stop command; if the vehicle stops short,
    // this state can resume ReverseCreep without another Drive/Reverse cycle.
    return hold_action(RecoveryReason::ReverseEscapeBraking);
  }
  if (std::abs(input.signed_speed_mps) >= config_.max_reverse_speed_mps) {
    // The speed ceiling is a regulator, not a completed escape. Brake in
    // Reverse and resume the same maneuver once speed is below the ceiling;
    // otherwise a constant-acceleration creep can terminate after only a
    // fraction of the required escape distance. Distance, duration and
    // clearance limits above remain authoritative while speed is regulated.
    return hold_action(RecoveryReason::ReverseSpeedLimit);
  }
  return reverse_action(
    RecoveryReason::ReverseInProgress, input.reverse_steering_tire_angle_rad);
}

RecoveryAction RecoverySupervisor::update_forward_maneuver(const RecoveryInput & input)
{
  if (
    !input.gear_report_fresh || !gear_report_is_valid(input.reported_gear) ||
    input.reported_gear != Gear::Drive)
  {
    transition(RecoveryState::SafeStop, RecoveryReason::DriveGearLost, input.now_sec);
    return safe_stop_action(RecoveryReason::DriveGearLost);
  }
  if (input.collision_worsening) {
    if (active_stepwise_escape_) {
      transition(
        RecoveryState::StopAndReassess, RecoveryReason::CollisionWorsening,
        input.now_sec);
      return hold_action(RecoveryReason::CollisionWorsening);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::CollisionWorsening, input.now_sec);
    return safe_stop_action(RecoveryReason::CollisionWorsening);
  }
  if (config_.max_forward_speed_mps <= 0.0 ||
    std::abs(input.signed_speed_mps) >= config_.max_forward_speed_mps)
  {
    transition(RecoveryState::SafeStop, RecoveryReason::ForwardSpeedLimit, input.now_sec);
    return safe_stop_action(RecoveryReason::ForwardSpeedLimit);
  }
  if (
    active_stepwise_escape_ &&
    input.traveled_distance_m >= config_.escape_step_distance_m)
  {
    if (!input.step_contact_improved) {
      transition(
        RecoveryState::SafeStop, RecoveryReason::ContactNotImproving,
        input.now_sec);
      return safe_stop_action(RecoveryReason::ContactNotImproving);
    }
    transition(
      RecoveryState::StopAndReassess, RecoveryReason::EscapeStepComplete,
      input.now_sec);
    return hold_action(RecoveryReason::EscapeStepComplete);
  }
  if (!clearance_is_safe(input)) {
    transition(
      RecoveryState::StopAndReassess, RecoveryReason::ForwardHazardAppeared,
      input.now_sec);
    return hold_action(RecoveryReason::ForwardHazardAppeared);
  }
  if (input.recovery_escape_confirmed) {
    escape_confirmed_before_drive_ = true;
    transition(
      RecoveryState::LowSpeedRejoin, RecoveryReason::ForwardEscapeConfirmed,
      input.now_sec);
    return rejoin_action(
      RecoveryReason::ForwardEscapeConfirmed,
      input.rejoin_steering_tire_angle_rad);
  }
  if (input.traveled_distance_m >= config_.max_forward_distance_m) {
    transition(RecoveryState::SafeStop, RecoveryReason::ForwardDistanceLimit, input.now_sec);
    return safe_stop_action(RecoveryReason::ForwardDistanceLimit);
  }
  if (state_elapsed(input.now_sec) >= config_.max_forward_duration_sec) {
    if (active_stepwise_escape_) {
      transition(
        RecoveryState::StopAndReassess, RecoveryReason::ForwardDurationLimit,
        input.now_sec);
      return hold_action(RecoveryReason::ForwardDurationLimit);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::ForwardDurationLimit, input.now_sec);
    return safe_stop_action(RecoveryReason::ForwardDurationLimit);
  }
  return forward_action(
    RecoveryReason::ForwardInProgress, input.reverse_steering_tire_angle_rad);
}

RecoveryAction RecoverySupervisor::update_stop_and_reassess(const RecoveryInput & input)
{
  if (!stopped_confirmed(input)) {
    return hold_action(RecoveryReason::StopConfirmationPending);
  }
  if (input.recovery_escape_confirmed) {
    active_stepwise_escape_ = false;
    escape_confirmed_before_drive_ = true;
    transition(
      RecoveryState::LowSpeedRejoin, RecoveryReason::ForwardEscapeConfirmed,
      input.now_sec);
    return rejoin_action(
      RecoveryReason::ForwardEscapeConfirmed,
      input.rejoin_steering_tire_angle_rad);
  }
  if (escape_step_count_ >= config_.max_escape_steps) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::EscapeStepLimitReached,
      input.now_sec);
    return safe_stop_action(RecoveryReason::EscapeStepLimitReached);
  }
  active_stepwise_escape_ = false;
  transition(RecoveryState::CheckClearance, RecoveryReason::ClearanceCheck, input.now_sec);
  return hold_action(RecoveryReason::ClearanceCheck);
}

RecoveryAction RecoverySupervisor::update_stop_before_drive(const RecoveryInput & input)
{
  if (!stopped_confirmed(input)) {
    return hold_action(RecoveryReason::StopConfirmationPending);
  }
  if (config_.max_gear_command_requests == 0U) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::GearCommandLimitReached, input.now_sec);
    return safe_stop_action(RecoveryReason::GearCommandLimitReached);
  }
  transition(RecoveryState::ShiftToDrive, RecoveryReason::DriveGearRequested, input.now_sec);
  return request_gear_action(Gear::Drive, RecoveryReason::DriveGearRequested, input.now_sec);
}

RecoveryAction RecoverySupervisor::update_wait_drive_report(const RecoveryInput & input)
{
  if (input.gear_report_fresh && !gear_report_is_valid(input.reported_gear)) {
    transition(RecoveryState::SafeStop, RecoveryReason::GearReportInvalid, input.now_sec);
    return safe_stop_action(RecoveryReason::GearReportInvalid);
  }
  if (input.gear_report_fresh && input.reported_gear == Gear::Drive) {
    if (safe_stop_after_drive_) {
      safe_stop_after_drive_ = false;
      transition(
        RecoveryState::SafeStop, RecoveryReason::ContactNotImproving,
        input.now_sec);
      return safe_stop_action(RecoveryReason::ContactNotImproving);
    }
    if (reassess_after_drive_) {
      reassess_after_drive_ = false;
      transition(
        RecoveryState::StopAndReassess, RecoveryReason::EscapeStepComplete,
        input.now_sec);
      return update_stop_and_reassess(input);
    }
    if (!escape_confirmed_before_drive_) {
      transition(
        RecoveryState::SafeStop, RecoveryReason::EscapeNotConfirmed,
        input.now_sec);
      return safe_stop_action(RecoveryReason::EscapeNotConfirmed);
    }
    transition(
      RecoveryState::LowSpeedRejoin, RecoveryReason::DriveGearConfirmed,
      input.now_sec);
    return update_low_speed_rejoin(input);
  }
  if (state_elapsed(input.now_sec) >= config_.gear_report_timeout_sec) {
    transition(RecoveryState::SafeStop, RecoveryReason::GearReportTimedOut, input.now_sec);
    return safe_stop_action(RecoveryReason::GearReportTimedOut);
  }
  if (
    gear_command_request_count_ < config_.max_gear_command_requests &&
    last_gear_request_sec_.has_value() &&
    input.now_sec - *last_gear_request_sec_ >= config_.gear_command_resend_interval_sec)
  {
    return request_gear_action(Gear::Drive, RecoveryReason::DriveGearRequested, input.now_sec);
  }
  return hold_action(
    input.gear_report_fresh ? RecoveryReason::DriveGearRequested :
    RecoveryReason::GearReportMissing);
}

RecoveryAction RecoverySupervisor::update_low_speed_rejoin(const RecoveryInput & input)
{
  if (
    !input.gear_report_fresh || !gear_report_is_valid(input.reported_gear) ||
    input.reported_gear != Gear::Drive)
  {
    transition(RecoveryState::SafeStop, RecoveryReason::DriveGearLost, input.now_sec);
    return safe_stop_action(RecoveryReason::DriveGearLost);
  }
  if (!input.rejoin_safe) {
    transition(RecoveryState::SafeStop, RecoveryReason::RejoinUnsafe, input.now_sec);
    return safe_stop_action(RecoveryReason::RejoinUnsafe);
  }
  if (!input.rejoin_forward_clear) {
    if (
      config_.retry_rejoin_blocked_path &&
      (escape_step_count_ < config_.max_escape_steps ||
      attempt_count_ < config_.max_attempts))
    {
      escape_confirmed_before_drive_ = false;
      transition(
        RecoveryState::StopAndConfirm, RecoveryReason::RejoinPathBlocked,
        input.now_sec);
      return hold_action(RecoveryReason::RejoinPathBlocked);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::RejoinPathBlocked, input.now_sec);
    return safe_stop_action(RecoveryReason::RejoinPathBlocked);
  }
  if (!escape_confirmed_before_drive_) {
    transition(RecoveryState::SafeStop, RecoveryReason::EscapeNotConfirmed, input.now_sec);
    return safe_stop_action(RecoveryReason::EscapeNotConfirmed);
  }
  if (!input.rear_information_complete) {
    aligned_since_sec_.reset();
    return hold_action(RecoveryReason::RearInformationIncomplete);
  }
  if (state_elapsed(input.now_sec) >= config_.rejoin_timeout_sec) {
    if (
      config_.retry_rejoin_timeout &&
      (escape_step_count_ < config_.max_escape_steps ||
      attempt_count_ < config_.max_attempts))
    {
      escape_confirmed_before_drive_ = false;
      transition(
        RecoveryState::StopAndConfirm, RecoveryReason::RejoinTimedOut,
        input.now_sec);
      return hold_action(RecoveryReason::RejoinTimedOut);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::RejoinTimedOut, input.now_sec);
    return safe_stop_action(RecoveryReason::RejoinTimedOut);
  }

  // Always expose the one-shot reset request before Normal control can resume, including when
  // rejoin_confirm_sec is configured as zero.
  if (normal_reset_pending_) {
    return rejoin_action(
      RecoveryReason::RejoinInProgress,
      input.rejoin_steering_tire_angle_rad);
  }

  const bool aligned =
    std::abs(input.lateral_error_m) <= config_.max_rejoin_lateral_error_m &&
    std::abs(input.heading_error_rad) <= config_.max_rejoin_heading_error_rad;
  if (aligned) {
    if (!aligned_since_sec_.has_value()) {
      aligned_since_sec_ = input.now_sec;
    }
    if (input.now_sec - *aligned_since_sec_ >= config_.rejoin_confirm_sec) {
      cooldown_until_sec_ = input.now_sec + config_.cooldown_sec;
      transition(RecoveryState::Normal, RecoveryReason::RejoinComplete, input.now_sec);
      return normal_action(RecoveryReason::RejoinComplete);
    }
  } else {
    aligned_since_sec_.reset();
  }

  return rejoin_action(
    RecoveryReason::RejoinInProgress,
    input.rejoin_steering_tire_angle_rad);
}

void RecoverySupervisor::transition(
  const RecoveryState next, const RecoveryReason reason, const double now_sec) noexcept
{
  if (state_ == next) {
    state_reason_ = reason;
    return;
  }
  state_ = next;
  state_reason_ = reason;
  state_entered_sec_ = now_sec;

  if (
    next == RecoveryState::StopAndConfirm || next == RecoveryState::StopBeforeDrive ||
    next == RecoveryState::StopAndReassess)
  {
    stopped_since_sec_.reset();
  }
  if (next == RecoveryState::ShiftToReverse || next == RecoveryState::ShiftToDrive) {
    gear_command_request_count_ = 0U;
    last_gear_request_sec_.reset();
  }
  if (next == RecoveryState::LowSpeedRejoin) {
    aligned_since_sec_.reset();
    normal_reset_pending_ = true;
  } else {
    solver_unhealthy_since_sec_.reset();
  }
  if (next == RecoveryState::Normal || next == RecoveryState::SafeStop) {
    stopped_since_sec_.reset();
    aligned_since_sec_.reset();
    gear_command_request_count_ = 0U;
    last_gear_request_sec_.reset();
    if (next == RecoveryState::SafeStop) {
      normal_reset_pending_ = false;
    }
    active_stepwise_escape_ = false;
    reassess_after_drive_ = false;
    safe_stop_after_drive_ = false;
    escape_confirmed_before_drive_ = false;
  }
  if (next != RecoveryState::SafeStop) {
    clearance_safe_since_sec_.reset();
  }
}

RecoveryAction RecoverySupervisor::normal_action(const RecoveryReason reason) const noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::NormalControl;
  action.reason = reason;
  return action;
}

RecoveryAction RecoverySupervisor::hold_action(const RecoveryReason reason) const noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::HoldStop;
  action.inhibit_boost = true;
  action.reason = reason;
  return action;
}

RecoveryAction RecoverySupervisor::safe_stop_action(const RecoveryReason reason) const noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::SafeStop;
  action.inhibit_boost = true;
  action.reason = reason;
  return action;
}

RecoveryAction RecoverySupervisor::request_gear_action(
  const Gear gear, const RecoveryReason reason, const double now_sec) noexcept
{
  ++gear_command_request_count_;
  last_gear_request_sec_ = now_sec;
  RecoveryAction action;
  action.type = gear == Gear::Reverse ?
    RecoveryActionType::RequestReverse : RecoveryActionType::RequestDrive;
  action.requested_gear = gear;
  action.inhibit_boost = true;
  action.reason = reason;
  return action;
}

RecoveryAction RecoverySupervisor::reverse_action(
  const RecoveryReason reason, const double steering_tire_angle_rad) const noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::ReverseCreep;
  action.acceleration_magnitude_mps2 = config_.reverse_acceleration_magnitude_mps2;
  action.steering_tire_angle_rad = steering_tire_angle_rad;
  action.inhibit_boost = true;
  action.reason = reason;
  return action;
}

RecoveryAction RecoverySupervisor::forward_action(
  const RecoveryReason reason, const double steering_tire_angle_rad) const noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::ForwardCreep;
  action.acceleration_magnitude_mps2 = config_.forward_acceleration_magnitude_mps2;
  action.steering_tire_angle_rad = steering_tire_angle_rad;
  action.inhibit_boost = true;
  action.reason = reason;
  return action;
}

RecoveryAction RecoverySupervisor::rejoin_action(
  const RecoveryReason reason, const double steering_tire_angle_rad) noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::LowSpeedRejoin;
  action.rejoin_speed_limit_mps = config_.rejoin_speed_limit_mps;
  action.steering_tire_angle_rad = steering_tire_angle_rad;
  action.inhibit_boost = true;
  action.reset_normal_control = normal_reset_pending_;
  normal_reset_pending_ = false;
  action.reason = reason;
  return action;
}

RecoveryReason RecoverySupervisor::clearance_reason(const RecoveryInput & input) const noexcept
{
  if (!input.rear_information_complete) {
    return RecoveryReason::RearInformationIncomplete;
  }
  if (!input.rear_static_clear) {
    return RecoveryReason::RearStaticBlocked;
  }
  if (!input.rear_v2x_clear) {
    return RecoveryReason::RearVehicleBlocked;
  }
  return RecoveryReason::ClearanceCheck;
}

bool RecoverySupervisor::clearance_is_safe(const RecoveryInput & input) const noexcept
{
  return input.rear_information_complete && input.rear_static_clear && input.rear_v2x_clear;
}

bool RecoverySupervisor::stopped_confirmed(const RecoveryInput & input) noexcept
{
  if (std::abs(input.signed_speed_mps) > config_.stop_speed_mps) {
    stopped_since_sec_.reset();
    return false;
  }
  if (!stopped_since_sec_.has_value()) {
    stopped_since_sec_ = input.now_sec;
  }
  return input.now_sec - *stopped_since_sec_ >= config_.stop_confirm_sec;
}

bool RecoverySupervisor::gear_report_is_valid(const Gear gear) const noexcept
{
  return gear == Gear::Neutral || gear == Gear::Drive || gear == Gear::Reverse;
}

bool RecoverySupervisor::input_is_finite(const RecoveryInput & input) const noexcept
{
  return std::isfinite(input.now_sec) && std::isfinite(input.signed_speed_mps) &&
         finite_nonnegative(input.traveled_distance_m) &&
         finite_nonnegative(input.episode_traveled_distance_m) &&
         std::isfinite(input.reverse_steering_tire_angle_rad) &&
         std::isfinite(input.rejoin_steering_tire_angle_rad) &&
         std::isfinite(input.lateral_error_m) && std::isfinite(input.heading_error_rad) &&
         finite_nonnegative(input.detector.stationary_duration_sec) &&
         finite_nonnegative(input.detector.solver_fallback_duration_sec) &&
         finite_nonnegative(input.detector.pose_displacement_m) &&
         std::isfinite(input.detector.progress_delta_m);
}

double RecoverySupervisor::state_elapsed(const double now_sec) const noexcept
{
  if (!state_entered_sec_.has_value() || now_sec < *state_entered_sec_) {
    return 0.0;
  }
  return now_sec - *state_entered_sec_;
}

StuckRecoveryCore::StuckRecoveryCore(CoreConfig config)
: config_(std::move(config)), detector_(config_.detector), supervisor_(config_.supervisor)
{
  if (config_.supervisor.aggressive_sim_recovery_enabled && !config_.simulation_only) {
    throw std::invalid_argument(
            "aggressive simulation recovery requires CoreConfig::simulation_only");
  }
}

CoreOutput StuckRecoveryCore::update(const CoreInput & input)
{
  CoreOutput output;
  output.execution_mode = execution_mode(input.simulation_environment);
  output.actuation_allowed = output.execution_mode == ExecutionMode::Active;

  if (output.execution_mode == ExecutionMode::Disabled) {
    detector_.reset();
    supervisor_.reset_session();
    solver_fallback_recovery_episode_ = false;
    output.detector = disabled_decision(input.detector);
    output.action = RecoveryAction{};
    output.action.reason = RecoveryReason::Disabled;
    output.state = RecoveryState::Normal;
    output.state_reason = RecoveryReason::Disabled;
    return output;
  }

  output.detector = detector_.update(input.detector);
  output.shadow_candidate = is_shadow_candidate(output.detector.verdict);
  if (output.execution_mode != ExecutionMode::Active) {
    supervisor_.reset_session();
    solver_fallback_recovery_episode_ = false;
    output.action = RecoveryAction{};
    output.action.reason = output.execution_mode == ExecutionMode::Shadow ?
      RecoveryReason::ShadowMode : RecoveryReason::SimulationOnlyBlocked;
    output.state = RecoveryState::Normal;
    output.state_reason = output.action.reason;
    return output;
  }

  RecoveryInput recovery = input.recovery;
  recovery.now_sec = input.detector.now_sec;
  recovery.detector = output.detector;
  recovery.race_active = input.detector.race_started;
  recovery.control_enabled = input.detector.control_enabled;
  recovery.odometry_valid = input.detector.odometry_fresh &&
    std::isfinite(input.detector.signed_speed_mps);
  const RecoveryState current_state = supervisor_.state();
  const bool recovery_no_longer_depends_on_solver =
    current_state != RecoveryState::Normal &&
    current_state != RecoveryState::LowSpeedRejoin;
  const bool confirmed_solver_fallback_candidate =
    (current_state == RecoveryState::Normal ||
    current_state == RecoveryState::SuspectStuck) &&
    (output.detector.solver_fallback_qualified ||
    output.detector.solver_evidence_free_qualified);
  if (confirmed_solver_fallback_candidate) {
    solver_fallback_recovery_episode_ = true;
  }
  // The normal MPC solver is not used while the recovery supervisor exclusively
  // owns the stop, gear-shift, maneuver, and low-speed rejoin commands. A
  // solver-qualified episode must retain that qualification until rejoin is
  // complete; otherwise its first measured motion clears the detector timer
  // and creates a circular wait for the failed solver before external rejoin.
  // A solver failure that starts during a non-solver recovery still uses the
  // bounded LowSpeedRejoin hold/timeout in RecoverySupervisor.
  recovery.solver_healthy = !input.detector.solver_fallback ||
    recovery_no_longer_depends_on_solver ||
    confirmed_solver_fallback_candidate ||
    (current_state == RecoveryState::LowSpeedRejoin &&
    solver_fallback_recovery_episode_);
  // deliberate_stop is a detector eligibility gate for normal V2X behavior.
  // Once Recovery owns the command, directional static/V2X clearance is the
  // maneuver safety gate. Preserve only an explicit recovery hard stop from
  // the adapter instead of turning a transient Follow/SafetyBrake state into
  // a latched ControlInterrupted SafeStop.
  recovery.hard_stop_requested = input.recovery.hard_stop_requested;
  recovery.awsim_recovery_settled = !input.detector.awsim_recovery_settling;
  recovery.signed_speed_mps = input.detector.signed_speed_mps;

  output.action = supervisor_.update(recovery);
  output.state = supervisor_.state();
  output.state_reason = supervisor_.state_reason();
  if (output.state == RecoveryState::Normal) {
    solver_fallback_recovery_episode_ = false;
  }
  return output;
}

void StuckRecoveryCore::reset_session() noexcept
{
  detector_.reset();
  supervisor_.reset_session();
  solver_fallback_recovery_episode_ = false;
}

ExecutionMode StuckRecoveryCore::execution_mode(const bool simulation_environment) const noexcept
{
  if (!config_.enabled) {
    return ExecutionMode::Disabled;
  }
  if (config_.shadow_mode) {
    return ExecutionMode::Shadow;
  }
  if (config_.simulation_only && !simulation_environment) {
    return ExecutionMode::SimulationOnlyBlocked;
  }
  return ExecutionMode::Active;
}

const CoreConfig & StuckRecoveryCore::config() const noexcept
{
  return config_;
}

const StuckDetector & StuckRecoveryCore::detector() const noexcept
{
  return detector_;
}

const RecoverySupervisor & StuckRecoveryCore::supervisor() const noexcept
{
  return supervisor_;
}

const char * to_string(const StuckVerdict verdict) noexcept
{
  switch (verdict) {
    case StuckVerdict::NotEligible:
      return "NotEligible";
    case StuckVerdict::Moving:
      return "Moving";
    case StuckVerdict::Suspected:
      return "Suspected";
    case StuckVerdict::Confirmed:
      return "Confirmed";
  }
  return "Unknown";
}

const char * to_string(const StuckRejectReason reason) noexcept
{
  switch (reason) {
    case StuckRejectReason::None:
      return "none";
    case StuckRejectReason::FeatureDisabled:
      return "feature_disabled";
    case StuckRejectReason::RaceNotStarted:
      return "race_not_started";
    case StuckRejectReason::ControlDisabled:
      return "control_disabled";
    case StuckRejectReason::OdometryStale:
      return "odometry_stale";
    case StuckRejectReason::InvalidInput:
      return "invalid_input";
    case StuckRejectReason::NonMonotonicTime:
      return "non_monotonic_time";
    case StuckRejectReason::SolverFallback:
      return "solver_fallback";
    case StuckRejectReason::SolverFallbackMissingWallEvidence:
      return "solver_fallback_missing_wall_evidence";
    case StuckRejectReason::DeliberateStop:
      return "deliberate_stop";
    case StuckRejectReason::GearTransition:
      return "gear_transition";
    case StuckRejectReason::AwsimRecoverySettling:
      return "awsim_recovery_settling";
    case StuckRejectReason::NoForwardIntent:
      return "no_forward_intent";
    case StuckRejectReason::VehicleMoving:
      return "vehicle_moving";
    case StuckRejectReason::PoseProgressing:
      return "pose_progressing";
    case StuckRejectReason::PathProgressing:
      return "path_progressing";
    case StuckRejectReason::ObservationWindowIncomplete:
      return "observation_window_incomplete";
    case StuckRejectReason::MissingCorroboratingEvidence:
      return "missing_corroborating_evidence";
  }
  return "unknown";
}

const char * to_string(const Gear gear) noexcept
{
  switch (gear) {
    case Gear::NoCommand:
      return "NoCommand";
    case Gear::Unknown:
      return "Unknown";
    case Gear::Neutral:
      return "Neutral";
    case Gear::Drive:
      return "Drive";
    case Gear::Reverse:
      return "Reverse";
  }
  return "Unknown";
}

const char * to_string(const ManeuverDirection direction) noexcept
{
  switch (direction) {
    case ManeuverDirection::Unknown:
      return "Unknown";
    case ManeuverDirection::Reverse:
      return "Reverse";
    case ManeuverDirection::Forward:
      return "Forward";
  }
  return "Unknown";
}

bool recovery_candidate_commit_allowed(const RecoveryState state) noexcept
{
  switch (state) {
    case RecoveryState::ShiftToReverse:
    case RecoveryState::WaitReverseReport:
    case RecoveryState::ReverseManeuver:
    case RecoveryState::ForwardManeuver:
      return true;
    case RecoveryState::Normal:
    case RecoveryState::SuspectStuck:
    case RecoveryState::WaitAwsimRecovery:
    case RecoveryState::StopAndConfirm:
    case RecoveryState::CheckClearance:
    case RecoveryState::WaitForClear:
    case RecoveryState::StopAndReassess:
    case RecoveryState::StopBeforeDrive:
    case RecoveryState::ShiftToDrive:
    case RecoveryState::WaitDriveReport:
    case RecoveryState::LowSpeedRejoin:
    case RecoveryState::SafeStop:
      return false;
  }
  return false;
}

bool recovery_reverse_intent_latch_allowed(const RecoveryState state) noexcept
{
  switch (state) {
    case RecoveryState::StopAndConfirm:
    case RecoveryState::CheckClearance:
    case RecoveryState::WaitForClear:
    case RecoveryState::ShiftToReverse:
    case RecoveryState::WaitReverseReport:
    case RecoveryState::ReverseManeuver:
    case RecoveryState::StopAndReassess:
    case RecoveryState::StopBeforeDrive:
    case RecoveryState::ShiftToDrive:
    case RecoveryState::WaitDriveReport:
      return true;
    case RecoveryState::Normal:
    case RecoveryState::SuspectStuck:
    case RecoveryState::WaitAwsimRecovery:
    case RecoveryState::ForwardManeuver:
    case RecoveryState::LowSpeedRejoin:
    case RecoveryState::SafeStop:
      return false;
  }
  return false;
}

bool recovery_reverse_direction_required(
  const ReverseDirectionPolicyInput & input) noexcept
{
  return input.reverse_only_episode || input.reverse_intent_latched ||
         (input.coordinated_stop_active && !input.forward_fallback_unlocked) ||
         input.solver_reverse_only_candidate ||
         (input.obstacle_reverse_first && !input.forward_fallback_unlocked);
}

const char * to_string(const RecoveryState state) noexcept
{
  switch (state) {
    case RecoveryState::Normal:
      return "NORMAL";
    case RecoveryState::SuspectStuck:
      return "SUSPECT_STUCK";
    case RecoveryState::WaitAwsimRecovery:
      return "WAIT_AWSIM_RECOVERY";
    case RecoveryState::StopAndConfirm:
      return "STOP_AND_CONFIRM";
    case RecoveryState::CheckClearance:
      return "CHECK_CLEARANCE";
    case RecoveryState::WaitForClear:
      return "WAIT_FOR_CLEAR";
    case RecoveryState::ShiftToReverse:
      return "SHIFT_TO_REVERSE";
    case RecoveryState::WaitReverseReport:
      return "WAIT_REVERSE_REPORT";
    case RecoveryState::ReverseManeuver:
      return "REVERSE_MANEUVER";
    case RecoveryState::ForwardManeuver:
      return "FORWARD_MANEUVER";
    case RecoveryState::StopAndReassess:
      return "STOP_AND_REASSESS";
    case RecoveryState::StopBeforeDrive:
      return "STOP_BEFORE_DRIVE";
    case RecoveryState::ShiftToDrive:
      return "SHIFT_TO_DRIVE";
    case RecoveryState::WaitDriveReport:
      return "WAIT_DRIVE_REPORT";
    case RecoveryState::LowSpeedRejoin:
      return "LOW_SPEED_REJOIN";
    case RecoveryState::SafeStop:
      return "SAFE_STOP";
  }
  return "UNKNOWN";
}

const char * to_string(const RecoveryActionType action) noexcept
{
  switch (action) {
    case RecoveryActionType::NormalControl:
      return "NormalControl";
    case RecoveryActionType::HoldStop:
      return "HoldStop";
    case RecoveryActionType::RequestReverse:
      return "RequestReverse";
    case RecoveryActionType::ReverseCreep:
      return "ReverseCreep";
    case RecoveryActionType::ForwardCreep:
      return "ForwardCreep";
    case RecoveryActionType::RequestDrive:
      return "RequestDrive";
    case RecoveryActionType::LowSpeedRejoin:
      return "LowSpeedRejoin";
    case RecoveryActionType::SafeStop:
      return "SafeStop";
  }
  return "Unknown";
}

const char * to_string(const RecoveryReason reason) noexcept
{
  switch (reason) {
    case RecoveryReason::None:
      return "none";
    case RecoveryReason::Disabled:
      return "disabled";
    case RecoveryReason::ShadowMode:
      return "shadow_mode";
    case RecoveryReason::SimulationOnlyBlocked:
      return "simulation_only_blocked";
    case RecoveryReason::RaceInactive:
      return "race_inactive";
    case RecoveryReason::AwaitingStuckConfirmation:
      return "awaiting_stuck_confirmation";
    case RecoveryReason::StuckSuspected:
      return "stuck_suspected";
    case RecoveryReason::StuckConfirmed:
      return "stuck_confirmed";
    case RecoveryReason::AwsimRecoveryWaiting:
      return "awsim_recovery_waiting";
    case RecoveryReason::AwsimRecoveryResolved:
      return "awsim_recovery_resolved";
    case RecoveryReason::StopConfirmationPending:
      return "stop_confirmation_pending";
    case RecoveryReason::ClearanceCheck:
      return "clearance_check";
    case RecoveryReason::RearStaticBlocked:
      return "rear_static_blocked";
    case RecoveryReason::RearVehicleBlocked:
      return "rear_vehicle_blocked";
    case RecoveryReason::RearInformationIncomplete:
      return "rear_information_incomplete";
    case RecoveryReason::ClearanceWaitTimedOut:
      return "clearance_wait_timed_out";
    case RecoveryReason::ManeuverDirectionUnknown:
      return "maneuver_direction_unknown";
    case RecoveryReason::AttemptLimitReached:
      return "attempt_limit_reached";
    case RecoveryReason::ReverseGearRequested:
      return "reverse_gear_requested";
    case RecoveryReason::ReverseGearConfirmed:
      return "reverse_gear_confirmed";
    case RecoveryReason::GearReportMissing:
      return "gear_report_missing";
    case RecoveryReason::GearReportInvalid:
      return "gear_report_invalid";
    case RecoveryReason::GearReportTimedOut:
      return "gear_report_timed_out";
    case RecoveryReason::GearCommandLimitReached:
      return "gear_command_limit_reached";
    case RecoveryReason::ReverseInProgress:
      return "reverse_in_progress";
    case RecoveryReason::ReverseEscapeBraking:
      return "reverse_escape_braking";
    case RecoveryReason::ReverseDistanceLimit:
      return "reverse_distance_limit";
    case RecoveryReason::ReverseDurationLimit:
      return "reverse_duration_limit";
    case RecoveryReason::ReverseSpeedLimit:
      return "reverse_speed_limit";
    case RecoveryReason::ReverseEscapeConfirmed:
      return "reverse_escape_confirmed";
    case RecoveryReason::CollisionWorsening:
      return "collision_worsening";
    case RecoveryReason::RearHazardAppeared:
      return "rear_hazard_appeared";
    case RecoveryReason::ReverseGearLost:
      return "reverse_gear_lost";
    case RecoveryReason::ForwardInProgress:
      return "forward_in_progress";
    case RecoveryReason::ForwardDistanceLimit:
      return "forward_distance_limit";
    case RecoveryReason::ForwardDurationLimit:
      return "forward_duration_limit";
    case RecoveryReason::ForwardSpeedLimit:
      return "forward_speed_limit";
    case RecoveryReason::ForwardEscapeConfirmed:
      return "forward_escape_confirmed";
    case RecoveryReason::ForwardHazardAppeared:
      return "forward_hazard_appeared";
    case RecoveryReason::EscapeStepComplete:
      return "escape_step_complete";
    case RecoveryReason::EscapeStepLimitReached:
      return "escape_step_limit_reached";
    case RecoveryReason::ContactNotImproving:
      return "contact_not_improving";
    case RecoveryReason::DriveGearRequested:
      return "drive_gear_requested";
    case RecoveryReason::DriveGearConfirmed:
      return "drive_gear_confirmed";
    case RecoveryReason::DriveGearLost:
      return "drive_gear_lost";
    case RecoveryReason::EscapeNotConfirmed:
      return "escape_not_confirmed";
    case RecoveryReason::RejoinInProgress:
      return "rejoin_in_progress";
    case RecoveryReason::RejoinComplete:
      return "rejoin_complete";
    case RecoveryReason::RejoinTimedOut:
      return "rejoin_timed_out";
    case RecoveryReason::RejoinUnsafe:
      return "rejoin_unsafe";
    case RecoveryReason::RejoinPathBlocked:
      return "rejoin_path_blocked";
    case RecoveryReason::SolverRecoveryPending:
      return "solver_recovery_pending";
    case RecoveryReason::AggressiveRetry:
      return "aggressive_retry";
    case RecoveryReason::CooldownActive:
      return "cooldown_active";
    case RecoveryReason::OdometryUnsafe:
      return "odometry_unsafe";
    case RecoveryReason::SolverUnsafe:
      return "solver_unsafe";
    case RecoveryReason::ControlInterrupted:
      return "control_interrupted";
    case RecoveryReason::InvalidInput:
      return "invalid_input";
    case RecoveryReason::NonMonotonicTime:
      return "non_monotonic_time";
    case RecoveryReason::SessionReset:
      return "session_reset";
  }
  return "unknown";
}

const char * to_string(const ExecutionMode mode) noexcept
{
  switch (mode) {
    case ExecutionMode::Disabled:
      return "disabled";
    case ExecutionMode::Shadow:
      return "shadow";
    case ExecutionMode::SimulationOnlyBlocked:
      return "simulation_only_blocked";
    case ExecutionMode::Active:
      return "active";
  }
  return "unknown";
}

}  // namespace multi_purpose_mpc_ros::stuck_recovery
