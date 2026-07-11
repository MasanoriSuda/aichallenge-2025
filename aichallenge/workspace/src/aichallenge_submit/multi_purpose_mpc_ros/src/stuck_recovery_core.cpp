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
  validate_nonnegative(config.gear_report_timeout_sec, "gear report timeout");
  validate_nonnegative(
    config.gear_command_resend_interval_sec, "gear command resend interval");
  validate_nonnegative(config.max_reverse_distance_m, "maximum reverse distance");
  validate_nonnegative(config.max_reverse_duration_sec, "maximum reverse duration");
  validate_nonnegative(
    config.reverse_acceleration_magnitude_mps2, "reverse acceleration magnitude");
  validate_nonnegative(config.rejoin_speed_limit_mps, "rejoin speed limit");
  validate_nonnegative(config.max_rejoin_lateral_error_m, "maximum rejoin lateral error");
  validate_nonnegative(config.max_rejoin_heading_error_rad, "maximum rejoin heading error");
  validate_nonnegative(config.rejoin_confirm_sec, "rejoin confirmation duration");
  validate_nonnegative(config.rejoin_timeout_sec, "rejoin timeout");
  validate_nonnegative(config.cooldown_sec, "recovery cooldown");
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
  last_update_sec_ = input.now_sec;

  const bool forward_intent =
    input.requested_forward_speed_mps >= config_.forward_intent_speed_mps ||
    input.requested_acceleration_mps2 >= config_.forward_intent_acceleration_mps2;
  const bool evidence = input.wall_evidence || input.collision_hint;

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
  if (input.solver_fallback) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::SolverFallback);
  }
  if (input.deliberate_stop) {
    return reject_and_reset(StuckVerdict::NotEligible, StuckRejectReason::DeliberateStop);
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
  if (stationary_duration_sec < config_.stationary_duration_sec) {
    return reject(
      input, StuckVerdict::Moving, StuckRejectReason::ObservationWindowIncomplete,
      forward_intent, evidence);
  }
  if (!evidence) {
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
  decision.pose_displacement_m =
    std::isfinite(input.pose_displacement_m) ? input.pose_displacement_m : 0.0;
  decision.progress_delta_m =
    std::isfinite(input.unwrapped_progress_delta_m) ?
    input.unwrapped_progress_delta_m : 0.0;
  decision.forward_intent = forward_intent;
  decision.corroborating_evidence = evidence;
  return decision;
}

void StuckDetector::reset_observation() noexcept
{
  stationary_since_sec_.reset();
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
  if (!input.solver_healthy) {
    if (state_ == RecoveryState::Normal || state_ == RecoveryState::SuspectStuck) {
      transition(RecoveryState::Normal, RecoveryReason::SolverUnsafe, input.now_sec);
      return normal_action(RecoveryReason::SolverUnsafe);
    }
    transition(RecoveryState::SafeStop, RecoveryReason::SolverUnsafe, input.now_sec);
    return safe_stop_action(RecoveryReason::SolverUnsafe);
  }
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
      return safe_stop_action(state_reason_);
  }
  transition(RecoveryState::SafeStop, RecoveryReason::InvalidInput, input.now_sec);
  return safe_stop_action(RecoveryReason::InvalidInput);
}

void RecoverySupervisor::reset_session() noexcept
{
  state_ = RecoveryState::Normal;
  state_reason_ = RecoveryReason::SessionReset;
  attempt_count_ = 0U;
  gear_command_request_count_ = 0U;
  state_entered_sec_.reset();
  last_update_sec_.reset();
  stopped_since_sec_.reset();
  aligned_since_sec_.reset();
  last_gear_request_sec_.reset();
  cooldown_until_sec_.reset();
  normal_reset_pending_ = false;
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

bool RecoverySupervisor::safe_stop_latched() const noexcept
{
  return state_ == RecoveryState::SafeStop;
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
    transition(RecoveryState::SuspectStuck, RecoveryReason::StuckConfirmed, input.now_sec);
    return hold_action(RecoveryReason::StuckConfirmed);
  }
  if (input.detector.verdict == StuckVerdict::Suspected) {
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
  if (input.detector.verdict != StuckVerdict::Confirmed) {
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
  if (!clearance_is_safe(input)) {
    const RecoveryReason reason = clearance_reason(input);
    transition(RecoveryState::WaitForClear, reason, input.now_sec);
    return hold_action(reason);
  }
  if (attempt_count_ >= config_.max_attempts) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::AttemptLimitReached, input.now_sec);
    return safe_stop_action(RecoveryReason::AttemptLimitReached);
  }
  if (config_.max_gear_command_requests == 0U) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::GearCommandLimitReached, input.now_sec);
    return safe_stop_action(RecoveryReason::GearCommandLimitReached);
  }

  ++attempt_count_;
  transition(
    RecoveryState::ShiftToReverse, RecoveryReason::ReverseGearRequested, input.now_sec);
  return request_gear_action(Gear::Reverse, RecoveryReason::ReverseGearRequested, input.now_sec);
}

RecoveryAction RecoverySupervisor::update_wait_for_clear(const RecoveryInput & input)
{
  if (clearance_is_safe(input)) {
    transition(RecoveryState::CheckClearance, RecoveryReason::ClearanceCheck, input.now_sec);
    return hold_action(RecoveryReason::ClearanceCheck);
  }
  if (state_elapsed(input.now_sec) >= config_.clearance_wait_timeout_sec) {
    transition(
      RecoveryState::SafeStop, RecoveryReason::ClearanceWaitTimedOut, input.now_sec);
    return safe_stop_action(RecoveryReason::ClearanceWaitTimedOut);
  }
  return hold_action(clearance_reason(input));
}

RecoveryAction RecoverySupervisor::update_wait_reverse_report(const RecoveryInput & input)
{
  if (input.gear_report_fresh && !gear_report_is_valid(input.reported_gear)) {
    transition(RecoveryState::SafeStop, RecoveryReason::GearReportInvalid, input.now_sec);
    return safe_stop_action(RecoveryReason::GearReportInvalid);
  }
  if (input.gear_report_fresh && input.reported_gear == Gear::Reverse) {
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
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::CollisionWorsening,
      input.now_sec);
    return hold_action(RecoveryReason::CollisionWorsening);
  }
  if (!clearance_is_safe(input)) {
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::RearHazardAppeared,
      input.now_sec);
    return hold_action(RecoveryReason::RearHazardAppeared);
  }
  if (input.traveled_distance_m >= config_.max_reverse_distance_m) {
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::ReverseDistanceLimit,
      input.now_sec);
    return hold_action(RecoveryReason::ReverseDistanceLimit);
  }
  if (state_elapsed(input.now_sec) >= config_.max_reverse_duration_sec) {
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::ReverseDurationLimit,
      input.now_sec);
    return hold_action(RecoveryReason::ReverseDurationLimit);
  }
  if (input.recovery_escape_confirmed) {
    transition(
      RecoveryState::StopBeforeDrive, RecoveryReason::ReverseEscapeConfirmed,
      input.now_sec);
    return hold_action(RecoveryReason::ReverseEscapeConfirmed);
  }
  return reverse_action(RecoveryReason::ReverseInProgress);
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
  if (state_elapsed(input.now_sec) >= config_.rejoin_timeout_sec) {
    transition(RecoveryState::SafeStop, RecoveryReason::RejoinTimedOut, input.now_sec);
    return safe_stop_action(RecoveryReason::RejoinTimedOut);
  }

  // Always expose the one-shot reset request before Normal control can resume, including when
  // rejoin_confirm_sec is configured as zero.
  if (normal_reset_pending_) {
    return rejoin_action(RecoveryReason::RejoinInProgress);
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

  return rejoin_action(RecoveryReason::RejoinInProgress);
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

  if (next == RecoveryState::StopAndConfirm || next == RecoveryState::StopBeforeDrive) {
    stopped_since_sec_.reset();
  }
  if (next == RecoveryState::ShiftToReverse || next == RecoveryState::ShiftToDrive) {
    gear_command_request_count_ = 0U;
    last_gear_request_sec_.reset();
  }
  if (next == RecoveryState::LowSpeedRejoin) {
    aligned_since_sec_.reset();
    normal_reset_pending_ = true;
  }
  if (next == RecoveryState::Normal || next == RecoveryState::SafeStop) {
    stopped_since_sec_.reset();
    aligned_since_sec_.reset();
    gear_command_request_count_ = 0U;
    last_gear_request_sec_.reset();
    if (next == RecoveryState::SafeStop) {
      normal_reset_pending_ = false;
    }
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

RecoveryAction RecoverySupervisor::reverse_action(const RecoveryReason reason) const noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::ReverseCreep;
  action.acceleration_magnitude_mps2 = config_.reverse_acceleration_magnitude_mps2;
  action.steering_tire_angle_rad = 0.0;
  action.inhibit_boost = true;
  action.reason = reason;
  return action;
}

RecoveryAction RecoverySupervisor::rejoin_action(const RecoveryReason reason) noexcept
{
  RecoveryAction action;
  action.type = RecoveryActionType::LowSpeedRejoin;
  action.rejoin_speed_limit_mps = config_.rejoin_speed_limit_mps;
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
         std::isfinite(input.lateral_error_m) && std::isfinite(input.heading_error_rad) &&
         finite_nonnegative(input.detector.stationary_duration_sec) &&
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
}

CoreOutput StuckRecoveryCore::update(const CoreInput & input)
{
  CoreOutput output;
  output.execution_mode = execution_mode(input.simulation_environment);
  output.actuation_allowed = output.execution_mode == ExecutionMode::Active;

  if (output.execution_mode == ExecutionMode::Disabled) {
    detector_.reset();
    supervisor_.reset_session();
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
  recovery.solver_healthy = !input.detector.solver_fallback;
  recovery.hard_stop_requested = input.detector.deliberate_stop;
  recovery.awsim_recovery_settled = !input.detector.awsim_recovery_settling;
  recovery.signed_speed_mps = input.detector.signed_speed_mps;

  output.action = supervisor_.update(recovery);
  output.state = supervisor_.state();
  output.state_reason = supervisor_.state_reason();
  return output;
}

void StuckRecoveryCore::reset_session() noexcept
{
  detector_.reset();
  supervisor_.reset_session();
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
    case RecoveryReason::ReverseDistanceLimit:
      return "reverse_distance_limit";
    case RecoveryReason::ReverseDurationLimit:
      return "reverse_duration_limit";
    case RecoveryReason::ReverseEscapeConfirmed:
      return "reverse_escape_confirmed";
    case RecoveryReason::CollisionWorsening:
      return "collision_worsening";
    case RecoveryReason::RearHazardAppeared:
      return "rear_hazard_appeared";
    case RecoveryReason::ReverseGearLost:
      return "reverse_gear_lost";
    case RecoveryReason::DriveGearRequested:
      return "drive_gear_requested";
    case RecoveryReason::DriveGearConfirmed:
      return "drive_gear_confirmed";
    case RecoveryReason::DriveGearLost:
      return "drive_gear_lost";
    case RecoveryReason::RejoinInProgress:
      return "rejoin_in_progress";
    case RecoveryReason::RejoinComplete:
      return "rejoin_complete";
    case RecoveryReason::RejoinTimedOut:
      return "rejoin_timed_out";
    case RecoveryReason::RejoinUnsafe:
      return "rejoin_unsafe";
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
