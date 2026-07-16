#include "multi_purpose_mpc_ros/awsim_boost_start_dash.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace multi_purpose_mpc_ros::awsim_boost
{
namespace
{

std::string normalize(std::string_view value)
{
  const auto is_space = [](const unsigned char character) {
      return std::isspace(character) != 0;
    };
  const auto first = std::find_if_not(value.begin(), value.end(), is_space);
  const auto last = std::find_if_not(value.rbegin(), value.rend(), is_space).base();
  if (first >= last) {
    return {};
  }

  std::string normalized(first, last);
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return normalized;
}

}  // namespace

StartDashGuard::StartDashGuard(Config config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.status_timeout_sec) || config_.status_timeout_sec <= 0.0) {
    throw std::invalid_argument("AWSIM boost status timeout must be finite and positive");
  }
  if (
    !std::isfinite(config_.confirmation_timeout_sec) ||
    config_.confirmation_timeout_sec <= 0.0)
  {
    throw std::invalid_argument("AWSIM boost confirmation timeout must be finite and positive");
  }
  if (
    !std::isfinite(config_.motion_speed_threshold_mps) ||
    config_.motion_speed_threshold_mps <= 0.0)
  {
    throw std::invalid_argument("AWSIM boost motion speed threshold must be finite and positive");
  }
  if (
    !std::isfinite(config_.max_trigger_speed_mps) ||
    config_.max_trigger_speed_mps < config_.motion_speed_threshold_mps)
  {
    throw std::invalid_argument(
            "AWSIM boost maximum trigger speed must be finite and at least the motion threshold");
  }
  if (
    !std::isfinite(config_.motion_trigger_timeout_sec) ||
    config_.motion_trigger_timeout_sec <= 0.0)
  {
    throw std::invalid_argument("AWSIM boost motion trigger timeout must be finite and positive");
  }
  phase_ = config_.enabled && config_.mode == Mode::StartOnce ? Phase::Armed : Phase::Disabled;
}

StateEvent StartDashGuard::on_awsim_state(const std::string_view state)
{
  const std::string normalized = normalize(state);
  if (normalized == "ready") {
    if (
      config_.trigger == Trigger::FirstForwardMotion && !finish_seen_ &&
      phase_ == Phase::Armed)
    {
      ready_seen_ = true;
      phase_ = Phase::AwaitingMotion;
    }
    return StateEvent::None;
  }
  if (normalized == "start") {
    // A stale or reordered Start after Finish still belongs to the spent session. Wait for the
    // explicit Finish -> Spawned boundary before accepting a new start.
    if (finish_seen_) {
      return StateEvent::None;
    }
    start_seen_ = true;
    if (
      config_.trigger == Trigger::FirstForwardMotion && phase_ == Phase::Armed)
    {
      // Fallback for launch paths that omit Ready. The speed ceiling is checked by evaluate().
      ready_seen_ = true;
      phase_ = Phase::AwaitingMotion;
    }
    if (start_event_emitted_) {
      return StateEvent::None;
    }
    start_event_emitted_ = true;
    return StateEvent::StartEntered;
  }
  if (normalized == "finish") {
    start_seen_ = false;
    ready_seen_ = false;
    motion_detected_at_.reset();
    if (phase_ == Phase::AwaitingMotion || phase_ == Phase::MotionDetected) {
      phase_ = Phase::Armed;
    }
    if (finish_seen_) {
      return StateEvent::None;
    }
    finish_seen_ = true;
    return StateEvent::Finished;
  }
  if (normalized != "spawned") {
    return StateEvent::None;
  }

  if (finish_seen_) {
    rearm_for_new_session();
    return StateEvent::NewSession;
  }

  // A normal race begins with Spawned. It may also be duplicated or delayed, so a Spawned
  // message without a preceding Finish must never clear a spent latch.
  if (phase_ == Phase::Armed) {
    start_seen_ = false;
    status_.reset();
  }
  return StateEvent::None;
}

bool StartDashGuard::on_awsim_status(
  const std::vector<float> & data, const TimePoint received_at)
{
  if (phase_ == Phase::Disabled) {
    return false;
  }
  if (data.size() < 7U) {
    status_.reset();
    return false;
  }

  const double remaining = static_cast<double>(data[5]);
  const double boosting_value = static_cast<double>(data[6]);
  if (!std::isfinite(remaining) || !std::isfinite(boosting_value)) {
    status_.reset();
    return false;
  }

  status_ = Status{remaining, boosting_value >= 0.5, received_at};
  update_confirmation_from_status();
  return true;
}

Evaluation StartDashGuard::evaluate(const TriggerContext & context, const TimePoint now)
{
  Evaluation evaluation;
  if (phase_ == Phase::Disabled) {
    evaluation.reason = BlockReason::Disabled;
    return evaluation;
  }

  if (phase_ == Phase::PulseSent) {
    if (
      pulse_sent_at_.has_value() && now >= pulse_sent_at_.value() &&
      std::chrono::duration<double>(now - pulse_sent_at_.value()).count() >=
      config_.confirmation_timeout_sec)
    {
      phase_ = Phase::UnconfirmedSpent;
      evaluation.reason = BlockReason::ConfirmationTimedOut;
      return evaluation;
    }
    evaluation.reason = BlockReason::AlreadySpent;
    return evaluation;
  }
  if (
    phase_ == Phase::Confirmed || phase_ == Phase::UnconfirmedSpent ||
    phase_ == Phase::LaunchExpiredSpent)
  {
    evaluation.reason = BlockReason::AlreadySpent;
    return evaluation;
  }

  if (config_.trigger == Trigger::AwsimStart) {
    if (!start_seen_) {
      evaluation.reason = BlockReason::AwaitingStart;
      return evaluation;
    }
  } else {
    if (!ready_seen_ || phase_ == Phase::Armed) {
      evaluation.reason = BlockReason::AwaitingReady;
      return evaluation;
    }
    if (!std::isfinite(context.forward_speed_mps)) {
      evaluation.reason = BlockReason::InvalidForwardSpeed;
      return evaluation;
    }
    if (!motion_detected_at_.has_value()) {
      if (context.forward_speed_mps > config_.max_trigger_speed_mps) {
        phase_ = Phase::LaunchExpiredSpent;
        evaluation.reason = BlockReason::MotionTriggerSpeedExceeded;
        return evaluation;
      }
      if (context.forward_speed_mps < config_.motion_speed_threshold_mps) {
        evaluation.reason = BlockReason::AwaitingMotion;
        return evaluation;
      }
      motion_detected_at_ = now;
      phase_ = Phase::MotionDetected;
      evaluation.motion_detected_now = true;
    }
    if (now < motion_detected_at_.value()) {
      phase_ = Phase::LaunchExpiredSpent;
      evaluation.reason = BlockReason::MotionTriggerTimedOut;
      return evaluation;
    }
    evaluation.motion_elapsed_sec =
      std::chrono::duration<double>(now - motion_detected_at_.value()).count();
    if (evaluation.motion_elapsed_sec > config_.motion_trigger_timeout_sec) {
      phase_ = Phase::LaunchExpiredSpent;
      evaluation.reason = BlockReason::MotionTriggerTimedOut;
      return evaluation;
    }
    if (context.forward_speed_mps > config_.max_trigger_speed_mps) {
      phase_ = Phase::LaunchExpiredSpent;
      evaluation.reason = BlockReason::MotionTriggerSpeedExceeded;
      return evaluation;
    }
  }

  if (!context.control_enabled) {
    evaluation.reason = BlockReason::ControlDisabled;
    return evaluation;
  }
  if (!context.normal_command_published) {
    evaluation.reason = BlockReason::CommandNotPublished;
    return evaluation;
  }
  if (context.solver_fallback_active) {
    evaluation.reason = BlockReason::SolverFallbackActive;
    return evaluation;
  }
  if (context.v2x_safety_brake_active) {
    evaluation.reason = BlockReason::SafetyBrakeActive;
    return evaluation;
  }
  if (context.reverse_or_recovery_active) {
    evaluation.reason = BlockReason::ReverseOrRecoveryActive;
    return evaluation;
  }
  if (context.failsafe_active) {
    evaluation.reason = BlockReason::FailsafeActive;
    return evaluation;
  }
  if (!status_.has_value()) {
    evaluation.reason = BlockReason::MissingStatus;
    return evaluation;
  }
  if (now < status_->received_at) {
    evaluation.reason = BlockReason::StaleStatus;
    return evaluation;
  }
  const double status_age = std::chrono::duration<double>(now - status_->received_at).count();
  if (status_age > config_.status_timeout_sec) {
    evaluation.reason = BlockReason::StaleStatus;
    return evaluation;
  }
  if (status_->remaining < 1.0) {
    evaluation.reason = BlockReason::NoRemainingBoost;
    return evaluation;
  }
  if (status_->is_boosting) {
    evaluation.reason = BlockReason::AlreadyBoosting;
    return evaluation;
  }

  remaining_before_pulse_ = status_->remaining;
  pulse_sent_at_ = now;
  phase_ = Phase::PulseSent;
  evaluation.action = Action::PublishPulse;
  return evaluation;
}

Evaluation StartDashGuard::evaluate(
  const bool control_enabled, const bool failsafe_active, const TimePoint now)
{
  TriggerContext context;
  context.control_enabled = control_enabled;
  context.normal_command_published = true;
  context.failsafe_active = failsafe_active;
  return evaluate(context, now);
}

Phase StartDashGuard::phase() const noexcept
{
  return phase_;
}

bool StartDashGuard::has_valid_status() const noexcept
{
  return status_.has_value();
}

std::optional<double> StartDashGuard::remaining() const noexcept
{
  if (!status_.has_value()) {
    return std::nullopt;
  }
  return status_->remaining;
}

std::optional<bool> StartDashGuard::is_boosting() const noexcept
{
  if (!status_.has_value()) {
    return std::nullopt;
  }
  return status_->is_boosting;
}

void StartDashGuard::rearm_for_new_session()
{
  phase_ = config_.enabled && config_.mode == Mode::StartOnce ? Phase::Armed : Phase::Disabled;
  start_seen_ = false;
  ready_seen_ = false;
  start_event_emitted_ = false;
  finish_seen_ = false;
  status_.reset();
  motion_detected_at_.reset();
  remaining_before_pulse_.reset();
  pulse_sent_at_.reset();
}

void StartDashGuard::update_confirmation_from_status()
{
  if (
    phase_ != Phase::PulseSent || !status_.has_value() ||
    !remaining_before_pulse_.has_value())
  {
    return;
  }

  if (
    status_->is_boosting ||
    status_->remaining <= remaining_before_pulse_.value() - 0.5)
  {
    phase_ = Phase::Confirmed;
  }
}

Mode parse_mode(const std::string_view value)
{
  const std::string normalized = normalize(value);
  if (normalized == "disabled") {
    return Mode::Disabled;
  }
  if (normalized == "start_once") {
    return Mode::StartOnce;
  }
  throw std::invalid_argument("unknown AWSIM boost mode: " + std::string(value));
}

Trigger parse_trigger(const std::string_view value)
{
  const std::string normalized = normalize(value);
  if (normalized == "awsim_start") {
    return Trigger::AwsimStart;
  }
  if (normalized == "first_forward_motion") {
    return Trigger::FirstForwardMotion;
  }
  throw std::invalid_argument("unknown AWSIM boost trigger: " + std::string(value));
}

EnabledResolution resolve_enabled(
  const bool default_enabled, const std::map<int, bool> & domain_enabled,
  const std::optional<int> ros_domain_id) noexcept
{
  if (!ros_domain_id.has_value()) {
    return {default_enabled, false, -1};
  }

  const auto override = domain_enabled.find(ros_domain_id.value());
  if (override == domain_enabled.end()) {
    return {default_enabled, false, ros_domain_id.value()};
  }
  return {override->second, true, ros_domain_id.value()};
}

const char * to_string(const Mode mode) noexcept
{
  switch (mode) {
    case Mode::Disabled:
      return "disabled";
    case Mode::StartOnce:
      return "start_once";
  }
  return "unknown";
}

const char * to_string(const Trigger trigger) noexcept
{
  switch (trigger) {
    case Trigger::AwsimStart:
      return "awsim_start";
    case Trigger::FirstForwardMotion:
      return "first_forward_motion";
  }
  return "unknown";
}

const char * to_string(const Phase phase) noexcept
{
  switch (phase) {
    case Phase::Disabled:
      return "Disabled";
    case Phase::Armed:
      return "Armed";
    case Phase::AwaitingMotion:
      return "AwaitingMotion";
    case Phase::MotionDetected:
      return "MotionDetected";
    case Phase::PulseSent:
      return "PulseSent";
    case Phase::Confirmed:
      return "Confirmed";
    case Phase::UnconfirmedSpent:
      return "UnconfirmedSpent";
    case Phase::LaunchExpiredSpent:
      return "LaunchExpiredSpent";
  }
  return "Unknown";
}

const char * to_string(const StateEvent event) noexcept
{
  switch (event) {
    case StateEvent::None:
      return "None";
    case StateEvent::StartEntered:
      return "StartEntered";
    case StateEvent::Finished:
      return "Finished";
    case StateEvent::NewSession:
      return "NewSession";
  }
  return "Unknown";
}

const char * to_string(const BlockReason reason) noexcept
{
  switch (reason) {
    case BlockReason::None:
      return "none";
    case BlockReason::Disabled:
      return "disabled";
    case BlockReason::AwaitingStart:
      return "awaiting Start state";
    case BlockReason::AwaitingReady:
      return "awaiting Ready state";
    case BlockReason::AwaitingMotion:
      return "awaiting forward launch motion";
    case BlockReason::ControlDisabled:
      return "control disabled";
    case BlockReason::CommandNotPublished:
      return "normal control command not published";
    case BlockReason::FailsafeActive:
      return "control failsafe active";
    case BlockReason::SafetyBrakeActive:
      return "V2X SafetyBrake active";
    case BlockReason::SolverFallbackActive:
      return "MPC solver fallback active";
    case BlockReason::ReverseOrRecoveryActive:
      return "reverse or stuck recovery active";
    case BlockReason::InvalidForwardSpeed:
      return "invalid forward speed";
    case BlockReason::MotionTriggerTimedOut:
      return "launch motion trigger timed out";
    case BlockReason::MotionTriggerSpeedExceeded:
      return "launch motion trigger speed exceeded";
    case BlockReason::MissingStatus:
      return "missing valid /awsim/status";
    case BlockReason::StaleStatus:
      return "stale /awsim/status";
    case BlockReason::NoRemainingBoost:
      return "no remaining boost";
    case BlockReason::AlreadyBoosting:
      return "AWSIM already boosting";
    case BlockReason::AlreadySpent:
      return "start boost already spent";
    case BlockReason::ConfirmationTimedOut:
      return "boost confirmation timed out; no retry";
  }
  return "unknown";
}

}  // namespace multi_purpose_mpc_ros::awsim_boost
