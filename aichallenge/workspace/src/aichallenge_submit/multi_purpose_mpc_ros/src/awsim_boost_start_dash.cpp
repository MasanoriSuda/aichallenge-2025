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
  phase_ = config_.enabled && config_.mode == Mode::StartOnce ? Phase::Armed : Phase::Disabled;
}

StateEvent StartDashGuard::on_awsim_state(const std::string_view state)
{
  const std::string normalized = normalize(state);
  if (normalized == "start") {
    // A stale or reordered Start after Finish still belongs to the spent session. Wait for the
    // explicit Finish -> Spawned boundary before accepting a new start.
    if (finish_seen_) {
      return StateEvent::None;
    }
    start_seen_ = true;
    if (start_event_emitted_) {
      return StateEvent::None;
    }
    start_event_emitted_ = true;
    return StateEvent::StartEntered;
  }
  if (normalized == "finish") {
    start_seen_ = false;
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

Evaluation StartDashGuard::evaluate(
  const bool control_enabled, const bool failsafe_active, const TimePoint now)
{
  if (phase_ == Phase::Disabled) {
    return {Action::None, BlockReason::Disabled};
  }

  if (phase_ == Phase::PulseSent) {
    if (
      pulse_sent_at_.has_value() && now >= pulse_sent_at_.value() &&
      std::chrono::duration<double>(now - pulse_sent_at_.value()).count() >=
      config_.confirmation_timeout_sec)
    {
      phase_ = Phase::UnconfirmedSpent;
      return {Action::None, BlockReason::ConfirmationTimedOut};
    }
    return {Action::None, BlockReason::AlreadySpent};
  }
  if (phase_ == Phase::Confirmed || phase_ == Phase::UnconfirmedSpent) {
    return {Action::None, BlockReason::AlreadySpent};
  }
  if (!start_seen_) {
    return {Action::None, BlockReason::AwaitingStart};
  }
  if (!control_enabled) {
    return {Action::None, BlockReason::ControlDisabled};
  }
  if (failsafe_active) {
    return {Action::None, BlockReason::FailsafeActive};
  }
  if (!status_.has_value()) {
    return {Action::None, BlockReason::MissingStatus};
  }
  if (now < status_->received_at) {
    return {Action::None, BlockReason::StaleStatus};
  }
  const double status_age = std::chrono::duration<double>(now - status_->received_at).count();
  if (status_age > config_.status_timeout_sec) {
    return {Action::None, BlockReason::StaleStatus};
  }
  if (status_->remaining < 1.0) {
    return {Action::None, BlockReason::NoRemainingBoost};
  }
  if (status_->is_boosting) {
    return {Action::None, BlockReason::AlreadyBoosting};
  }

  remaining_before_pulse_ = status_->remaining;
  pulse_sent_at_ = now;
  phase_ = Phase::PulseSent;
  return {Action::PublishPulse, BlockReason::None};
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
  start_event_emitted_ = false;
  finish_seen_ = false;
  status_.reset();
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

const char * to_string(const Phase phase) noexcept
{
  switch (phase) {
    case Phase::Disabled:
      return "Disabled";
    case Phase::Armed:
      return "Armed";
    case Phase::PulseSent:
      return "PulseSent";
    case Phase::Confirmed:
      return "Confirmed";
    case Phase::UnconfirmedSpent:
      return "UnconfirmedSpent";
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
    case BlockReason::ControlDisabled:
      return "control disabled";
    case BlockReason::FailsafeActive:
      return "control failsafe active";
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
