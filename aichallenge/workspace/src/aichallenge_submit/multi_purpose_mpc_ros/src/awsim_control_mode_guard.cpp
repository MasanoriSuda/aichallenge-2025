#include "multi_purpose_mpc_ros/awsim_control_mode_guard.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace multi_purpose_mpc_ros::awsim_control_mode
{
namespace
{

constexpr double kTimeEpsilonSec = 1e-9;

std::string normalize(const std::string_view value)
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

LaunchEngagementGuard::LaunchEngagementGuard(Config config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.retry_period_sec) || config_.retry_period_sec <= 0.0) {
    throw std::invalid_argument("AWSIM control mode retry period must be finite and positive");
  }
  if (!std::isfinite(config_.retry_timeout_sec) || config_.retry_timeout_sec <= 0.0) {
    throw std::invalid_argument("AWSIM control mode retry timeout must be finite and positive");
  }
  if (
    !std::isfinite(config_.motion_speed_threshold_mps) ||
    config_.motion_speed_threshold_mps <= 0.0)
  {
    throw std::invalid_argument(
            "AWSIM control mode motion threshold must be finite and positive");
  }
  phase_ = config_.enabled ? Phase::Idle : Phase::Disabled;
}

Decision LaunchEngagementGuard::on_awsim_state(
  const std::string_view state, const double steady_now_sec)
{
  if (phase_ == Phase::Disabled || !std::isfinite(steady_now_sec)) {
    return {};
  }

  const auto normalized = normalize(state);
  if (normalized == "ready") {
    phase_ = Phase::AwaitingMotion;
    start_time_sec_ = steady_now_sec;
    last_request_time_sec_ = steady_now_sec;
    return Decision{Event::ReadyRequest, true, true};
  }
  if (normalized == "start") {
    phase_ = Phase::AwaitingMotion;
    start_time_sec_ = steady_now_sec;
    last_request_time_sec_ = steady_now_sec;
    return Decision{Event::StartRequest, true, true};
  }
  if (
    normalized == "spawned" || normalized == "grounded" ||
    normalized == "finish")
  {
    const bool changed = phase_ != Phase::Idle;
    reset();
    return Decision{changed ? Event::Reset : Event::None, false, false};
  }
  return Decision{Event::None, false, suppress_stuck_recovery()};
}

Decision LaunchEngagementGuard::update(
  const double steady_now_sec, const double signed_speed_mps)
{
  if (phase_ != Phase::AwaitingMotion) {
    return Decision{Event::None, false, suppress_stuck_recovery()};
  }
  if (!std::isfinite(steady_now_sec) || !std::isfinite(signed_speed_mps)) {
    phase_ = Phase::TimedOut;
    return Decision{Event::TimedOut, false, false};
  }
  if (std::abs(signed_speed_mps) >= config_.motion_speed_threshold_mps) {
    phase_ = Phase::MotionConfirmed;
    return Decision{Event::MotionConfirmed, false, false};
  }
  if (
    steady_now_sec < start_time_sec_ ||
    steady_now_sec - start_time_sec_ + kTimeEpsilonSec >= config_.retry_timeout_sec)
  {
    phase_ = Phase::TimedOut;
    return Decision{Event::TimedOut, false, false};
  }
  if (
    steady_now_sec < last_request_time_sec_ ||
    steady_now_sec - last_request_time_sec_ + kTimeEpsilonSec >= config_.retry_period_sec)
  {
    last_request_time_sec_ = steady_now_sec;
    return Decision{Event::RetryRequest, true, true};
  }
  return Decision{Event::None, false, true};
}

Phase LaunchEngagementGuard::phase() const noexcept
{
  return phase_;
}

bool LaunchEngagementGuard::suppress_stuck_recovery() const noexcept
{
  return phase_ == Phase::AwaitingMotion;
}

void LaunchEngagementGuard::reset() noexcept
{
  phase_ = config_.enabled ? Phase::Idle : Phase::Disabled;
  start_time_sec_ = 0.0;
  last_request_time_sec_ = 0.0;
}

const char * to_string(const Phase phase) noexcept
{
  switch (phase) {
    case Phase::Disabled:
      return "Disabled";
    case Phase::Idle:
      return "Idle";
    case Phase::AwaitingMotion:
      return "AwaitingMotion";
    case Phase::MotionConfirmed:
      return "MotionConfirmed";
    case Phase::TimedOut:
      return "TimedOut";
  }
  return "Unknown";
}

const char * to_string(const Event event) noexcept
{
  switch (event) {
    case Event::None:
      return "None";
    case Event::ReadyRequest:
      return "ReadyRequest";
    case Event::StartRequest:
      return "StartRequest";
    case Event::RetryRequest:
      return "RetryRequest";
    case Event::MotionConfirmed:
      return "MotionConfirmed";
    case Event::TimedOut:
      return "TimedOut";
    case Event::Reset:
      return "Reset";
  }
  return "Unknown";
}

}  // namespace multi_purpose_mpc_ros::awsim_control_mode
