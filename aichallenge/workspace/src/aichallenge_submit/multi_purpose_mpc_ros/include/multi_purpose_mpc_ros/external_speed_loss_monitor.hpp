#ifndef MULTI_PURPOSE_MPC_ROS__EXTERNAL_SPEED_LOSS_MONITOR_HPP_
#define MULTI_PURPOSE_MPC_ROS__EXTERNAL_SPEED_LOSS_MONITOR_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

namespace multi_purpose_mpc_ros::external_speed_loss
{

struct Sample
{
  double timestamp_sec{};
  double speed_mps{};
};

struct Event
{
  double previous_speed_mps{};
  double current_speed_mps{};
  double interval_sec{};
  double speed_loss_mps{};
  double observed_acceleration_mps2{};
  double reportable_loss_threshold_mps{};
};

/// Change-only observer for a speed loss that is too large to be explained by
/// the configured normal braking envelope. It never changes control output.
class Monitor
{
public:
  std::optional<Event> update(const Sample & sample, double minimum_acceleration_mps2)
  {
    if (!std::isfinite(sample.timestamp_sec) || !std::isfinite(sample.speed_mps)) {
      previous_.reset();
      return std::nullopt;
    }

    const auto previous = previous_;
    previous_ = sample;
    if (!previous.has_value()) {
      return std::nullopt;
    }

    const double interval_sec = sample.timestamp_sec - previous->timestamp_sec;
    if (
      !std::isfinite(interval_sec) || interval_sec <= 0.0 ||
      interval_sec > maximum_correlated_interval_sec)
    {
      return std::nullopt;
    }

    const double speed_loss_mps = previous->speed_mps - sample.speed_mps;
    const double configured_braking_mps2 =
      std::isfinite(minimum_acceleration_mps2) ?
      std::abs(std::min(0.0, minimum_acceleration_mps2)) : 0.0;
    const double reportable_loss_threshold_mps = std::max(
      minimum_reportable_speed_loss_mps,
      control_envelope_multiplier * configured_braking_mps2 * interval_sec);
    if (speed_loss_mps < reportable_loss_threshold_mps) {
      return std::nullopt;
    }

    return Event{
      previous->speed_mps,
      sample.speed_mps,
      interval_sec,
      speed_loss_mps,
      (sample.speed_mps - previous->speed_mps) / interval_sec,
      reportable_loss_threshold_mps};
  }

  void reset() noexcept
  {
    previous_.reset();
  }

private:
  // At the 40 Hz controller rate this is already far beyond a normal
  // configured-brake speed step, while remaining insensitive to odometry noise.
  static constexpr double minimum_reportable_speed_loss_mps{1.0};
  // A diagnostic tolerance, not a control constraint. It separates an
  // external impulse/penalty from the commanded braking envelope.
  static constexpr double control_envelope_multiplier{2.0};
  // Larger gaps do not establish a causal pair and are rebased silently.
  static constexpr double maximum_correlated_interval_sec{0.25};

  std::optional<Sample> previous_;
};

}  // namespace multi_purpose_mpc_ros::external_speed_loss

#endif  // MULTI_PURPOSE_MPC_ROS__EXTERNAL_SPEED_LOSS_MONITOR_HPP_
