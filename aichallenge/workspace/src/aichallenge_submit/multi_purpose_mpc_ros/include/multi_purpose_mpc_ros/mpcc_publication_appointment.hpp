#pragma once

#include <rclcpp/create_timer.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_publication_appointment {

enum class ClockStatus { Current, Regressed, Stalled };

/// Observe ROS progress using the existing steady-clock safety timeout. Equal
/// ROS samples never renew this clock's progress time.
class ClockWatchdog {
public:
  using SteadyClock = std::chrono::steady_clock;
  ClockWatchdog(std::int64_t ros_ns, SteadyClock::time_point steady)
  : last_ros_ns_(ros_ns), last_progress_(steady) {}

  ClockStatus observe(std::int64_t ros_ns, SteadyClock::time_point steady,
                      std::chrono::duration<double> maximum_stall) {
    const bool regressed = ros_ns < last_ros_ns_;
    if (ros_ns != last_ros_ns_) {
      last_ros_ns_ = ros_ns;
      last_progress_ = steady;
    }
    if (regressed) return ClockStatus::Regressed;
    return steady - last_progress_ > maximum_stall ? ClockStatus::Stalled : ClockStatus::Current;
  }

private:
  std::int64_t last_ros_ns_{};
  SteadyClock::time_point last_progress_;
};

/// A one-shot wakeup at an immutable ROS appointment. This grants no command
/// authority and deliberately does not extend a late appointment's deadline.
class Alarm {
public:
  using Callback = std::function<void(std::int64_t)>;

  Alarm(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers,
        rclcpp::Clock::SharedPtr clock, Callback callback)
  : base_(std::move(base)), timers_(std::move(timers)), clock_(std::move(clock)),
    callback_(std::move(callback)) {}

  ~Alarm() { cancel(); }
  Alarm(const Alarm &) = delete;
  Alarm &operator=(const Alarm &) = delete;

  void cancel() {
    generation_.reset();
    if (timer_) timer_->cancel();
    timer_.reset();
  }

  void arm(std::int64_t appointment_ns) {
    cancel();
    const auto current_ns = clock_->now().nanoseconds();
    if (appointment_ns < 0 || current_ns < 0)
      throw std::invalid_argument("publication appointment needs a nonnegative clock");
    const auto delay_ns = appointment_ns > current_ns ? appointment_ns - current_ns : 0;
    generation_ = std::make_shared<const char>(0);
    const auto generation = generation_;
    timer_ = rclcpp::create_timer(base_, timers_, clock_,
      rclcpp::Duration{std::chrono::nanoseconds{delay_ns}},
      [this, generation, appointment_ns]() {
        if (generation_ != generation) return;
        // Keep the firing timer alive when the callback rearms this alarm.
        const auto fired = timer_;
        fired->cancel();
        if (clock_->now().nanoseconds() < appointment_ns) {
          // ROS timer internals can rephase after a backwards clock jump.
          // Retain the original appointment instead of accepting an early wake.
          arm(appointment_ns);
          return;
        }
        callback_(appointment_ns);
      });
  }

private:
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_;
  rclcpp::Clock::SharedPtr clock_;
  Callback callback_;
  std::shared_ptr<const char> generation_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace multi_purpose_mpc_ros::mpcc_publication_appointment
