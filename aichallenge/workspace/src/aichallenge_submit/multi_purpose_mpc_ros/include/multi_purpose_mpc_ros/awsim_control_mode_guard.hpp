#ifndef MULTI_PURPOSE_MPC_ROS__AWSIM_CONTROL_MODE_GUARD_HPP_
#define MULTI_PURPOSE_MPC_ROS__AWSIM_CONTROL_MODE_GUARD_HPP_

#include <string_view>

namespace multi_purpose_mpc_ros::awsim_control_mode
{

enum class Phase
{
  Disabled,
  Idle,
  AwaitingMotion,
  MotionConfirmed,
  TimedOut,
};

enum class Event
{
  None,
  ReadyRequest,
  StartRequest,
  RetryRequest,
  MotionConfirmed,
  TimedOut,
  Reset,
};

struct Config
{
  bool enabled{false};
  double retry_period_sec{0.2};
  double retry_timeout_sec{5.0};
  double motion_speed_threshold_mps{0.1};
};

struct Decision
{
  Event event{Event::None};
  bool publish_request{false};
  bool suppress_stuck_recovery{false};
};

class LaunchEngagementGuard
{
public:
  explicit LaunchEngagementGuard(Config config);

  Decision on_awsim_state(std::string_view state, double steady_now_sec);
  Decision update(double steady_now_sec, double signed_speed_mps);

  [[nodiscard]] Phase phase() const noexcept;
  [[nodiscard]] bool suppress_stuck_recovery() const noexcept;

private:
  void reset() noexcept;

  Config config_;
  Phase phase_{Phase::Disabled};
  double start_time_sec_{0.0};
  double last_request_time_sec_{0.0};
};

const char * to_string(Phase phase) noexcept;
const char * to_string(Event event) noexcept;

}  // namespace multi_purpose_mpc_ros::awsim_control_mode

#endif  // MULTI_PURPOSE_MPC_ROS__AWSIM_CONTROL_MODE_GUARD_HPP_
