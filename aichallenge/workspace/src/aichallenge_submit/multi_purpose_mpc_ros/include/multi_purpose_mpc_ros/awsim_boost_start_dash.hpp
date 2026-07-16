#ifndef MULTI_PURPOSE_MPC_ROS__AWSIM_BOOST_START_DASH_HPP_
#define MULTI_PURPOSE_MPC_ROS__AWSIM_BOOST_START_DASH_HPP_

#include <array>
#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace multi_purpose_mpc_ros::awsim_boost
{

inline constexpr std::array<float, 2> kCommandPulseValues{1.0F, 0.0F};

enum class Mode
{
  Disabled,
  StartOnce,
};

enum class Trigger
{
  AwsimStart,
  FirstForwardMotion,
};

enum class Phase
{
  Disabled,
  Armed,
  AwaitingMotion,
  MotionDetected,
  PulseSent,
  Confirmed,
  UnconfirmedSpent,
  LaunchExpiredSpent,
};

enum class Action
{
  None,
  PublishPulse,
};

// Race-session edges are reported independently of whether boost output is enabled. This lets
// other controller features use the authoritative AWSIM Start boundary without enabling boost.
enum class StateEvent
{
  None,
  StartEntered,
  Finished,
  NewSession,
};

enum class BlockReason
{
  None,
  Disabled,
  AwaitingStart,
  AwaitingReady,
  AwaitingMotion,
  ControlDisabled,
  CommandNotPublished,
  FailsafeActive,
  SafetyBrakeActive,
  SolverFallbackActive,
  ReverseOrRecoveryActive,
  InvalidForwardSpeed,
  MotionTriggerTimedOut,
  MotionTriggerSpeedExceeded,
  MissingStatus,
  StaleStatus,
  NoRemainingBoost,
  AlreadyBoosting,
  AlreadySpent,
  ConfirmationTimedOut,
};

struct Config
{
  bool enabled{false};
  bool domain_enabled_applied{false};
  int domain_enabled_domain{-1};
  Mode mode{Mode::Disabled};
  Trigger trigger{Trigger::AwsimStart};
  double motion_speed_threshold_mps{0.1};
  double max_trigger_speed_mps{1.0};
  double motion_trigger_timeout_sec{0.5};
  double status_timeout_sec{0.5};
  double confirmation_timeout_sec{2.0};
};

struct EnabledResolution
{
  bool enabled{false};
  bool domain_override_applied{false};
  int domain_id{-1};
};

struct Evaluation
{
  Action action{Action::None};
  BlockReason reason{BlockReason::None};
  bool motion_detected_now{false};
  double motion_elapsed_sec{0.0};
};

struct TriggerContext
{
  bool control_enabled{false};
  bool normal_command_published{false};
  bool failsafe_active{false};
  bool v2x_safety_brake_active{false};
  bool solver_fallback_active{false};
  bool reverse_or_recovery_active{false};
  double forward_speed_mps{0.0};
};

class StartDashGuard
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit StartDashGuard(Config config);

  StateEvent on_awsim_state(std::string_view state);
  bool on_awsim_status(const std::vector<float> & data, TimePoint received_at);
  Evaluation evaluate(const TriggerContext & context, TimePoint now);
  Evaluation evaluate(bool control_enabled, bool failsafe_active, TimePoint now);

  [[nodiscard]] Phase phase() const noexcept;
  [[nodiscard]] bool has_valid_status() const noexcept;
  [[nodiscard]] std::optional<double> remaining() const noexcept;
  [[nodiscard]] std::optional<bool> is_boosting() const noexcept;

private:
  struct Status
  {
    double remaining{};
    bool is_boosting{};
    TimePoint received_at;
  };

  void rearm_for_new_session();
  void update_confirmation_from_status();

  Config config_;
  Phase phase_{Phase::Disabled};
  bool start_seen_{false};
  bool ready_seen_{false};
  bool start_event_emitted_{false};
  bool finish_seen_{false};
  std::optional<Status> status_;
  std::optional<TimePoint> motion_detected_at_;
  std::optional<double> remaining_before_pulse_;
  std::optional<TimePoint> pulse_sent_at_;
};

Mode parse_mode(std::string_view value);
Trigger parse_trigger(std::string_view value);
EnabledResolution resolve_enabled(
  bool default_enabled, const std::map<int, bool> & domain_enabled,
  std::optional<int> ros_domain_id) noexcept;
const char * to_string(Mode mode) noexcept;
const char * to_string(Trigger trigger) noexcept;
const char * to_string(Phase phase) noexcept;
const char * to_string(StateEvent event) noexcept;
const char * to_string(BlockReason reason) noexcept;

}  // namespace multi_purpose_mpc_ros::awsim_boost

#endif  // MULTI_PURPOSE_MPC_ROS__AWSIM_BOOST_START_DASH_HPP_
