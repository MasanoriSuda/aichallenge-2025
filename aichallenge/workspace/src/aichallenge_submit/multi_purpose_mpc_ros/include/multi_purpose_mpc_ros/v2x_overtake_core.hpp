#ifndef MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
#define MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_

#include <optional>

namespace multi_purpose_mpc_ros::v2x_overtake_core
{

enum class StartWindowStatus
{
  NotConfigured,
  AwaitingStart,
  InvalidElapsed,
  Applied,
  Expired,
};

struct SpeedLimitRequest
{
  double normal_speed_mps{};
  double global_hard_cap_mps{};
  std::optional<double> start_speed_mps;
  double start_window_duration_sec{};
  std::optional<double> elapsed_since_start_sec;
};

struct SpeedLimitResolution
{
  double speed_mps{};
  StartWindowStatus start_window_status{StartWindowStatus::NotConfigured};
};

/// Resolve the vehicle speed ceiling while preserving a global hard cap.
///
/// A configured Start window may temporarily exceed normal_speed_mps, but it
/// can never exceed global_hard_cap_mps. Missing, non-finite, or negative
/// elapsed time falls back to the capped normal speed.
SpeedLimitResolution resolve_effective_speed_limit(const SpeedLimitRequest & request);
const char * to_string(StartWindowStatus status) noexcept;

enum class PassSide : int
{
  Right = -1,
  None = 0,
  Left = 1,
};

enum class SideSelectionReason
{
  Preferred,
  Alternate,
  Locked,
  LockedUnavailable,
  PreferredUnavailable,
  NoFeasibleSide,
  InvalidPreference,
};

struct SideSelectionRequest
{
  PassSide preferred{PassSide::None};
  PassSide locked{PassSide::None};
  bool left_feasible{false};
  bool right_feasible{false};
  bool allow_alternate{true};
};

struct SideSelection
{
  PassSide side{PassSide::None};
  SideSelectionReason reason{SideSelectionReason::NoFeasibleSide};
};

/// Select a feasible pass side without changing sides after one is locked.
SideSelection select_pass_side(const SideSelectionRequest & request) noexcept;

PassSide opposite_side(PassSide side) noexcept;

enum class ContinuityAction
{
  Continue,
  Hold,
  Return,
  Recovery,
};

struct ContinuityRequest
{
  bool solver_recovery_requested{false};
  bool target_position_jump{false};
  bool rear_clear_observed{false};
  bool rear_clear_confirmed{false};
  bool side_vehicle_present{false};
  bool target_seen{false};
  double target_age_sec{};
  double target_hold_sec{};
  bool target_not_ahead{false};
};

/// Decide how an active ShiftOut/Pass phase reacts when behavior no longer requests Overtake.
ContinuityAction resolve_target_continuity(const ContinuityRequest & request);

struct ReacquireRequest
{
  bool enabled{false};
  bool stable_target_id{false};
  bool same_target{false};
  bool same_side{false};
  bool gap_available{false};
  bool execution_allowed{false};
  double return_elapsed_sec{};
  double reacquire_window_sec{};
  double return_progress{};
  double max_return_progress{};
};

/// Allow Return -> Pass only for the same stable target and pass side early in Return.
bool can_reacquire_during_return(const ReacquireRequest & request) noexcept;

struct ForwardDistanceRequest
{
  double accumulated_distance_m{};
  double forward_speed_mps{};
  double delta_sec{};
  double max_observation_gap_sec{};
};

struct ForwardDistanceResolution
{
  double accumulated_distance_m{};
  bool observation_accepted{false};
};

/// Integrate forward distance for one control observation.
///
/// Invalid/rolled-back/late observations do not change the accumulated distance.
/// Configuration errors throw std::invalid_argument.
ForwardDistanceResolution integrate_forward_distance(const ForwardDistanceRequest & request);

enum class RecoveryExitReason
{
  Active,
  DistanceComplete,
  LateralComplete,
  Stalled,
  TimedOut,
  InvalidObservation,
};

struct RecoveryPolicyRequest
{
  double configured_velocity_limit_mps{};
  double elapsed_sec{};
  double traveled_distance_m{};
  double target_distance_m{};
  double lateral_error_m{};
  double lateral_completion_m{};
  double stalled_sec{};
  double stall_timeout_sec{};
  double timeout_sec{};
};

struct RecoveryPolicyResolution
{
  double velocity_limit_mps{};
  RecoveryExitReason exit_reason{RecoveryExitReason::Active};
};

/// Resolve the bounded overtake Recovery policy.
///
/// The returned velocity limit is the configured ceiling and intentionally does not shrink with
/// current vehicle speed. Runtime observation errors fail closed via InvalidObservation.
RecoveryPolicyResolution resolve_recovery_policy(const RecoveryPolicyRequest & request);
const char * to_string(RecoveryExitReason reason) noexcept;

struct SolverCooldownRequest
{
  double now_sec{};
  double current_until_sec{};
  double duration_sec{};
};

/// Arm or extend the solver-failure cooldown without shortening an existing deadline.
double arm_solver_cooldown(const SolverCooldownRequest & request);

/// Return true strictly before a finite solver-failure cooldown deadline.
bool is_solver_cooldown_active(double now_sec, double cooldown_until_sec) noexcept;

}  // namespace multi_purpose_mpc_ros::v2x_overtake_core

#endif  // MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
