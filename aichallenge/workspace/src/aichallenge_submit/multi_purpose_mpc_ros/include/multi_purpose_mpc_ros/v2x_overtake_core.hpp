#ifndef MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
#define MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_

#include <cstddef>
#include <optional>
#include <vector>

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

enum class OvertakeSpeedStage
{
  ShiftOut,
  Pass,
};

struct OvertakeSpeedReferenceRequest
{
  OvertakeSpeedStage stage{OvertakeSpeedStage::ShiftOut};
  double base_reference_speed_mps{};
  double hard_cap_mps{};
  double front_speed_mps{};
  double entry_speed_mps{};
  double shiftout_max_closing_speed_mps{};
};

struct OvertakeSpeedReferenceResolution
{
  double reference_speed_mps{};
  bool front_cap_applied{false};
};

/// Keep closing speed bounded while shifting out, then release the front-speed
/// ceiling in Pass so the original trajectory reference can be used.
OvertakeSpeedReferenceResolution resolve_overtake_speed_reference(
  const OvertakeSpeedReferenceRequest & request);

struct PredictionTimeRequest
{
  double elapsed_sec{};
  double segment_distance_m{};
  double predicted_speed_mps{};
  double minimum_speed_mps{};
  double maximum_time_sec{};
};

/// Advance a path-aligned prediction clock by distance / predicted speed.
double advance_prediction_time(const PredictionTimeRequest & request);

struct CoursePoint
{
  double x_m{};
  double y_m{};
};

struct ForwardCourseProjectionRequest
{
  std::size_t start_index{};
  bool circular{false};
  double origin_x_m{};
  double origin_y_m{};
  double target_x_m{};
  double target_y_m{};
  double target_vx_mps{};
  double target_vy_mps{};
  double lookbehind_distance_m{};
  double lookahead_distance_m{};
  double max_cross_track_distance_m{};
};

struct ForwardCourseProjection
{
  bool valid{false};
  double forward_distance_m{};
  double lateral_m{};
  double along_track_speed_mps{};
  double cross_track_distance_m{};
  std::size_t segment_index{};
};

/// Project ego and a V2X target onto the same bounded section of a reference
/// polyline. The result is path progress, not the target's Cartesian distance
/// in the ego tangent frame. A circular path may wrap once, but the bounded
/// lookahead prevents selecting a spatially close branch far around the lap.
ForwardCourseProjection project_forward_course_progress(
  const std::vector<CoursePoint> & path, const ForwardCourseProjectionRequest & request);

struct PassCompletionRequest
{
  double distance_to_hard_curve_m{};
  double curve_buffer_m{};
  double front_distance_m{};
  double front_speed_mps{};
  double planned_ego_speed_mps{};
  double return_clear_distance_m{};
  double minimum_shift_distance_m{};
  double merge_buffer_m{};
  double minimum_relative_speed_mps{};
};

struct PassCompletionResolution
{
  bool feasible{false};
  double available_distance_m{};
  double required_distance_m{};
  double relative_speed_mps{};
};

/// Estimate whether ego can clear the target before the next hard curve.
PassCompletionResolution resolve_pass_completion(const PassCompletionRequest & request);

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

struct StallWatchdogRequest
{
  bool active{false};
  double speed_mps{};
  double now_sec{};
  double previous_update_sec{};
  double stall_since_sec{};
  double speed_threshold_mps{};
  double timeout_sec{};
  double max_observation_gap_sec{};
};

struct StallWatchdogResolution
{
  double update_sec{};
  double stall_since_sec{};
  double stalled_sec{};
  bool observation_accepted{false};
  bool timed_out{false};
};

/// Update a bounded low-speed stall observation.
///
/// `previous_update_sec` and `stall_since_sec` use NaN to represent an inactive timer. A clock
/// rollback or observation gap restarts the timer from the current observation instead of
/// carrying elapsed time across an unobserved interval.
StallWatchdogResolution update_stall_watchdog(const StallWatchdogRequest & request);
const char * to_string(RecoveryExitReason reason) noexcept;

struct FrontHazardHoldRequest
{
  bool enabled{false};
  bool hazard_observed{false};
  bool target_rear_clear{false};
  double now_sec{};
  double current_until_sec{};
  double hold_sec{};
};

struct FrontHazardHoldResolution
{
  bool active{false};
  double until_sec{};
  double remaining_sec{};
};

/// Keep a recently observed front hazard active across a short V2X geometry dropout.
///
/// A fresh hazard arms or extends the deadline. A positive rear-clear observation releases the
/// hold immediately. Invalid configuration or clock values throw std::invalid_argument.
FrontHazardHoldResolution update_front_hazard_hold(const FrontHazardHoldRequest & request);

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

struct SolverReentryGateRequest
{
  bool arm{false};
  bool blocked{false};
  int consecutive_successes{0};
  bool solver_succeeded{false};
  bool cooldown_active{false};
  int required_successes{1};
};

struct SolverReentryGateResolution
{
  bool blocked{false};
  int consecutive_successes{0};
  bool released{false};
};

/// Require both cooldown expiry and consecutive healthy MPC solves before another overtake.
///
/// Arming always resets the success count. Any failed solve while blocked also resets the count.
/// Invalid counters or a non-positive required_successes value throw std::invalid_argument.
SolverReentryGateResolution update_solver_reentry_gate(
  const SolverReentryGateRequest & request);

struct SolverFallbackSteeringRequest
{
  double current_steering_rad{};
  double max_steering_rad{};
  double steer_rate_radps{};
  double step_sec{};
};

/// Move a solver-fallback steering command toward neutral without exceeding the rate limit.
double rate_limit_solver_fallback_steering_toward_neutral(
  const SolverFallbackSteeringRequest & request);

struct SolverFallbackNeutralizationRequest
{
  int consecutive_failures{0};
  int steering_hold_cycles{0};
  bool force_neutralize{false};
};

/// Select neutral steering recovery after a bounded failure hold window.
///
/// force_neutralize bypasses the hold window. Negative counters throw std::invalid_argument.
bool should_neutralize_solver_fallback_steering(
  const SolverFallbackNeutralizationRequest & request);

}  // namespace multi_purpose_mpc_ros::v2x_overtake_core

#endif  // MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
