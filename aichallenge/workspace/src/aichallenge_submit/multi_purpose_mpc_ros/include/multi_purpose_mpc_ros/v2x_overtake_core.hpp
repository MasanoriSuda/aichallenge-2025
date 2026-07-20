#ifndef MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
#define MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_

#include <cstddef>
#include <limits>
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

struct FollowSpeedLimitRequest
{
  bool enabled{false};
  bool suppressed{false};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  /// 0 keeps the legacy behavior: apply the cap at every detected front distance.
  double activation_distance_m{0.0};
  double front_speed_mps{std::numeric_limits<double>::infinity()};
  double moving_front_speed_threshold_mps{};
  double moving_front_speed_margin_mps{};
  /// Center-to-center distance where a moving front should be speed-matched.
  /// Zero preserves the legacy fixed moving_front_speed_margin_mps behavior.
  double moving_front_target_distance_m{};
  /// Maximum amount to command below the moving front speed while recovering clearance.
  double moving_front_recovery_speed_margin_mps{};
  /// Signed speed-margin gain [(m/s)/m] around moving_front_target_distance_m.
  double moving_front_distance_gain{};
  double slow_front_distance_limit_mps{};
  double slow_front_velocity_cap_mps{};
  double maximum_speed_mps{};
};

struct FollowSpeedLimitResolution
{
  bool active{false};
  bool moving_front{false};
  bool moving_front_clearance_recovery{false};
  double moving_front_speed_margin_mps{};
  double speed_limit_mps{std::numeric_limits<double>::infinity()};
};

/// Apply the generic Follow cap only inside its dedicated distance gate.
///
/// Detection, front-risk, curve and emergency policies remain outside this
/// helper. A zero activation distance preserves the legacy unbounded gate.
/// Moving fronts use a distance-dependent signed speed margin when a target
/// distance and gain are configured. Slow fronts use the smaller of the
/// distance-derived limit and the configured Follow cap.
FollowSpeedLimitResolution resolve_follow_speed_limit(
  const FollowSpeedLimitRequest & request);

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

/// Keep closing speed bounded while the caller selects ShiftOut, then release
/// the front-speed ceiling only after the caller has confirmed a clear Pass.
OvertakeSpeedReferenceResolution resolve_overtake_speed_reference(
  const OvertakeSpeedReferenceRequest & request);

struct ShiftOutCompletionRequest
{
  bool phase_hold_elapsed{false};
  double traveled_distance_m{};
  double required_distance_m{};
  double current_lateral_m{};
  double target_lateral_m{};
  double lateral_tolerance_m{};
  int pass_side_sign{};
};

/// True once the vehicle reaches or crosses the target line toward pass_side_sign.
/// Overshoot on the selected pass side is complete, not an error.
bool has_reached_pass_side_lateral_goal(
  double current_lateral_m, double target_lateral_m,
  double lateral_tolerance_m, int pass_side_sign) noexcept;

/// Enter Pass only after both longitudinal shift distance and directional
/// lateral target completion. Invalid observations never complete ShiftOut.
bool is_shiftout_complete(const ShiftOutCompletionRequest & request) noexcept;

struct OvertakeFrontCapReleaseRequest
{
  bool pass_phase{false};
  bool lateral_complete{false};
  bool target_seen{false};
  double target_longitudinal_m{};
};

/// Release the front-speed cap only when the locked target is observed no
/// longer ahead after lateral ShiftOut has completed.
bool can_release_overtake_front_cap(
  const OvertakeFrontCapReleaseRequest & request) noexcept;

struct PassFrontOverlapExclusionRequest
{
  bool pass_phase{false};
  bool locked_target{false};
  double relative_lateral_m{};
  double required_lateral_clearance_m{};
  bool already_latched{false};
};

/// A locked target that is laterally separated in Pass is a side-by-side
/// vehicle, not a centerline front obstacle. The result stays true after the
/// first clearance until Pass ends so hairpin frame rotation cannot chatter it.
/// Other vehicles remain unchanged.
bool can_exclude_locked_target_from_front_overlap(
  const PassFrontOverlapExclusionRequest & request) noexcept;

struct ActivePassGapHoldRequest
{
  bool pass_phase{false};
  bool lateral_clearance_latched{false};
  bool locked_target_seen{false};
  bool locked_target_position_jump{false};
};

/// Once ShiftOut has completed and the locked target is side-by-side, do not
/// treat a transient gap-width/time failure as a new-pass rejection.
bool can_hold_active_pass_after_gap_loss(const ActivePassGapHoldRequest & request) noexcept;

struct OvertakeLateralPlannerOwnershipRequest
{
  bool explicit_line_enabled{false};
  bool behavior_requests_overtake{false};
  bool line_phase_active{false};
};

/// The explicit ShiftOut/Pass line and the gap planner must not inject lateral
/// references/bounds into the same MPC solve.
bool explicit_overtake_line_owns_lateral_plan(
  const OvertakeLateralPlannerOwnershipRequest & request) noexcept;

struct SideOvertakeEntryRequest
{
  bool continuing_overtake{false};
  double target_longitudinal_m{};
  double rear_tolerance_m{};
};

/// Do not start a new lateral pass for a side vehicle that ego has already
/// passed in common course progress. Existing committed passes remain valid.
bool can_start_side_overtake(const SideOvertakeEntryRequest & request) noexcept;

/// Limit closing speed only while Pass is waiting for lateral-clearance latch.
/// ShiftOut keeps its adaptive limit and a latched Pass is released elsewhere.
double resolve_unlatched_pass_closing_speed(
  double configured_closing_speed_mps, double unlatched_pass_closing_speed_mps,
  bool pass_phase, bool lateral_clearance_latched);

struct OvertakeLineHorizonProgressRequest
{
  bool hold_target{false};
  double phase_traveled_m{};
  double horizon_distance_m{};
  double phase_distance_m{};
};

/// Advance the explicit lateral line inside the prediction horizon using both
/// distance already traveled in the phase and distance ahead of ego. Without
/// phase_traveled_m, the ramp restarts at ego every control cycle and ShiftOut
/// can never converge at the configured distance.
double resolve_overtake_line_horizon_progress(
  const OvertakeLineHorizonProgressRequest & request) noexcept;

struct PassSideLateralGoalRequest
{
  int pass_side_sign{};
  double base_lateral_offset_m{};
  double target_lateral_m{};
  double minimum_separation_m{};
};

/// Place the pass line on the selected side of both the base trajectory and
/// the locked target. This prevents a target that moves toward the same side
/// in a curve from consuming the fixed lateral pass offset.
double resolve_pass_side_lateral_goal(const PassSideLateralGoalRequest & request) noexcept;

struct AdaptiveShiftOutClosingSpeedRequest
{
  double minimum_closing_speed_mps{};
  double maximum_closing_speed_mps{};
  double front_distance_m{};
  double protected_front_distance_m{};
  double remaining_shiftout_distance_m{};
  double ego_speed_mps{};
  double minimum_speed_mps{};
  double minimum_time_sec{};
};

struct AdaptiveShiftOutClosingSpeedResolution
{
  double closing_speed_mps{};
  double remaining_time_sec{};
  double distance_budget_m{};
};

/// Select a ShiftOut closing-speed cap that preserves the protected front
/// distance over the estimated remaining lateral-shift time.
AdaptiveShiftOutClosingSpeedResolution resolve_adaptive_shiftout_closing_speed(
  const AdaptiveShiftOutClosingSpeedRequest & request);

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

struct OvertakeGuardPhaseRequest
{
  bool continuing_overtake{false};
  double entry_min_front_distance_m{};
  double continuation_min_front_distance_m{};
};

struct OvertakeGuardPhaseResolution
{
  double min_front_distance_m{};
  bool require_prepare_distance{true};
};

/// Select the front-distance guard for a new pass or an already active pass.
///
/// The prepare-distance check belongs to pass entry. Once a pass is active,
/// only the continuation front-distance threshold is reused; the geometric
/// gap and lateral reachability checks remain the caller's responsibility.
OvertakeGuardPhaseResolution resolve_overtake_guard_phase(
  const OvertakeGuardPhaseRequest & request);

struct OvertakeCurveContinuationRequest
{
  bool continuation_enabled{false};
  bool inner_soft_curve_enabled{false};
  bool continuing_overtake{false};
  bool soft_curve_forbidden{false};
  bool hard_curve_forbidden{false};
  bool inner_curve_pass{false};
  bool cooldown_active{false};
  bool emergency_brake{false};
};

/// Allow an active locked pass through a soft curve without weakening hard guards.
bool can_continue_overtake_in_soft_curve(
  const OvertakeCurveContinuationRequest & request) noexcept;

struct OuterCurveOvertakeRequest
{
  bool entry_enabled{false};
  bool hard_continuation_enabled{false};
  bool continuing_overtake{false};
  bool soft_curve_forbidden{false};
  bool hard_curve_forbidden{false};
  bool explicit_forbidden_wp{false};
  bool cooldown_active{false};
  bool emergency_brake{false};
  bool gap_available{false};
  bool locked_target_seen{false};
  int pass_side_sign{0};
  int inner_curve_pass_side{0};
};

struct OuterCurveOvertakeResolution
{
  bool entry_allowed{false};
  bool hard_continuation_allowed{false};
};

/// Allow a new pass only on the outside of a soft curve, then keep the same locked
/// outside line through a hard curve while its geometric gap remains available.
/// Explicit forbidden waypoints, inner passes, cooldown and emergency braking are never relaxed.
OuterCurveOvertakeResolution resolve_outer_curve_overtake(
  const OuterCurveOvertakeRequest & request) noexcept;

struct InnerCurveOvertakeRequest
{
  bool entry_enabled{false};
  bool hard_continuation_enabled{false};
  bool continuing_overtake{false};
  bool soft_curve_forbidden{false};
  bool hard_curve_forbidden{false};
  bool explicit_forbidden_wp{false};
  bool cooldown_active{false};
  bool emergency_brake{false};
  bool gap_available{false};
  bool locked_target_seen{false};
  int pass_side_sign{0};
  int inner_curve_pass_side{0};
};

struct InnerCurveOvertakeResolution
{
  bool entry_allowed{false};
  bool hard_continuation_allowed{false};
};

/// Allow a new pass on the inside of a soft curve, then keep the same locked
/// inside line through a hard curve while its geometric gap remains available.
/// A new pass is never started after the hard-curve boundary.
InnerCurveOvertakeResolution resolve_inner_curve_overtake(
  const InnerCurveOvertakeRequest & request) noexcept;

struct ActiveHardCurveContinuationRequest
{
  bool enabled{false};
  bool continuing_overtake{false};
  bool pass_phase{false};
  bool locked_target_seen{false};
  bool lateral_clearance_latched{false};
  bool hard_curve_ahead{false};
  bool explicit_forbidden_wp{false};
  bool cooldown_active{false};
  bool emergency_brake{false};
  PassCompletionRequest completion;
};

struct ActiveHardCurveContinuationResolution
{
  bool allowed{false};
  PassCompletionResolution completion;
};

/// Allow only an already shifted-out Pass to finish before a detected hard boundary.
/// A Pass that already established and latched lateral clearance stays committed even when
/// the conservative distance estimate briefly becomes infeasible at the boundary.
ActiveHardCurveContinuationResolution resolve_active_hard_curve_continuation(
  const ActiveHardCurveContinuationRequest & request);

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

struct LowSpeedPassSideCandidate
{
  bool feasible{false};
  double target_lateral_m{};
  double width_m{};
};

struct LowSpeedPassSideRequest
{
  double current_lateral_m{};
  LowSpeedPassSideCandidate left;
  LowSpeedPassSideCandidate right;
};

/// Select the feasible stopped-vehicle pass side requiring the smaller lateral
/// transition. Width is only a tie-breaker after the minimum-width checks have
/// already declared both candidates feasible.
PassSide select_reachable_low_speed_pass_side(
  const LowSpeedPassSideRequest & request) noexcept;

/// Return true only after ego has physically entered the selected pass
/// corridor. Until then the corridor must remain a target, not an immediately
/// active hard state bound.
bool has_entered_low_speed_pass_corridor(
  double current_lateral_m, double lower_m, double upper_m,
  double tolerance_m = 0.05) noexcept;

/// Limit speed during the lateral shift, then restore the configured pass
/// speed after ego has entered the selected corridor.
double resolve_low_speed_pass_velocity(
  double pass_velocity_mps, double shift_velocity_mps, bool corridor_entered);

struct LowSpeedShiftSteeringRequest
{
  double current_lateral_m{};
  double current_heading_error_rad{};
  double target_lateral_m{};
  double reference_curvature_radpm{};
  double wheelbase_m{};
  double max_steering_rad{};
  double lateral_gain{};
  double heading_gain{};
};

/// Compute a bounded steering target for the short, low-speed transition into
/// a stopped-vehicle pass corridor. The feedback follows the same lateral and
/// heading-error signs as the spatial bicycle model.
double resolve_low_speed_shift_steering(
  const LowSpeedShiftSteeringRequest & request);

bool is_low_speed_shift_complete(
  double current_lateral_m, double current_heading_error_rad,
  double target_lateral_m, double lateral_tolerance_m,
  double heading_tolerance_rad) noexcept;

/// Release the direct low-speed shift controller only after its target pose is
/// settled and the complete stopped-vehicle pack has cleared the ego vehicle.
bool should_release_low_speed_shift_control(
  bool pose_settled, bool has_front_vehicle, bool has_side_vehicle,
  bool has_clearance_vehicle, double clear_duration_sec,
  double required_clear_duration_sec) noexcept;

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
  bool active_execution_latched{false};
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
  bool target_observed_safe{false};
};

struct FrontHazardHoldResolution
{
  bool active{false};
  double until_sec{};
  double remaining_sec{};
};

/// Keep a recently observed front hazard active across a short V2X geometry dropout.
///
/// A fresh hazard arms or extends the deadline. A positive rear-clear observation, or a fresh
/// observation that confirms the moving target is no longer closing, releases the hold
/// immediately. Invalid configuration or clock values throw std::invalid_argument.
FrontHazardHoldResolution update_front_hazard_hold(const FrontHazardHoldRequest & request);

enum class FrontDangerAction
{
  None,
  RelativeSpeedLimit,
  SafetyBrake,
};

struct FrontDangerActionRequest
{
  bool inside_stopping_distance{false};
  bool emergency_brake{false};
  double front_speed_mps{};
  double moving_front_speed_threshold_mps{};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  /// Moving-front center-to-center distance that requires a full stop.
  /// Zero disables this additional hard-clearance check.
  double moving_front_hard_distance_m{};
};

/// Resolve a close-front geometry observation without turning every moving-front headway event
/// into a full stop. Emergency risk and stopped/slow fronts remain fail-closed; a moving front
/// uses the caller's relative-speed limit path instead. A moving front inside
/// the configured hard center distance remains fail-closed even after the
/// relative speed has nearly matched.
FrontDangerAction resolve_front_danger_action(const FrontDangerActionRequest & request);
const char * to_string(FrontDangerAction action) noexcept;

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
