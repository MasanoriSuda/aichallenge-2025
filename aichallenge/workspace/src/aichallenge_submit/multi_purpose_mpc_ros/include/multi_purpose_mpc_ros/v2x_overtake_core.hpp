#ifndef MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
#define MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_

#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::v2x_overtake_core
{

/// Treat a small negative receipt age as fresh when callback execution races
/// with the control-cycle ROS-time snapshot. Larger future ages fail closed.
bool is_v2x_receipt_age_fresh(
  double age_sec, double timeout_sec, double future_tolerance_sec) noexcept;

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

struct GenericFollowCapOwnershipRequest
{
  bool shiftout_active{false};
  bool pass_active{false};
  bool front_matches_locked_target{false};
};

/// An active ShiftOut/Pass owns the generic front-speed cap only for its locked target.
/// Front-risk, deceleration and emergency limits remain separate policies.
bool should_apply_generic_follow_cap(
  const GenericFollowCapOwnershipRequest & request) noexcept;

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

struct StartGridBreakoutSpeedReferenceRequest
{
  bool validated_breakout{false};
  double base_reference_speed_mps{};
  double hard_cap_mps{};
  double front_speed_mps{};
  double entry_speed_mps{};
  double shiftout_max_closing_speed_mps{};
};

/// A collision-inflated, executable start-grid corridor owns longitudinal separation from entry.
/// Before validation, retain the normal ShiftOut front cap. Once validated, expose the full race
/// reference immediately; the caller's MPC acceleration and global/domain caps remain authoritative.
OvertakeSpeedReferenceResolution resolve_start_grid_breakout_speed_reference(
  const StartGridBreakoutSpeedReferenceRequest & request);

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
  bool active_execution_phase{false};
  bool lateral_complete{false};
  bool execution_horizon_unconstrained{false};
  bool lateral_separation_clear{false};
  bool lateral_separation_release_active{false};
  bool lateral_separation_above_reapply_threshold{false};
  bool constrained_horizon_release_allowed{false};
  bool committed_pass_speed_hold_allowed{false};
  bool target_seen{false};
  double target_longitudinal_m{};
};

/// During committed ShiftOut/Pass, release the front-speed cap only after the
/// pass-side lateral goal is complete and its execution horizon is not limited
/// by lateral acceleration or wall constraints. Physical lateral clearance uses
/// caller-provided release/reapply hysteresis. A caller may accept a
/// constrained-but-physically-feasible Pass horizon: initial release then
/// requires full physical lateral clearance, while an existing release may use
/// the lower reapply threshold. Once an existing release belongs to a body-clear
/// committed Pass, the caller may also hold it across a transient line-goal or
/// execution-horizon feasibility error. The caller-provided hold permission
/// must retain physical clearance, observation-continuity and contact guards.
/// A target already behind may release the cap, but still requires the completed
/// lateral path unless the committed hold is active.
bool can_release_overtake_front_cap(
  const OvertakeFrontCapReleaseRequest & request) noexcept;

struct CommittedPassSpeedFloorRequest
{
  bool enabled{false};
  bool pass_phase{false};
  bool lateral_complete{false};
  bool front_cap_released{false};
  bool lateral_exclusion_latched{false};
  bool lateral_separation_above_reapply_threshold{false};
  bool target_seen{false};
  bool execution_path_physically_feasible{false};
  bool actual_wall_contact{false};
  double target_speed_mps{};
  double slow_target_max_speed_mps{};
  double configured_min_speed_mps{};
};

/// A reference-only velocity floor may help finish a physically committed Pass
/// around a stopped/very-slow target. The caller must still clamp the result to
/// all MPC hard speed bounds; this policy never overrides a safety limit.
bool should_apply_committed_pass_speed_floor(
  const CommittedPassSpeedFloorRequest & request) noexcept;

enum class CommittedPassFrontCapTransitionReason
{
  None,
  ConstrainedFeasiblePassHorizonAccepted,
  LateralGoalAndExecutionHorizonClear,
  TargetNoLongerAhead,
  LockedTargetUnavailable,
  LateralGoalIncomplete,
  ExecutionHorizonConstrained,
  LateralClearanceBelowReapplyThreshold,
};

struct CommittedPassPolicyRequest
{
  bool preserve_validated_breakout_line{false};
  bool shiftout_phase{false};
  bool pass_phase{false};
  bool lateral_complete{false};
  bool execution_horizon_unconstrained{false};
  bool execution_path_physically_feasible{false};
  bool actual_wall_contact{false};
  bool lateral_exclusion_latched{false};
  bool prior_front_cap_release_active{false};
  bool lateral_separation_clear{false};
  bool lateral_separation_above_reapply_threshold{false};
  bool locked_target_body_lateral_clear{false};
  bool locked_target_position_jump{false};
  bool target_seen{false};
  double target_longitudinal_m{};
  bool committed_pass_speed_floor_enabled{false};
  double target_speed_mps{};
  double slow_target_max_speed_mps{};
  double committed_pass_min_speed_mps{};
};

struct CommittedPassPolicyResolution
{
  bool active_execution{false};
  bool constrained_horizon_release_allowed{false};
  bool committed_pass_speed_hold_allowed{false};
  bool front_cap_release_ready{false};
  bool front_cap_state_update_required{false};
  bool committed_pass_speed_hold_active{false};
  bool constrained_horizon_front_cap_release_active{false};
  bool committed_pass_speed_floor_active{false};
  CommittedPassFrontCapTransitionReason transition_reason{
    CommittedPassFrontCapTransitionReason::None};
};

/// Resolve all longitudinal speed ownership decisions for a committed ShiftOut/Pass.
/// This composes the existing front-cap and speed-floor policies without changing
/// their conditions. The caller remains responsible for state updates, logging and
/// clamping the resulting speed floor to hard limits.
CommittedPassPolicyResolution resolve_committed_pass_policy(
  const CommittedPassPolicyRequest & request) noexcept;

const char * to_string(CommittedPassFrontCapTransitionReason reason) noexcept;

struct PassFrontOverlapExclusionRequest
{
  bool active_execution_phase{false};
  bool locked_target{false};
  double relative_lateral_m{};
  double required_lateral_clearance_m{};
  bool already_latched{false};
};

/// A locked target that is laterally separated during committed ShiftOut/Pass is
/// a side-by-side vehicle, not a centerline front obstacle. The caller may keep
/// the result latched during Pass so hairpin frame rotation cannot chatter it.
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

/// Once ShiftOut has completed and the locked target is side-by-side, keep the
/// committed Pass across entry-gap/reachability re-evaluation. The caller is
/// responsible for retaining hard execution, target-continuity and wall gates.
bool can_hold_active_pass_after_gap_loss(const ActivePassGapHoldRequest & request) noexcept;

struct ValidatedStartGridBreakoutContinuityRequest
{
  bool continuing_breakout{false};
  bool active_line{false};
  bool target_matches{false};
  bool locked_target_seen{false};
  bool locked_target_position_jump{false};
  bool explicit_forbidden_wp{false};
};

/// A start-grid side corridor is fully validated before the line is latched.
/// Keep that same line through later gap-width/reachability re-evaluation so
/// the maneuver cannot fall back behind the grid target mid-pass. A changed or
/// discontinuous target and an explicit forbidden waypoint still cancel it.
bool can_hold_validated_start_grid_breakout(
  const ValidatedStartGridBreakoutContinuityRequest & request) noexcept;

/// Curve-side classification is required for both soft and hard curvature
/// zones. Explicit forbidden waypoints remain unconditionally blocked.
bool should_resolve_curve_pass_side(
  bool soft_curve_forbidden, bool hard_curve_forbidden,
  bool explicit_forbidden_wp) noexcept;

struct ActiveLineGapLossHoldRequest
{
  bool enabled{false};
  bool active_line{false};
  bool locked_target_seen{false};
  bool locked_target_position_jump{false};
  bool transient_gap_failure{false};
  bool explicit_forbidden_wp{false};
  bool cooldown_active{false};
  bool emergency_brake{false};
  double now_sec{};
  double last_valid_gap_sec{};
  double hold_sec{};
};

struct ActiveLineGapLossHoldResolution
{
  bool active{false};
  double remaining_sec{0.0};
};

/// Keep a locked ShiftOut/Pass line for a bounded time after only a transient
/// gap-width/time/reachability or live execution-corridor failure. Callers use
/// separate last-valid timestamps when those signals have independent validity.
/// Hard safety and target-continuity failures are never held, and a hold never
/// extends its own deadline.
ActiveLineGapLossHoldResolution resolve_active_line_gap_loss_hold(
  const ActiveLineGapLossHoldRequest & request) noexcept;

struct OvertakeLateralPlannerOwnershipRequest
{
  bool explicit_line_enabled{false};
  bool behavior_requests_overtake{false};
  bool line_phase_active{false};
};

/// The explicit ShiftOut/Pass line exclusively owns the lateral reference.
/// Obstacle-aware corridor bounds may still be supplied by the gap planner.
bool explicit_overtake_line_owns_lateral_plan(
  const OvertakeLateralPlannerOwnershipRequest & request) noexcept;

struct GapPlannerStateBoundsRequest
{
  bool explicit_line_owns_plan{false};
};

/// A selected obstacle-free interval is not a continuous reachable hard state
/// bound for an explicit ShiftOut/Pass line. Keep it as a feasibility guard.
bool should_apply_gap_planner_state_bounds(
  const GapPlannerStateBoundsRequest & request) noexcept;

struct LiveExecutionCorridorBlockRequest
{
  bool raw_corridor_blocked{false};
  bool pass_phase{false};
  bool lateral_clearance_latched{false};
};

/// A live planner dropout can still abort ShiftOut and an uncommitted Pass.
/// Once Pass has established lateral separation, keep the explicit line and
/// let target intrusion, wall, emergency, continuity, and solver guards own
/// cancellation instead of re-applying entry-corridor geometry.
bool should_block_live_execution_corridor(
  const LiveExecutionCorridorBlockRequest & request) noexcept;

struct GapPlannerNoGapVelocityLimitRequest
{
  bool follow_behavior{false};
  bool follow_limit_enabled{false};
  bool overtake_fallback_target{false};
  bool committed_execution_corridor_bypass{false};
  bool transient_execution_corridor_hold{false};
};

/// Keep the generic no-gap speed limit for normal planner ownership, including
/// an explicitly enabled Follow policy. A laterally committed Pass that treats
/// live-corridor loss as diagnostic-only, or an active line retained by the
/// bounded transient-loss hold, must not retain only the planner's no-gap
/// longitudinal limit.
bool should_apply_gap_planner_no_gap_velocity_limit(
  const GapPlannerNoGapVelocityLimitRequest & request) noexcept;

struct LockedTargetPassSideIntrusionRequest
{
  bool active_line{false};
  int pass_side_sign{};
  /// Pass has already established the configured ego/target lateral separation.
  bool lateral_clearance_latched{false};
  bool target_seen{false};
  bool target_position_jump{false};
  double target_longitudinal_m{};
  /// Apply lateral-ordering cancellation only when the ahead target is within
  /// this longitudinal distance. Positive infinity preserves the legacy policy.
  double maximum_guard_longitudinal_m{std::numeric_limits<double>::infinity()};
  /// Common-course target lateral position minus ego lateral position.
  double target_relative_lateral_m{};
  /// Required signed ordering margin between ego and the target.
  double ordering_margin_m{};
};

/// Before lateral separation is established and the target is longitudinally
/// close, a valid pass has ego outside the target on the selected side. Detect
/// when that target reaches or crosses the line so ShiftOut cannot keep
/// steering ego into it. A farther target can be crossed laterally while ego
/// remains behind; inflated-corridor and reachability checks own that entry.
/// Once Pass has latched separation, common-course lateral ordering can rotate
/// through a hairpin and is no longer an authoritative cancellation signal.
bool locked_target_intrudes_pass_side(
  const LockedTargetPassSideIntrusionRequest & request) noexcept;

struct SideOvertakeEntryRequest
{
  bool continuing_overtake{false};
  double target_longitudinal_m{};
  double rear_tolerance_m{};
};

/// Do not start a new lateral pass for a side vehicle that ego has already
/// passed in common course progress. Existing committed passes remain valid.
bool can_start_side_overtake(const SideOvertakeEntryRequest & request) noexcept;

/// A side-only target that is already behind ego must not turn an otherwise
/// clear road into Follow. Invalid observations remain conservative.
bool side_only_target_requires_follow(
  double target_longitudinal_m, double rear_tolerance_m) noexcept;

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

struct OvertakeLineHeadingReferenceRequest
{
  double previous_lateral_m{};
  double current_lateral_m{};
  double delta_s_m{};
  double base_curvature_radpm{};
};

/// Convert a lateral offset profile d(s) into a heading-error reference. This
/// keeps the MPC e_y and e_psi references geometrically consistent while the
/// explicit line shifts across the base trajectory.
double resolve_overtake_line_heading_reference(
  const OvertakeLineHeadingReferenceRequest & request) noexcept;

/// Fail closed when the current vehicle footprint cannot be validated against
/// the static wall map during an executing ShiftOut or Pass.
///
/// wall_geometry_available is explicit so deployments without the optional
/// static grid preserve their existing behavior.
bool should_abort_active_overtake_for_static_wall(
  bool active_execution_phase, bool wall_geometry_available,
  bool sample_valid, bool sample_out_of_map, bool sample_has_contact) noexcept;

/// A static-wall clamp may move a target after the normal lateral-acceleration
/// limiter has run. Abort the executing line when that adjusted target again
/// exceeds the configured acceleration limit instead of publishing an
/// unrealizable heading jump.
bool static_wall_clamp_requires_overtake_recovery(
  bool active_execution_phase, bool static_target_adjusted,
  double required_lateral_accel_mps2, double maximum_lateral_accel_mps2) noexcept;

struct PassSideLateralGoalRequest
{
  int pass_side_sign{};
  double base_lateral_offset_m{};
  double target_lateral_m{};
  double minimum_separation_m{};
  /// A validated pass corridor latches its center so a moving target cannot drag the line.
  std::optional<double> fixed_lateral_goal_m;
};

/// Use a finite fixed goal when supplied. Otherwise place the pass line on the selected side of
/// both the base trajectory and locked target. The fixed form is intended for the center of a
/// validated, vehicle-inflated pass corridor.
double resolve_pass_side_lateral_goal(const PassSideLateralGoalRequest & request) noexcept;

struct FeasiblePassSideLateralGoalRequest
{
  int pass_side_sign{};
  double preferred_goal_m{};
  double target_lateral_m{};
  double minimum_separation_m{};
  double feasible_lower_bound_m{};
  double feasible_upper_bound_m{};
  bool enforce_target_separation{false};
};

struct FeasiblePassSideLateralGoalResolution
{
  double goal_m{};
  bool target_separation_feasible{false};
};

/// Intersect the current wall-feasible lateral interval with the selected-side
/// minimum target separation. If the intersection is empty, preserve the wall
/// interval and report the target separation as infeasible so speed/collision
/// protection can remain active.
FeasiblePassSideLateralGoalResolution resolve_feasible_pass_side_lateral_goal(
  const FeasiblePassSideLateralGoalRequest & request) noexcept;

struct PassCorridorCenterRequest
{
  bool active{};
  double lower_bound_m{};
  double upper_bound_m{};
};

/// Return the center of a valid ego-center corridor. The supplied bounds are expected to have
/// already applied obstacle inflation and wall clearance.
std::optional<double> resolve_pass_corridor_center(
  const PassCorridorCenterRequest & request) noexcept;

struct CompletedPassReturnRequest
{
  bool pass_phase{false};
  bool lateral_separation_latched{false};
  bool target_seen{false};
  bool physical_path_blocked{false};
  double target_longitudinal_m{};
  double rear_clear_distance_m{};
};

/// A committed Pass whose locked target is already behind should merge back
/// instead of entering a margin-only Recovery. Physical contact or a physically
/// infeasible path remains a Recovery condition.
bool should_return_completed_pass_before_margin_recovery(
  const CompletedPassReturnRequest & request) noexcept;

struct ReturnCorridorOccupancyRequest
{
  bool vehicle_is_locked_target{false};
  bool geometry_valid{false};
  double ego_lateral_m{};
  double vehicle_lateral_m{};
  double vehicle_longitudinal_m{};
  double lateral_clearance_m{};
  double rear_clearance_m{};
  double front_clearance_m{};
};

/// Return true when a non-target V2X vehicle occupies the lateral sweep from
/// the current pass line to the reference path and is close enough
/// longitudinally to make an immediate merge unsafe.
bool blocks_overtake_return_corridor(
  const ReturnCorridorOccupancyRequest & request) noexcept;

struct EarlyReturnCancellationRequest
{
  bool return_phase{false};
  bool reacquire_enabled{false};
  bool return_corridor_blocked{false};
  double return_elapsed_sec{};
  double reacquire_window_sec{};
  double return_progress{};
  double maximum_return_progress{};
};

/// Cancel only a newly started Return when another vehicle enters the merge
/// corridor. A late outward correction is intentionally rejected.
bool should_cancel_early_overtake_return(
  const EarlyReturnCancellationRequest & request) noexcept;

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

struct CourseAlignedPredictionRequest
{
  bool enabled{false};
  bool projection_valid{false};
  double target_forward_distance_m{};
  double target_lateral_m{};
  double target_along_track_speed_mps{};
  double horizon_time_sec{};
  double ego_horizon_course_distance_m{};
  double fallback_longitudinal_m{};
  double fallback_lateral_m{};
};

struct CourseAlignedPrediction
{
  bool used_course_alignment{false};
  double longitudinal_m{};
  double lateral_m{};
};

/// Advance a target along the reference course while retaining its projected
/// lateral offset. Invalid or disabled projections preserve the Cartesian
/// constant-velocity result supplied by the caller.
CourseAlignedPrediction resolve_course_aligned_prediction(
  const CourseAlignedPredictionRequest & request) noexcept;

struct CourseLateralPredictionRequest
{
  bool enabled{false};
  bool current_projection_valid{false};
  bool previous_projection_valid{false};
  double current_lateral_m{};
  double previous_lateral_m{};
  double sample_interval_sec{};
  double sample_age_sec{};
  double horizon_time_sec{};
  double velocity_deadband_mps{};
  double maximum_velocity_mps{};
  double fallback_lateral_m{};
};

struct CourseLateralPrediction
{
  bool used_course_lateral_velocity{false};
  double lateral_m{};
  double raw_lateral_velocity_mps{};
  double applied_lateral_velocity_mps{};
};

/// Predict target lateral position from the change in Frenet lateral offset
/// between two source samples. Small velocities are suppressed and remaining
/// velocities are bounded before extrapolation. Invalid observations preserve
/// the Cartesian constant-velocity result supplied by the caller.
CourseLateralPrediction resolve_course_lateral_prediction(
  const CourseLateralPredictionRequest & request) noexcept;

struct CoursePoint
{
  double x_m{};
  double y_m{};
};

struct VehicleRelativeLateralRequest
{
  bool course_projection_used{false};
  double vehicle_course_lateral_m{};
  double ego_course_lateral_m{};
  double local_relative_lateral_m{};
};

/// Course projection returns each vehicle's lateral coordinate relative to the
/// reference path, not relative to ego. Subtract ego's coordinate before using
/// it as a collision-overlap test. Fall back to local relative geometry when a
/// finite common-course pair is unavailable.
double resolve_vehicle_relative_lateral(
  const VehicleRelativeLateralRequest & request) noexcept;

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
  std::optional<double> preferred_target_path_progress_m;
  double max_target_path_progress_change_m{std::numeric_limits<double>::infinity()};
};

struct ForwardCourseProjection
{
  bool valid{false};
  double forward_distance_m{};
  double lateral_m{};
  double along_track_speed_mps{};
  double cross_track_distance_m{};
  std::size_t segment_index{};
  double target_path_progress_m{};
};

/// Project ego and a V2X target onto the same bounded section of a reference
/// polyline. The result is path progress, not the target's Cartesian distance
/// in the ego tangent frame. A circular path may wrap once, but the bounded
/// lookahead prevents selecting a spatially close branch far around the lap.
/// When a previous target progress is supplied, candidates on a topologically
/// distant branch are rejected even if that branch is closer in Cartesian
/// distance.
ForwardCourseProjection project_forward_course_progress(
  const std::vector<CoursePoint> & path, const ForwardCourseProjectionRequest & request);

/// Distinguish a real rejection by the target-progress continuity constraint
/// from a target that is unavailable even to the same unconstrained bounded
/// projection. The unconstrained result is diagnostic-only.
bool is_course_progress_continuity_constraint_rejection(
  bool continuity_constraint_applied, bool constrained_projection_valid,
  bool unconstrained_projection_valid) noexcept;

struct RelativeCourseProgressContinuityRequest
{
  double previous_longitudinal_m{};
  double observed_longitudinal_m{};
  double elapsed_sec{};
  double ego_speed_mps{};
  double target_speed_mps{};
  double tolerance_m{};
};

/// Reject a relative course-progress change that cannot be explained by the
/// distance either vehicle could have travelled since the last accepted
/// observation. This keeps a nearby hairpin branch from appearing as an
/// instantaneous completed pass.
bool is_relative_course_progress_continuous(
  const RelativeCourseProgressContinuityRequest & request) noexcept;

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

struct CurveEntryCompletionOverrideRequest
{
  bool curve_entry_allowed{false};
  bool line_committed{false};
  bool front_vehicle_seen{false};
  double front_distance_m{};
  double maximum_front_distance_m{};
  double ego_speed_mps{};
  double front_speed_mps{};
  double minimum_relative_speed_mps{};
};

/// A curve-specific new-entry exception may bypass the distance estimate only when
/// the target is inside the bounded line-entry range and measured ego speed already
/// has the configured advantage over the front kart.
bool can_override_completion_for_curve_entry(
  const CurveEntryCompletionOverrideRequest & request) noexcept;

struct InnerCurvePrecommitRequest
{
  bool enabled{false};
  bool inner_curve_entry_allowed{false};
  bool line_committed{false};
  bool front_vehicle_seen{false};
  bool emergency_brake_required{false};
  double front_distance_m{};
  double minimum_front_distance_m{};
  double maximum_front_distance_m{};
  double continuous_open_distance_m{};
  double minimum_open_distance_m{};
  double ego_speed_mps{};
  double front_speed_mps{};
  double minimum_relative_speed_mps{};
};

/// Permit an uncommitted inner line to bypass only the pass-completion estimate.
///
/// The caller remains responsible for the inflated vehicle/wall corridor,
/// lateral-reachability, cooldown, and forbidden-waypoint checks.
bool can_precommit_inner_curve_line(
  const InnerCurvePrecommitRequest & request) noexcept;

struct OvertakeCompletionPermissionRequest
{
  bool completion_feasible{false};
  bool curve_entry_allowed{false};
  bool curve_continuation_allowed{false};
  bool line_committed{false};
  bool front_vehicle_seen{false};
  double front_distance_m{};
  double maximum_front_distance_m{};
  double ego_speed_mps{};
  double front_speed_mps{};
  double minimum_relative_speed_mps{};
};

/// Combine the normal completion-distance decision with the narrowly scoped
/// near-target/measured-speed curve-entry exception and committed mission continuity.
///
/// The completion-distance estimate is an entry admission check. Once a line is
/// committed, current corridor/target/wall guards own continuity and a temporary
/// stop must not require an already-positive measured closing speed to resume.
bool overtake_completion_policy_allows_execution(
  const OvertakeCompletionPermissionRequest & request) noexcept;

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
  bool hard_entry_enabled{false};
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
  bool hard_entry_allowed{false};
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
  bool hard_entry_enabled{false};
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
  bool hard_entry_allowed{false};
  bool hard_continuation_allowed{false};
};

/// Allow a new pass on the inside of a soft curve, then keep the same locked
/// inside line through a hard curve while its geometric gap remains available.
/// hard_entry_enabled is a separate simulation-race exception for starting
/// after the hard-curve boundary.
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
  HigherQuality,
  Locked,
  LockedQualitySwitch,
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

struct OvertakeSideQualityCandidate
{
  PassSide side{PassSide::None};
  bool feasible{false};
  double side_clearance_m{0.0};
  double corridor_width_m{0.0};
  double continuous_open_distance_m{0.0};
  double required_lateral_accel_mps2{0.0};
};

struct OvertakeSideQualitySelectionRequest
{
  PassSide preferred{PassSide::None};
  PassSide locked{PassSide::None};
  OvertakeSideQualityCandidate left;
  OvertakeSideQualityCandidate right;
  bool allow_locked_reselection{false};
  double minimum_score_advantage{0.0};
};

struct OvertakeSideQualitySelection
{
  PassSide side{PassSide::None};
  SideSelectionReason reason{SideSelectionReason::NoFeasibleSide};
  double left_score{-std::numeric_limits<double>::infinity()};
  double right_score{-std::numeric_limits<double>::infinity()};
};

/// Rank both executable sides using inflated lateral room, continuous open
/// distance and ShiftOut lateral-acceleration demand. Curve inside/outside is
/// intentionally not a score input.
double score_overtake_side_quality(
  const OvertakeSideQualityCandidate & candidate) noexcept;

/// Select the higher-quality executable side. A locked side remains preferred
/// unless early-reselection is explicitly enabled and the alternate exceeds
/// the configured score advantage.
OvertakeSideQualitySelection select_overtake_side_by_quality(
  const OvertakeSideQualitySelectionRequest & request) noexcept;

struct CurveAttackSideRequest
{
  PassSide inner_side{PassSide::None};
  PassSide locked_side{PassSide::None};
  bool left_feasible{false};
  bool right_feasible{false};
  double left_continuous_open_distance_m{0.0};
  double right_continuous_open_distance_m{0.0};
  double minimum_inner_open_distance_m{0.0};
};

/// Select an aggressive curve pass side before ShiftOut.
///
/// A sufficiently long inside corridor wins. Otherwise an executable outside
/// corridor is preferred. Once a side is locked, the lock remains authoritative
/// and an unavailable lock never causes an in-manoeuvre side switch.
SideSelection select_curve_attack_side(const CurveAttackSideRequest & request) noexcept;

PassSide opposite_side(PassSide side) noexcept;

bool selected_pass_side_ordering_conflict(
  bool active_shiftout, int pass_side_sign, bool target_seen,
  bool target_position_jump, double target_longitudinal_m,
  double maximum_guard_longitudinal_m, double target_relative_lateral_m,
  double ordering_margin_m) noexcept;

enum class EarlyShiftOutSideReplanAction
{
  Keep,
  Switch,
  Abort,
};

struct EarlyShiftOutSideReplanRequest
{
  bool enabled{false};
  bool side_switch_permitted{true};
  bool shiftout_phase{false};
  bool lateral_clearance_latched{false};
  PassSide locked_side{PassSide::None};
  PassSide candidate_side{PassSide::None};
  bool candidate_feasible{false};
  bool selected_side_conflict{false};
  double lateral_progress_m{0.0};
  double maximum_lateral_progress_m{0.0};
  double traveled_distance_m{0.0};
  double maximum_traveled_distance_m{0.0};
  double candidate_stable_sec{0.0};
  double required_stable_sec{0.0};
};

struct EarlyShiftOutSideReplanResolution
{
  EarlyShiftOutSideReplanAction action{EarlyShiftOutSideReplanAction::Keep};
  bool inside_switch_window{false};
};

/// Switch only in the shallow ShiftOut window after a stable alternate-side
/// decision. A stable selected-side conflict outside that window aborts rather
/// than crossing the target with a direct side reversal.
EarlyShiftOutSideReplanResolution resolve_early_shiftout_side_replan(
  const EarlyShiftOutSideReplanRequest & request) noexcept;

enum class OvertakeExecutionSideSource
{
  None,
  BehaviorRevalidation,
  MissionLock,
  BehaviorSelection,
};

struct OvertakeExecutionSideRequest
{
  bool resuming_paused_mission{false};
  int behavior_side_sign{0};
  int mission_side_sign{0};
};

struct OvertakeExecutionSideResolution
{
  int side_sign{0};
  OvertakeExecutionSideSource source{OvertakeExecutionSideSource::None};
};

/// Resolve the side used when starting or resuming an explicit OvertakeLine.
///
/// This preserves the current controller policy: a valid Behavior side owns a
/// FollowPrepare resume; otherwise the existing mission side wins before a new
/// Behavior selection. Keeping this policy in one named decision makes a later
/// mission-lock behavior change explicit and regression-testable.
OvertakeExecutionSideResolution resolve_overtake_execution_side(
  const OvertakeExecutionSideRequest & request) noexcept;

enum class OvertakeLineTransitionAction
{
  None,
  RecoverPhysicalWallContact,
  RejectEntryWallMargin,
  ResumePassForReturnCorridorBlocker,
  ReturnBeforeWallMarginRecovery,
  HoldCompletedPassForReturnCorridor,
  RecoverWallMargin,
  ReplanEarlyShiftOutSide,
  RecoverOccupiedPassSide,
  ReturnRearClear,
  RecoverLongitudinalProgress,
};

const char * to_string(OvertakeLineTransitionAction action) noexcept;

struct OvertakeLineTransitionRequest
{
  bool actual_wall_physical_contact{false};
  bool actual_wall_margin_blocked{false};
  bool actual_wall_sample_unavailable{false};
  bool starting_execution_phase{false};
  bool active_execution_phase{false};
  bool cancel_early_return_for_corridor_blocker{false};
  bool completed_pass_ready_to_return_before_margin_recovery{false};
  bool completed_pass_waiting_for_return_corridor{false};
  bool shiftout_phase{false};
  bool pass_phase{false};
  bool behavior_overtake{false};
  bool side_replan_ready{false};
  bool side_replan_abort{false};
  int side_replan_candidate_sign{0};
  int mission_side_sign{0};
  bool rear_clear_confirmed{false};
  bool return_corridor_blocked{false};
  bool pass_progress_watchdog_timed_out{false};
};

/// Select exactly one active OvertakeLine action using the controller's current
/// safety and mission priority. The caller remains responsible for state
/// mutation and logging; this function only makes the previously implicit
/// if/else priority testable.
OvertakeLineTransitionAction resolve_overtake_line_transition(
  const OvertakeLineTransitionRequest & request) noexcept;

/// Log only the first cycle of one action event. Passing None resets the
/// previous action in the caller, so a later recurrence is a new event.
bool should_log_overtake_line_transition_action(
  OvertakeLineTransitionAction action,
  OvertakeLineTransitionAction previous_action) noexcept;

/// Gate race-only V2X behavior when AWSIM state tracking is available. Launches
/// without state tracking retain the legacy always-active behavior. A prepared
/// start-grid Ready rollout is active because AWSIM already moves the vehicle
/// before its per-vehicle Start event reaches the controller.
bool is_v2x_behavior_session_active(
  bool state_tracking_enabled, bool race_started, bool start_grid_ready_rollout) noexcept;

struct LowSpeedBypassCandidateRequest
{
  bool enabled{false};
  bool candidate_vehicle_present{false};
  bool cooldown_active{false};
  bool start_grid_stop_suppressed{false};
  bool overtake_forbidden{false};
  bool continuing{false};
  bool ignore_soft_curve_forbidden{false};
  bool explicit_forbidden_wp{false};
  double vehicle_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_vehicle_speed_mps{};
  double forward_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_prepare_distance_m{};
  double maximum_entry_distance_m{};
};

/// Select a stopped-vehicle bypass candidate independently from the narrow
/// footprint-overlap set used by emergency braking. This allows a vehicle in
/// the future course corridor to start lateral planning before center overlap.
bool can_start_low_speed_bypass(const LowSpeedBypassCandidateRequest & request) noexcept;

struct StoppedCandidateConfirmationRequest
{
  bool candidate_present{false};
  bool same_candidate{false};
  double observation_sec{std::numeric_limits<double>::quiet_NaN()};
  double previous_observation_sec{std::numeric_limits<double>::quiet_NaN()};
  int previous_count{0};
  int required_count{1};
  double maximum_observation_gap_sec{};
};

struct StoppedCandidateConfirmationResult
{
  int observation_count{0};
  bool confirmed{false};
  double last_observation_sec{std::numeric_limits<double>::quiet_NaN()};
};

/// Count only distinct, consecutive V2X observations for the same stopped
/// vehicle. Repeated control ticks over one received sample do not advance the
/// confirmation.
StoppedCandidateConfirmationResult update_stopped_candidate_confirmation(
  const StoppedCandidateConfirmationRequest & request) noexcept;

struct StoppedVehicleLineOwnershipRequest
{
  bool low_speed_behavior_active{false};
  bool low_speed_candidate{false};
  bool low_speed_direct_control_active{false};
  bool committed_pass_mission_active{false};
  bool overtake_behavior_active{false};
  bool has_front_vehicle{false};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double front_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_stopped_speed_mps{};
  double stopped_detection_distance_m{};
};

/// Yield the generic OvertakeLine only when the stopped-vehicle bypass owns
/// the lateral plan. A candidate without a feasible local path must not erase
/// a committed generic pass mission.
bool should_yield_overtake_line_to_stopped_bypass(
  const StoppedVehicleLineOwnershipRequest & request) noexcept;

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

enum class LowSpeedDirectControlPhase
{
  Shift,
  Pass,
  Rejoin,
};

/// A feasible stopped-vehicle local path must start direct control even when
/// ego is already inside its pass corridor. In that case Shift is already
/// complete and Pass owns the first control cycle.
LowSpeedDirectControlPhase resolve_low_speed_direct_control_entry_phase(
  bool pass_corridor_entered) noexcept;

/// Stop an active stopped-vehicle direct maneuver when its live local corridor
/// is unavailable. Rejoin is already protected by independent wall guards and
/// intentionally no longer depends on the passed vehicle corridor.
bool should_stop_low_speed_direct_control_for_corridor(
  bool direct_control_active, bool rejoin_active,
  bool local_path_active, bool local_path_feasible) noexcept;

/// Select the bounded direct-control speed without handing ownership to MPC
/// inside a stopped-vehicle pack.
double resolve_low_speed_direct_control_velocity(
  LowSpeedDirectControlPhase phase,
  double shift_velocity_mps,
  double pass_velocity_mps,
  double rejoin_velocity_mps,
  double maximum_velocity_mps);

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

/// Limit a controller-side steering target so the published tire-angle command
/// cannot exceed the configured lateral acceleration at the measured speed.
double limit_low_speed_shift_steering_by_lateral_acceleration(
  double target_steering_rad, double current_speed_mps, double wheelbase_m,
  double maximum_lateral_acceleration_mps2, double steering_command_gain);

bool is_low_speed_shift_complete(
  double current_lateral_m, double current_heading_error_rad,
  double target_lateral_m, double lateral_tolerance_m,
  double heading_tolerance_rad) noexcept;

/// Start returning from the pass corridor after the complete stopped-vehicle
/// pack has remained clear for the configured hold duration.
bool should_begin_low_speed_shift_rejoin(
  bool has_front_vehicle, bool has_side_vehicle,
  bool has_clearance_vehicle, double clear_duration_sec,
  double required_clear_duration_sec) noexcept;

/// Release the direct low-speed shift controller only after its rejoin target
/// pose is settled and the complete stopped-vehicle pack remains clear.
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

struct CommittedExecutionContinuityRequest
{
  bool active_execution_phase{false};
  bool target_progress_continuous{false};
  bool target_ahead{false};
  bool target_pass_side_intrusion{false};
  bool live_execution_corridor_blocked{false};
  bool explicit_forbidden_waypoint{false};
  bool emergency_front_risk{false};
};

/// Once ShiftOut/Pass is committed, keep the fixed target/side/corridor across
/// transient behavior entry-policy changes. Only current execution hard guards
/// may invalidate this hold.
bool can_hold_committed_execution_after_behavior_drop(
  const CommittedExecutionContinuityRequest & request) noexcept;

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

struct RecoveryReacquireRequest
{
  bool enabled{false};
  bool phase_hold_elapsed{false};
  bool stable_target_id{false};
  bool same_target{false};
  bool target_progress_continuous{false};
  bool same_side{false};
  bool target_rear_clear{false};
  bool gap_available{false};
  bool execution_allowed{false};
  bool solver_ready{false};
};

/// Allow Recovery -> ShiftOut when the same executable pass opportunity becomes available again.
bool can_reacquire_during_recovery(const RecoveryReacquireRequest & request) noexcept;

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

struct CommittedPassProgressWatchdogRequest
{
  bool active{false};
  double target_longitudinal_m{};
  double traveled_distance_m{};
  double best_target_longitudinal_m{std::numeric_limits<double>::quiet_NaN()};
  double progress_checkpoint_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_progress_m{};
  double maximum_without_progress_distance_m{};
};

struct CommittedPassProgressWatchdogResolution
{
  double best_target_longitudinal_m{std::numeric_limits<double>::quiet_NaN()};
  double progress_checkpoint_distance_m{std::numeric_limits<double>::quiet_NaN()};
  bool observation_accepted{false};
  bool progressed{false};
  bool timed_out{false};
};

/// Abort a committed Pass that consumes too much course distance without
/// reducing the locked target's ahead distance. Inactive or invalid
/// observations pause the watchdog. A phase-distance rollback re-baselines it.
CommittedPassProgressWatchdogResolution update_committed_pass_progress_watchdog(
  const CommittedPassProgressWatchdogRequest & request) noexcept;

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

struct RecoveryVelocityLimitRequest
{
  double configured_velocity_limit_mps{};
  bool moving_follow_profile_available{false};
  double moving_follow_velocity_limit_mps{std::numeric_limits<double>::infinity()};
  bool solver_recovery_active{false};
};

struct RecoveryVelocityLimitResolution
{
  double velocity_limit_mps{};
  bool moving_follow_profile_used{false};
};

/// Select the longitudinal ceiling used while an overtake line returns in Recovery.
///
/// A fresh moving target delegates longitudinal control to the normal Follow profile. An
/// infinite Follow limit means the target is outside the normal Follow distance gate, so
/// Recovery adds no extra ceiling. Solver recovery and unavailable target observations retain
/// the configured fail-closed ceiling.
RecoveryVelocityLimitResolution resolve_recovery_velocity_limit(
  const RecoveryVelocityLimitRequest & request);

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

struct StartGridCorridorScoreRequest
{
  double corridor_center_ey{};
  double corridor_width{};
  double ego_ey{};
};

struct UnvalidatedOvertakeFallbackRequest
{
  bool start_grid_breakout_attempt{false};
  bool geometric_gap_available{false};
  bool side_clearance_available{false};
  bool emergency_brake{false};
};

/// Decide whether an instantaneous side-clearance fallback may replace the geometric gap.
///
/// A start-grid breakout must use the explicitly validated wall/car or car/car corridor. If that
/// corridor disappears, accepting the generic fallback can create a new target-relative line far
/// inside the track and carry it into the first hairpin.
bool can_try_unvalidated_overtake_fallback(
  const UnvalidatedOvertakeFallbackRequest & request) noexcept;

struct OffsetCurveFeasibilityRequest
{
  double reference_curvature_radpm{};
  double lateral_offset_m{};
  double max_abs_curvature_radpm{};
  double min_frenet_denominator{0.10};
};

struct OffsetCurveFeasibilityResult
{
  bool feasible{false};
  double offset_curvature_radpm{std::numeric_limits<double>::infinity()};
  double frenet_denominator{};
};

/// Check whether a constant Frenet offset remains turnable at one reference-path sample.
/// Inside offsets shrink the turn radius according to kappa/(1-kappa*e_y); outer offsets reduce
/// curvature. A non-positive/small denominator is rejected before it can create a folded line.
OffsetCurveFeasibilityResult evaluate_offset_curve_feasibility(
  const OffsetCurveFeasibilityRequest & request);

/// Score a collision-inflated start-grid corridor.
///
/// Lower is better. Lateral travel dominates; width is a small tie breaker so an equally close
/// wider corridor wins without making ego cross the track merely to chase width.
double score_start_grid_corridor(const StartGridCorridorScoreRequest & request);

/// Maximum lateral half-extent of a rectangle when its yaw is unknown.
///
/// This is the rectangle half-diagonal. It is used only where V2X does not provide target yaw;
/// the aggressive vehicle-vehicle slot keeps its separately configured circular inflation.
double conservative_rectangle_lateral_half_extent(double length_m, double width_m);

struct WallCorridorGeometryRequest
{
  double lower{};
  double upper{};
  bool lower_is_vehicle{false};
  bool upper_is_vehicle{false};
  /// Extra inflation for the sole vehicle boundary of a wall-vehicle corridor.
  double vehicle_extra_inflation_m{};
  /// Required ego-center clearance from every wall boundary.
  double wall_clearance_m{};
  /// Minimum residual ego-center interval after all geometry is applied.
  double minimum_width_m{};
};

struct WallCorridorGeometryResult
{
  bool feasible{false};
  double lower{};
  double upper{};

  double width() const noexcept
  {
    return upper - lower;
  }
};

/// Apply vehicle inflation and wall clearance before declaring a lateral corridor feasible.
///
/// Unlike the legacy target-only adjustment, this never clamps wall clearance to half of an
/// already narrow interval. A corridor that closes after the physical margins is rejected.
WallCorridorGeometryResult evaluate_wall_corridor_geometry(
  const WallCorridorGeometryRequest & request);

struct StartGridBoundaryCandidateRequest
{
  double forward_distance_m{};
  double lookbehind_distance_m{};
  double lookahead_distance_m{};
};

/// Accept a vehicle as a start-grid lateral boundary inside a bounded common-progress window.
///
/// Unlike normal front detection, the start grid intentionally admits a nearby side/rear vehicle:
/// staggered rows can put one edge of the useful vehicle-vehicle corridor slightly behind ego by
/// the time another domain reports Start.
bool is_start_grid_boundary_candidate(const StartGridBoundaryCandidateRequest & request);

struct InterVehicleRearClearRequest
{
  bool lower_vehicle_seen{false};
  bool upper_vehicle_seen{false};
  double lower_longitudinal_m{std::numeric_limits<double>::infinity()};
  double upper_longitudinal_m{std::numeric_limits<double>::infinity()};
  double return_clear_distance_m{};
};

/// Require both boundary vehicles to be observed behind ego before leaving a woven corridor.
bool is_inter_vehicle_corridor_rear_clear(const InterVehicleRearClearRequest & request);

}  // namespace multi_purpose_mpc_ros::v2x_overtake_core

#endif  // MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
