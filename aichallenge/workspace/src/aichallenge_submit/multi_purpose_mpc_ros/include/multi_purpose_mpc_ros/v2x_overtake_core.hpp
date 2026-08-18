#ifndef MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_
#define MULTI_PURPOSE_MPC_ROS__V2X_OVERTAKE_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace multi_purpose_mpc_ros::v2x_overtake_core
{

/// Session-scoped identity discovery for V2X completeness checks.
///
/// Only callers that have already validated the enclosing message should call
/// observe_valid_message().  Learned identities survive individual Recovery
/// attempts and are cleared only at the race-session boundary.
class V2XPeerIdentityTracker
{
public:
  /// Add every unique, non-empty identity from a structurally valid message.
  /// Returns false and leaves the learned set unchanged for invalid input.
  bool observe_valid_message(const std::vector<std::string> & vehicle_ids);

  /// A complete observation contains each learned identity exactly once and
  /// contains no unlearned identity.  An empty learned set fails closed.
  bool is_complete(const std::vector<std::string> & vehicle_ids) const;

  std::size_t learned_vehicle_count() const noexcept;
  void reset() noexcept;

private:
  std::unordered_set<std::string> learned_vehicle_ids_;
};

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

struct OvertakeEntryPrearmSpeedReferenceRequest
{
  bool active{false};
  double base_reference_speed_mps{};
  double hard_cap_mps{};
  double prearm_target_speed_mps{};
};

struct OvertakeEntryPrearmSpeedReferenceResolution
{
  double reference_speed_mps{};
  bool reference_floor_applied{false};
};

/// A validated entry pre-arm owns a bounded longitudinal reference floor, not
/// merely a ceiling. The dynamic hard cap remains authoritative.
OvertakeEntryPrearmSpeedReferenceResolution resolve_overtake_entry_prearm_speed_reference(
  const OvertakeEntryPrearmSpeedReferenceRequest & request);

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
  bool shiftout_phase{false};
  bool pass_phase{false};
  bool lateral_complete{false};
  bool front_cap_released{false};
  bool lateral_exclusion_latched{false};
  bool current_lateral_separation_clear{false};
  bool lateral_separation_above_reapply_threshold{false};
  bool target_position_jump{false};
  bool target_seen{false};
  bool execution_path_physically_feasible{false};
  bool actual_wall_contact{false};
  double target_speed_mps{};
  double slow_target_max_speed_mps{};
  double configured_min_speed_mps{};
};

/// A reference-only velocity floor may help finish a physically committed Pass,
/// or a ShiftOut that already has full current physical lateral separation,
/// around a stopped/very-slow target. The caller must still clamp the result to
/// all MPC hard speed bounds; this policy never overrides a safety limit or
/// releases the ShiftOut front-speed cap.
bool should_apply_committed_pass_speed_floor(
  const CommittedPassSpeedFloorRequest & request) noexcept;

struct CourseFrameFootprintSweepRequest
{
  double current_longitudinal_m{};
  double current_lateral_m{};
  double predicted_longitudinal_m{};
  double predicted_lateral_m{};
  double longitudinal_clearance_m{};
  double lateral_clearance_m{};
};

/// Return true when the linearly predicted relative center path does not enter
/// the open course-frame body-overlap rectangle. Touching its boundary is
/// separation; invalid observations fail closed.
bool course_frame_body_footprints_remain_separated(
  const CourseFrameFootprintSweepRequest & request) noexcept;

struct PredictedFootprintOverlapConfirmationRequest
{
  bool monitor_active{false};
  double now_sec{};
  double overlap_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double confirm_sec{};
};

struct PredictedFootprintOverlapConfirmation
{
  bool confirmed{false};
  double overlap_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double elapsed_sec{};
};

/// Confirm a continuously predicted body overlap before revoking an already
/// released minimum-motion Pass speed cap. Invalid input and a clear sample
/// reset confirmation.
PredictedFootprintOverlapConfirmation update_predicted_footprint_overlap_confirmation(
  const PredictedFootprintOverlapConfirmationRequest & request) noexcept;

enum class CommittedPassFrontCapTransitionReason
{
  None,
  MinimumMotionFootprintSweepClear,
  MinimumMotionSideBySideForwardEscape,
  ConstrainedFeasiblePassHorizonAccepted,
  LateralGoalAndExecutionHorizonClear,
  TargetNoLongerAhead,
  CurrentFootprintOverlap,
  PredictedFootprintOverlap,
  FootprintPredictionUnavailable,
  LockedTargetPositionJump,
  ActualWallContact,
  LockedTargetUnavailable,
  LateralGoalIncomplete,
  ExecutionHorizonConstrained,
  LateralClearanceBelowReapplyThreshold,
};

struct CommittedPassPolicyRequest
{
  bool preserve_validated_breakout_line{false};
  /// The side, lateral goal and ShiftOut/Pass/Return distances were admitted
  /// together and frozen before execution. This is required before ShiftOut
  /// may acquire longitudinal ownership from a footprint sweep.
  bool validated_frozen_plan{false};
  bool shiftout_phase{false};
  bool pass_phase{false};
  bool lateral_complete{false};
  bool execution_horizon_unconstrained{false};
  bool execution_path_physically_feasible{false};
  bool actual_wall_contact{false};
  bool minimum_motion_corridor_active{false};
  /// Robust separation is required to acquire the first release.  Once a
  /// validated Pass owns longitudinal control, these physical fields allow
  /// the lease to survive noise inside the preferred robust margin without
  /// relaxing the actual body boundary.
  bool current_body_footprints_physically_separated{false};
  bool current_body_footprints_separated{false};
  /// A current overlap must remain continuously observed before it revokes an
  /// already released competition-simulation Pass. Defaults fail closed.
  bool current_body_footprint_overlap_confirmed{true};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_physically_separated{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool predicted_body_footprint_overlap_confirmed{true};
  bool lateral_exclusion_latched{false};
  bool prior_front_cap_release_active{false};
  bool lateral_separation_clear{false};
  bool lateral_separation_above_reapply_threshold{false};
  bool locked_target_body_lateral_clear{false};
  bool locked_target_position_jump{false};
  bool target_seen{false};
  double target_longitudinal_m{};
  double body_longitudinal_clearance_m{};
  bool committed_pass_speed_floor_enabled{false};
  double target_speed_mps{};
  double slow_target_max_speed_mps{};
  double committed_pass_min_speed_mps{};
  /// Competition-simulation policy: after a minimum-motion Pass has acquired
  /// its release, prefer forward completion over a future-overlap prediction.
  /// Confirmed current footprint overlap, wall, path-feasibility and target-continuity guards
  /// remain hard.
  bool committed_pass_attack_mode_enabled{false};
};

struct CommittedPassPolicyResolution
{
  bool active_execution{false};
  bool minimum_motion_shiftout_release_allowed{false};
  bool minimum_motion_shiftout_predicted_overlap_grace_active{false};
  bool minimum_motion_footprint_release_allowed{false};
  bool minimum_motion_footprint_hold_active{false};
  bool minimum_motion_physical_clear_hold_active{false};
  bool minimum_motion_current_overlap_grace_active{false};
  bool minimum_motion_side_by_side_escape_active{false};
  bool minimum_motion_predicted_overlap_grace_active{false};
  bool minimum_motion_attack_hold_active{false};
  bool constrained_horizon_release_allowed{false};
  bool committed_pass_speed_hold_allowed{false};
  bool front_cap_release_ready{false};
  bool front_cap_state_update_required{false};
  bool committed_pass_speed_hold_active{false};
  bool constrained_horizon_front_cap_release_active{false};
  bool committed_pass_speed_floor_active{false};
  bool committed_shiftout_speed_floor_active{false};
  CommittedPassFrontCapTransitionReason transition_reason{
    CommittedPassFrontCapTransitionReason::None};
};

/// Resolve all longitudinal speed ownership decisions for a committed ShiftOut/Pass.
/// A frozen minimum-motion ShiftOut/Pass may acquire and retain release from a
/// validated body-footprint sweep instead of the legacy fixed lateral
/// threshold. Other Pass modes retain the legacy release/reapply hysteresis.
/// The caller remains responsible for state updates, logging and clamping the
/// resulting reference-only speed floor to hard limits.
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

struct LiveExecutionCorridorHoldReferenceRequest
{
  bool pass_phase{false};
  bool current_body_footprints_separated{false};
  double pass_phase_start_sec{std::numeric_limits<double>::quiet_NaN()};
  double last_valid_corridor_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct LiveExecutionCorridorHoldReferenceResolution
{
  double reference_sec{std::numeric_limits<double>::quiet_NaN()};
  bool pass_phase_reference_used{false};
};

/// ShiftOut and Pass have different execution geometry. A physically separated
/// Pass therefore receives a fresh, bounded live-corridor hold window instead
/// of inheriting time already consumed by a ShiftOut dropout. A later genuine
/// corridor-valid observation remains the newest reference. The caller still
/// owns target-continuity, emergency, wall, solver, and overlap guards.
LiveExecutionCorridorHoldReferenceResolution
resolve_live_execution_corridor_hold_reference(
  const LiveExecutionCorridorHoldReferenceRequest & request) noexcept;

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

enum class PassCommitStage
{
  Selectable,
  ShiftCommitted,
  SideBySideCommitted,
  RearClear,
};

struct PassCommitStageRequest
{
  bool frozen_execution_active{false};
  bool lateral_clearance_latched{false};
  bool forward_completion_latched{false};
  bool rear_clear{false};
  double target_front_distance_m{std::numeric_limits<double>::infinity()};
  double side_by_side_no_return_front_distance_m{0.0};
};

struct PassCommitStageResolution
{
  bool valid{false};
  PassCommitStage stage{PassCommitStage::Selectable};
  bool side_replan_allowed{false};
};

/// Project the existing mission/lateral latches onto one tactical commit
/// stage. A frozen path can still be replaced atomically while the target is
/// sufficiently ahead. Lateral clearance alone does not mean side-by-side:
/// after forward commit or the longitudinal no-return point, the selected side
/// is immutable until rear-clear.
PassCommitStageResolution resolve_pass_commit_stage(
  const PassCommitStageRequest & request) noexcept;

const char * to_string(PassCommitStage stage) noexcept;

struct CrossSideNoReturnLatchRequest
{
  bool execution_active{false};
  bool previously_latched{false};
  PassCommitStage observed_stage{PassCommitStage::Selectable};
  bool safe_separation_active{false};
  bool side_replacement_committed{false};
};

/// Keep the cross-track no-return decision monotonic for one frozen Mission.
/// The latch itself is never cleared by target motion; a separately validated
/// SafeSeparation tactical re-arm may bypass it for one replacement without
/// mutating it. OvertakeLineState reset owns latch release.
bool resolve_cross_side_no_return_latch(
  const CrossSideNoReturnLatchRequest & request) noexcept;

struct RuntimeCompletionTacticalRearmRequest
{
  bool replan_pending{false};
  bool pass_phase{false};
  bool safe_separation_active{false};
  PassCommitStage commit_stage{PassCommitStage::Selectable};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_footprint_sweep_separated{false};
  bool execution_corridor_blocked{false};
  bool hard_fault{false};
  bool cross_side_transition_committed{false};
  double target_longitudinal_m{std::numeric_limits<double>::infinity()};
  double minimum_front_distance_m{0.0};
};

/// Re-open left/right branch assessment for one runtime-completion replan.
/// SafeSeparation normally latches cross-side no-return, but that latch may be
/// bypassed while the target is still clearly ahead and all current/predicted
/// geometry guards remain valid. Side-by-side and hard-fault cases stay closed.
bool can_rearm_runtime_completion_tactical_replan(
  const RuntimeCompletionTacticalRearmRequest & request) noexcept;

/// Require an alternate-side rollout to retain the speed which ego already
/// has, without incorrectly demanding that a slower ego instantaneously match
/// a faster target throughout a curve. Rear-clear terminal speed remains a
/// separate admission condition.
double resolve_cross_side_minimum_speed_requirement(
  double current_ego_speed_mps, double target_speed_mps) noexcept;

struct SnapshotMinimumSpeedAdmissionRequest
{
  double predicted_minimum_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  /// Requirement used by the planner snapshot which produced the prediction.
  /// NaN means legacy metadata is unavailable and selects the live fallback.
  double planning_requirement_mps{std::numeric_limits<double>::quiet_NaN()};
  double live_requirement_mps{std::numeric_limits<double>::quiet_NaN()};
  double tolerance_mps{0.02};
};

struct SnapshotMinimumSpeedAdmissionResolution
{
  bool valid{false};
  bool admitted{false};
  bool used_planning_requirement{false};
  double effective_requirement_mps{std::numeric_limits<double>::quiet_NaN()};
  double margin_mps{std::numeric_limits<double>::quiet_NaN()};
};

/// Compare one tactical prediction against the requirement from the same
/// planning snapshot. The live requirement remains the fail-closed fallback
/// for legacy candidates without snapshot metadata. Callers must separately
/// enforce result freshness and current-state physical hard constraints.
SnapshotMinimumSpeedAdmissionResolution resolve_snapshot_minimum_speed_admission(
  const SnapshotMinimumSpeedAdmissionRequest & request) noexcept;

enum class CrossSideMissionReplacementReason
{
  None,
  Inactive,
  SameSide,
  NoReturn,
  SafeSeparation,
  CandidateInfeasible,
  RearClearUnchecked,
  RearClearInfeasible,
  AdditionalSideTransitionRequired,
  InvalidPrediction,
  WallReserveInsufficient,
  TimeBudgetExceeded,
  DistanceBudgetExceeded,
  MinimumSpeedInsufficient,
  RearClearSpeedInsufficient,
  Admitted,
};

struct CrossSideMissionReplacementRequest
{
  bool active_execution{false};
  bool side_changed{false};
  bool before_no_return{false};
  bool no_return_latched{false};
  bool safe_separation_active{false};
  /// SafeSeparation may explicitly re-open one cross-side choice after the
  /// target has moved clearly ahead and current/predicted geometry is clear.
  /// This must never be inferred from the longitudinal distance alone.
  bool tactical_no_return_rearmed{false};
  bool candidate_feasible{false};
  bool rear_clear_prediction_checked{false};
  bool rear_clear_prediction_feasible{false};
  bool candidate_requires_additional_side_transition{false};
  /// True only when the additional scheduled transition, its remaining Pass,
  /// and Return path have already passed the complete static preflight.
  bool candidate_additional_side_transition_preflight_validated{false};
  double predicted_rear_clear_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_rear_clear_distance_m{std::numeric_limits<double>::infinity()};
  double predicted_rear_clear_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double predicted_minimum_ego_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_rear_clear_speed_mps{0.0};
  double minimum_ego_speed_mps{0.0};
  double minimum_path_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double minimum_required_path_wall_clearance_m{0.0};
  double remaining_time_budget_sec{std::numeric_limits<double>::infinity()};
  double remaining_distance_budget_m{std::numeric_limits<double>::infinity()};
  bool pass_phase{false};
  double planning_minimum_ego_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_speed_tolerance_mps{0.02};
};

struct CrossSideMissionReplacementResolution
{
  bool valid{false};
  bool admitted{false};
  bool restart_shiftout{false};
  CrossSideMissionReplacementReason reason{CrossSideMissionReplacementReason::None};
};

/// Admit an opposite-side replacement only while it is still an early
/// maneuver and its complete rear-clear rollout fits the remaining runtime
/// budget without dropping below the caller-provided current-state speed
/// requirement. Rear-clear terminal speed is checked independently. A
/// Pass-phase replacement must restart ShiftOut so planning and runtime speed
/// policies remain identical.
CrossSideMissionReplacementResolution resolve_cross_side_mission_replacement(
  const CrossSideMissionReplacementRequest & request) noexcept;

const char * to_string(CrossSideMissionReplacementReason reason) noexcept;

struct EarlyPassSideIntrusionRiskRequest
{
  bool enabled{false};
  PassCommitStage stage{PassCommitStage::Selectable};
  int pass_side_sign{0};
  bool target_seen{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  double current_target_relative_lateral_m{std::numeric_limits<double>::infinity()};
  double predicted_target_relative_lateral_m{std::numeric_limits<double>::infinity()};
  double ordering_margin_m{0.0};
};

/// Detect a target moving toward the selected pass line before the legacy
/// current-position intrusion guard fires. Stability debounce and complete
/// alternate-mission preflight remain separate mandatory gates.
bool early_pass_side_intrusion_risk(
  const EarlyPassSideIntrusionRiskRequest & request) noexcept;

struct DirectPassPredictionHandoffRequest
{
  bool direct_pass_entry{false};
  bool selected_mission_frozen{false};
  bool locked_target_seen{false};
  bool target_id_available{false};
};

/// Behavior computes locked-target geometry from the line state at the start
/// of a control cycle. A direct Idle -> Pass transition must therefore defer
/// line execution for one cycle when the selected target has not yet appeared
/// in that locked output, rather than treating it as a failed prediction.
bool should_defer_direct_pass_prediction_handoff(
  const DirectPassPredictionHandoffRequest & request) noexcept;

enum class OvertakeEntryStage
{
  ShiftOut,
  Pass,
};

enum class OvertakeEntryStageReason
{
  NewMissionShiftOut,
  PausedMissionShiftOut,
  SafetyPauseShiftOut,
  BaseLineDirectPass,
  TinyShiftDirectPass,
  SameSideResumePass,
  SafetyPauseResumePass,
};

struct OvertakeEntryStageRequest
{
  bool direct_base_line_pass{false};
  bool direct_tiny_shift_pass{false};
  bool direct_same_side_resume{false};
  bool safety_pause_resume_pass{false};
  bool safety_pause_resume{false};
  bool resuming_paused_mission{false};
};

struct OvertakeEntryStageResolution
{
  OvertakeEntryStage stage{OvertakeEntryStage::ShiftOut};
  OvertakeEntryStageReason reason{OvertakeEntryStageReason::NewMissionShiftOut};
};

/// Resolve the initial line-execution stage and its diagnostic reason in one
/// place. Inputs are ordered by the existing controller precedence so a
/// refactor cannot silently turn a paused ShiftOut into a DirectPass or vice
/// versa. This policy classifies already-authorized execution only; it does
/// not admit a new Mission.
OvertakeEntryStageResolution resolve_overtake_entry_stage(
  const OvertakeEntryStageRequest & request) noexcept;

const char * to_string(OvertakeEntryStageReason reason) noexcept;

struct FreshShiftOutWallEntryRequest
{
  bool fresh_mission{false};
  bool shiftout_entry{false};
  bool actual_wall_physical_contact{false};
  bool current_wall_warning{false};
};

/// Hold only a fresh lateral ShiftOut when the current footprint is already
/// in wall contact or the current (not predicted) robust wall reserve is
/// exhausted. Direct Pass and resumed/committed Missions retain ownership.
bool should_hold_fresh_shiftout_for_wall(
  const FreshShiftOutWallEntryRequest & request) noexcept;

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

enum class OvertakeMissionPathStage
{
  Invalid,
  ShiftOut,
  Pass,
  Return,
  Complete,
};

struct OvertakeMissionPathRequest
{
  double path_distance_m{};
  double start_lateral_m{};
  double pass_lateral_m{};
  double return_lateral_m{};
  double shift_distance_m{};
  double pass_distance_m{};
  double return_distance_m{};
};

struct OvertakeMissionPathResolution
{
  bool valid{false};
  OvertakeMissionPathStage stage{OvertakeMissionPathStage::Invalid};
  double lateral_target_m{};
  double total_distance_m{};
};

/// Resolve one point of the immutable ShiftOut/Pass/Return path admitted for
/// an overtake mission. ShiftOut and Return use the same smoothstep profile as
/// the live phase generator; Pass holds the latched lateral goal.
OvertakeMissionPathResolution resolve_overtake_mission_path(
  const OvertakeMissionPathRequest & request) noexcept;

struct RecedingHorizonLateralSample
{
  double path_distance_m{};
  double lower_bound_m{};
  double upper_bound_m{};
  double reference_lateral_m{};
  double warm_start_lateral_m{};
};

struct RecedingHorizonLateralRequest
{
  bool enabled{false};
  double current_lateral_m{};
  double reference_weight{1.0};
  double warm_start_weight{0.5};
  double slope_weight{2.0};
  double curvature_weight{4.0};
  double current_anchor_weight{4.0};
  double relaxation{0.65};
  std::size_t iterations{12U};
  std::vector<RecedingHorizonLateralSample> samples;
};

struct RecedingHorizonLateralResolution
{
  bool valid{false};
  bool feasible{false};
  std::vector<double> lateral_targets_m;
  double objective{std::numeric_limits<double>::infinity()};
  double maximum_reference_adjustment_m{};
};

/// Solve a convex, box-constrained lateral trajectory over the complete live
/// horizon.  The samples describe one already selected pass-side topology;
/// target separation and wall clearance therefore enter as hard per-sample
/// bounds instead of post-hoc abort conditions.  Projected coordinate updates
/// keep every iterate feasible, and a warm start supplies temporal continuity.
RecedingHorizonLateralResolution optimize_receding_horizon_lateral_trajectory(
  const RecedingHorizonLateralRequest & request) noexcept;

struct RecedingHorizonTargetBoundsRequest
{
  int pass_side_sign{};
  double wall_lower_bound_m{};
  double wall_upper_bound_m{};
  double trust_lower_bound_m{};
  double trust_upper_bound_m{};
  double target_lateral_m{};
  double robust_center_separation_m{};
  double configured_center_separation_m{};
  double physical_center_separation_m{};
  bool allow_robust_degradation{false};
};

struct RecedingHorizonTargetBoundsResolution
{
  bool valid{false};
  bool robust_degraded{false};
  bool physical_separation_used{false};
  bool trust_region_expanded{false};
  double lower_bound_m{};
  double upper_bound_m{};
  double applied_center_separation_m{};
};

/// Apply target-side separation to one live-horizon sample. Robust planning
/// reserve is preferred, but it may degrade through configured separation to
/// physical body separation when the selected side still fits inside the
/// wall-feasible bounds. The trust region is not a physical guard and may be expanded
/// only for the final physical-separation attempt.
RecedingHorizonTargetBoundsResolution resolve_receding_horizon_target_bounds(
  const RecedingHorizonTargetBoundsRequest & request) noexcept;

struct RecedingHorizonElasticTargetBoundsRequest
{
  RecedingHorizonTargetBoundsRequest preferred_request;
  double hard_wall_lower_bound_m{};
  double hard_wall_upper_bound_m{};
  double hard_trust_lower_bound_m{};
  double hard_trust_upper_bound_m{};
  bool elastic_wall_clearance_enabled{false};
};

struct RecedingHorizonElasticTargetBoundsResolution
{
  RecedingHorizonTargetBoundsResolution target_bounds;
  bool hard_wall_clearance_used{false};
};

/// Resolve target bounds in the robust/preferred wall interval first. If that
/// interval cannot contain even the physical target half-space, elastic mode
/// retries inside the configured hard wall interval. The target resolver still
/// owns robust -> configured -> physical separation degradation.
RecedingHorizonElasticTargetBoundsResolution
resolve_receding_horizon_elastic_target_bounds(
  const RecedingHorizonElasticTargetBoundsRequest & request) noexcept;

struct ConservativePredictionSpeedRequest
{
  double current_ego_speed_mps{};
  double planned_ego_speed_mps{};
  double minimum_speed_mps{1.0};
};

struct ConservativePredictionSpeedResolution
{
  bool valid{false};
  bool current_momentum_retained{false};
  double prediction_ego_speed_mps{};
};

/// Resolve the ego speed used by spatial opponent prediction. A lower planned
/// speed must not assume that current closing momentum disappears instantly;
/// a faster plan still owns the prediction when it exceeds current speed.
ConservativePredictionSpeedResolution resolve_conservative_prediction_speed(
  const ConservativePredictionSpeedRequest & request) noexcept;

struct RecedingHorizonTargetPredictionRequest
{
  bool target_prediction_valid{false};
  double path_distance_m{};
  double nominal_ego_speed_mps{};
  double candidate_ego_speed_mps{};
  double prediction_horizon_sec{};
  double maximum_prediction_time_sec{};
  double target_lateral_now_m{};
  double target_lateral_predicted_m{};
  double target_longitudinal_now_m{};
  double target_longitudinal_predicted_m{};
  double longitudinal_overlap_threshold_m{};
};

struct RecedingHorizonTargetPredictionResolution
{
  bool valid{false};
  bool body_overlap_window{false};
  bool prediction_truncated{false};
  double sample_arrival_time_sec{};
  double prediction_time_sec{};
  double prediction_ratio{};
  double target_lateral_m{};
  double target_longitudinal_m{};
};

/// Re-evaluate the target pose for one path sample at a candidate ego speed.
/// The supplied longitudinal prediction is relative to the nominal ego speed.
/// Its rate is extrapolated to the path-sample arrival time and then corrected
/// for candidate ego speed. Lateral intent is held after the base prediction
/// horizon. Samples beyond maximum_prediction_time_sec are deliberately not a
/// hard encounter constraint; the receding horizon will admit them once they
/// enter the bounded prediction window.
RecedingHorizonTargetPredictionResolution
resolve_receding_horizon_target_prediction(
  const RecedingHorizonTargetPredictionRequest & request) noexcept;

struct RecedingHorizonExecutionBoundsRequest
{
  int pass_side_sign{};
  double wall_lower_bound_m{};
  double wall_upper_bound_m{};
  bool target_separation_active{false};
  double target_lateral_m{};
  double target_center_separation_m{};
};

struct RecedingHorizonExecutionBoundsResolution
{
  bool valid{false};
  double lower_bound_m{};
  double upper_bound_m{};
};

/// Resolve only execution-critical bounds for post-validation. Mission trust
/// regions intentionally do not enter this interval.
RecedingHorizonExecutionBoundsResolution resolve_receding_horizon_execution_bounds(
  const RecedingHorizonExecutionBoundsRequest & request) noexcept;

struct WallCorridorBoundRequest
{
  double base_lower_m{};
  double base_upper_m{};
  double clearance_m{};
};

struct WallCorridorBoundResolution
{
  bool valid{false};
  bool margin_degraded{false};
  double lower_m{};
  double upper_m{};
};

/// Contract one track interval by the requested wall clearance. If the
/// planning margin cannot fit, preserve the physical track interval and mark
/// the degradation instead of manufacturing an empty MPC constraint.
WallCorridorBoundResolution resolve_wall_corridor_bound(
  const WallCorridorBoundRequest & request) noexcept;

struct StagewiseMpcCorridorBoundsRequest
{
  bool enabled{false};
  std::vector<double> base_lower_m;
  std::vector<double> base_upper_m;
  std::vector<double> corridor_lower_m;
  std::vector<double> corridor_upper_m;
  std::vector<double> reference_lateral_m;
};

struct StagewiseMpcCorridorBoundsResolution
{
  bool valid{false};
  bool active{false};
  bool feasible{false};
  std::size_t applied_sample_count{};
  std::size_t first_infeasible_index{std::numeric_limits<std::size_t>::max()};
  double minimum_width_m{std::numeric_limits<double>::infinity()};
  std::vector<double> lower_m;
  std::vector<double> upper_m;
  std::vector<double> reference_lateral_m;
};

/// Intersect a validated overtake corridor with the tracking MPC state bounds.
/// The lateral reference remains soft and is only clipped into the resulting
/// hard interval. Disabled input returns the base bounds unchanged.
StagewiseMpcCorridorBoundsResolution resolve_stagewise_mpc_corridor_bounds(
  const StagewiseMpcCorridorBoundsRequest & request) noexcept;

struct TargetBoundMpcGateRequest
{
  bool mission_active{false};
  bool candidate_target_bound{false};
  bool solver_fallback{false};
  double now_sec{};
  double candidate_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double suppressed_until_sec{-std::numeric_limits<double>::infinity()};
  double confirm_sec{};
  double solver_cooldown_sec{};
};

struct TargetBoundMpcGateResolution
{
  bool target_bound_enabled{false};
  bool confirmation_pending{false};
  bool solver_suppressed{false};
  double candidate_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double suppressed_until_sec{-std::numeric_limits<double>::infinity()};
};

/// Promote fresh opponent bounds only after continuous confirmation. Candidate
/// loss releases immediately; a solver fallback starts a wall-only cooldown
/// and requires a fresh confirmation after that cooldown.
TargetBoundMpcGateResolution update_target_bound_mpc_gate(
  const TargetBoundMpcGateRequest & request) noexcept;

struct TrackingMpcTargetBoundReleaseRequest
{
  bool pass_phase{false};
  bool lateral_clearance_latched{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool target_course_progress_rejected{false};
  bool current_body_footprints_separated{false};
  bool predicted_footprint_sweep_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool execution_corridor_blocked{false};
  bool emergency_front_risk{false};
};

/// Once a validated Pass has acquired physical lateral clearance, opponent
/// bounds remain in the receding-horizon planner but need not be duplicated as
/// hard state bounds in the lower-level tracking MPC. Any continuity or
/// overlap uncertainty restores the target bounds immediately.
bool can_release_tracking_mpc_target_bound(
  const TrackingMpcTargetBoundReleaseRequest & request) noexcept;

struct RecedingHorizonRearClearBoundsReleaseRequest
{
  bool pass_phase{false};
  bool rear_clear_confirmed{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool predicted_overlap_confirmed{false};
  bool execution_corridor_blocked{false};
  bool emergency_front_risk{false};
  /// Return is admitted only after rear-clear.  Its already-passed target must
  /// not remain as a lateral obstacle merely because the live V2X observation
  /// expires while ego converges to the base line.
  bool return_phase{false};
};

/// Release opponent bounds only after longitudinal rear-clear is confirmed.
/// Body-clear may release a longitudinal speed cap, but it must not remove the
/// lateral obstacle while the locked target is still alongside or ahead.  An
/// admitted Return may use its latched rear-clear evidence without requiring a
/// continuing observation of the now-rearward target.
bool can_release_receding_horizon_rear_clear_bounds(
  const RecedingHorizonRearClearBoundsReleaseRequest & request) noexcept;

struct RearClearReturnDeferralHoldRequest
{
  bool pass_phase{false};
  bool rear_clear_confirmed{false};
  bool return_preflight_deferred{false};
  bool current_side_horizon_feasible{false};
  bool wall_physical_contact{false};
  bool wall_sample_unavailable{false};
  bool emergency_front_risk{false};
  bool solver_recovery_active{false};
  bool overtake_forbidden{false};
};

/// A failed Return preflight is a lateral replan request, not a Pass failure.
/// Keep the acquired side only while a freshly validated current-side horizon
/// exists and no higher-priority hard fault owns the controller.
bool can_hold_pass_during_rear_clear_return_deferral(
  const RearClearReturnDeferralHoldRequest & request) noexcept;

struct ImminentRearClearPassHoldRequest
{
  bool pass_phase{false};
  bool side_by_side_committed{false};
  bool forward_completion_latched{false};
  bool future_replan_failure{false};
  bool target_seen{false};
  bool target_matches{false};
  bool target_continuity_valid{false};
  double target_longitudinal_m{std::numeric_limits<double>::infinity()};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool fresh_forward_progress{false};
  bool execution_corridor_blocked{true};
  bool current_side_horizon_feasible{false};
  bool wall_physical_contact{false};
  bool wall_margin_blocked{false};
  bool wall_sample_unavailable{false};
  bool emergency_front_risk{false};
  bool solver_recovery_active{false};
  bool overtake_forbidden{false};
};

/// Keep Pass authority while an already committed, rearward target crosses the
/// final rear-clear threshold. This only converts a future target/physical
/// replan failure into a short, freshly wall-validated current-side hold; live
/// wall, front-risk and solver faults retain their normal priority.
bool can_hold_pass_until_imminent_rear_clear(
  const ImminentRearClearPassHoldRequest & request) noexcept;

struct RecedingHorizonWarmStartRequest
{
  double forward_progress_m{};
  std::vector<double> previous_path_distances_m;
  std::vector<double> previous_lateral_targets_m;
  std::vector<double> current_path_distances_m;
  std::vector<double> current_fallback_targets_m;
};

struct RecedingHorizonWarmStartResolution
{
  bool valid{false};
  bool used_previous_solution{false};
  std::vector<double> lateral_targets_m;
};

/// Shift the prior receding-horizon solution by measured forward progress and
/// interpolate it at the current horizon samples. Samples beyond the prior
/// horizon use the current baseline instead of extending a stale terminal
/// value.
RecedingHorizonWarmStartResolution resample_receding_horizon_warm_start(
  const RecedingHorizonWarmStartRequest & request) noexcept;

struct RecedingHorizonExecutionLeaseRequest
{
  bool enabled{false};
  bool active_execution_phase{false};
  bool mission_path_frozen{false};
  bool mission_side_valid{false};
  bool last_solution_feasible{false};
  bool mission_generation_matches{false};
  bool mission_side_matches{false};
  bool target_continuous_or_leased{false};
  bool target_position_jump{false};
  bool target_course_progress_rejected{false};
  bool execution_corridor_blocked{false};
  bool explicit_forbidden_waypoint{false};
  bool emergency_front_risk{false};
  bool solver_recovery_requested{false};
  bool hard_wall_fault{false};
  double now_sec{};
  double last_feasible_sec{-std::numeric_limits<double>::infinity()};
  double maximum_age_sec{};
};

struct RecedingHorizonRefreshRequest
{
  bool enabled{false};
  bool cached_evaluation_available{false};
  bool mission_context_matches{false};
  bool reference_waypoint_matches{false};
  bool continuity_lease_active{false};
  bool force_refresh{false};
  double now_sec{};
  double last_refresh_sec{-std::numeric_limits<double>::infinity()};
  double minimum_refresh_interval_sec{};
};

struct RecedingHorizonRefreshResolution
{
  bool refresh{true};
  bool reuse_cached_evaluation{false};
};

/// Keep a recently hard-validated horizon only while its complete planning
/// context and reference waypoint are unchanged.  The exact waypoint match
/// deliberately avoids shifting static-map validation onto an unseen stage.
RecedingHorizonRefreshResolution resolve_receding_horizon_refresh(
  const RecedingHorizonRefreshRequest & request) noexcept;

/// Retain ownership of a physically validated receding-horizon solution only
/// across a bounded observation/optimizer gap. Mission identity and every
/// current hard fault remain fail-closed.
bool can_retain_receding_horizon_execution_lease(
  const RecedingHorizonExecutionLeaseRequest & request) noexcept;

struct AsyncTacticalResultLeaseRequest
{
  bool enabled{false};
  bool result_success{false};
  bool target_matches{false};
  bool context_epoch_matches{false};
  bool mission_generation_matches{false};
  bool phase_matches{false};
  bool side_matches{false};
  bool current_hard_fault{false};
  double now_sec{};
  double snapshot_sec{-std::numeric_limits<double>::infinity()};
  double maximum_age_sec{};
};

/// Retain an already accepted asynchronous tactical result between worker
/// completions. Exact planning context and current hard guards are mandatory;
/// the lease only bridges the worker/control-rate mismatch.
bool can_reuse_async_tactical_result(
  const AsyncTacticalResultLeaseRequest & request) noexcept;

struct AsyncExecutionLeaseDurationRequest
{
  double configured_lease_sec{};
  bool async_worker_enabled{false};
  double evaluation_interval_sec{};
  double last_compute_ms{};
  double control_period_sec{};
  double maximum_result_age_sec{};
};

/// Cover one replaced/missed latest-only worker result without allowing an
/// execution prefix to outlive the bounded tactical-result lease.
double resolve_async_execution_lease_duration_sec(
  const AsyncExecutionLeaseDurationRequest & request) noexcept;

struct TargetBoundExecutionHoldRequest
{
  bool enabled{false};
  /// The caller has produced a wall-validated prefix which is safe to hold.
  /// This may be a committed Pass/completed ShiftOut prefix, a recently
  /// solved ShiftOut prefix which is physically revalidated each cycle, or a
  /// bounded current-lateral freeze while an incomplete ShiftOut is replanned.
  bool safe_execution_prefix_available{false};
  bool mission_path_frozen{false};
  bool target_bound_failure{false};
  bool physical_hold_path_feasible{false};
  /// A non-hard wall warning may use the short optimizer repair window, but
  /// must not extend the same lateral prefix through the Mission-wide budget.
  bool wall_preplan_warning{false};
  bool target_progress_continuous{false};
  bool target_position_jump{false};
  bool target_course_progress_rejected{false};
  bool current_body_footprints_separated{false};
  bool recoverable_side_contact_active{false};
  /// A retained solved trajectory, unlike a measured-lateral freeze, keeps
  /// moving laterally. Require the caller's current short-horizon opponent
  /// sweep to remain valid and separated on every reuse cycle.
  bool require_predicted_sweep_separation{false};
  bool predicted_sweep_valid{false};
  bool predicted_sweep_separated{false};
  bool actual_wall_contact{false};
  bool actual_wall_margin_blocked{false};
  bool actual_wall_sample_unavailable{false};
  bool emergency_front_risk{false};
  bool solver_recovery_requested{false};
  bool explicit_forbidden_waypoint{false};
  double hold_elapsed_sec{};
  double hold_traveled_m{};
  double maximum_hold_sec{};
  double maximum_hold_distance_m{};
  /// Once the short optimizer repair deadline expires, a committed Pass may
  /// retain the same physical prefix only while measured longitudinal
  /// progress is fresh and the immutable Mission budget remains available.
  bool forward_progress_extension_enabled{false};
  bool pass_phase{false};
  /// Pass already acquired the physical lateral-clearance latch. A future
  /// target-wall prediction conflict may then retain the current same-side
  /// physical prefix inside the immutable Mission budget, even if the short
  /// repair window was previously consumed. Current body/wall hard guards in
  /// can_hold_target_bound_execution_for_replan() remain mandatory.
  bool latched_pass_clearance_acquired{false};
  bool fresh_forward_progress{false};
  double mission_elapsed_sec{};
  double mission_traveled_m{};
  double absolute_maximum_sec{};
  double absolute_maximum_distance_m{};
  /// The current Mission generation already consumed its target-bound hold
  /// budget. A replacement generation is required before another hold.
  bool mission_hold_budget_exhausted{false};
};

/// Keep a physically feasible same-side execution prefix while a target-only
/// receding-horizon conflict is re-optimized. Callers may admit Pass or a
/// completed ShiftOut. An incomplete ShiftOut may admit a recently solved and
/// physically revalidated lateral prefix inside a separate short,
/// non-extendable budget; otherwise it freezes the measured lateral position.
/// This is deliberately narrower than the generic continuity lease: predicted
/// target overlap may trigger the hold, but non-recoverable body overlap and
/// every wall/front hard fault stay closed. A separately qualified recoverable
/// side contact may continue. The normal limit is a short optimizer-repair
/// deadline. A Pass can outlive that deadline only through fresh measured
/// progress and only inside the immutable Mission-wide time/distance limits.
bool target_bound_execution_hold_budget_available(
  const TargetBoundExecutionHoldRequest & request) noexcept;

bool can_hold_target_bound_execution_for_replan(
  const TargetBoundExecutionHoldRequest & request) noexcept;

struct TargetBoundExecutionHoldLifecycleRequest
{
  bool hold_active{false};
  bool target_bound_failure{false};
  bool fresh_horizon_active{false};
  /// An explicit non-target hard failure revokes the cumulative hold.  A
  /// cycle with neither a fresh horizon nor this flag is only a neutral
  /// planner/tactical gap and retains the original budget bookkeeping.
  bool hard_failure{false};
  bool budget_exhausted{false};
  double now_sec{};
  double clear_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double clear_stable_sec{};
};

struct TargetBoundExecutionHoldLifecycleResolution
{
  bool valid{false};
  bool hold_active{false};
  bool started{false};
  bool released{false};
  bool revoked{false};
  bool exhausted{false};
  double clear_since_sec{std::numeric_limits<double>::quiet_NaN()};
};

/// Preserve one cumulative target-bound execution-prefix budget across intermittent
/// feasible optimizer results. A target-bound failure clears the stability
/// timer, while a fresh horizon must remain continuous for clear_stable_sec
/// before the budget is released. A neutral planner gap retains bookkeeping;
/// only an explicit non-target hard failure revokes it. Budget exhaustion
/// terminates retained bookkeeping even during a neutral gap. This prevents
/// solution/tactical chatter from silently rearming the bounded forward-
/// continuation window.
TargetBoundExecutionHoldLifecycleResolution resolve_target_bound_execution_hold_lifecycle(
  const TargetBoundExecutionHoldLifecycleRequest & request) noexcept;

struct OvertakeBodyClearDeadlineRequest
{
  bool enabled{false};
  OvertakeMissionPathRequest mission_path;
  double target_longitudinal_m{};
  double ego_speed_mps{};
  double target_speed_mps{};
  double target_lateral_m{};
  double target_lateral_velocity_mps{};
  double target_lateral_prediction_horizon_sec{};
  double lateral_clearance_m{};
  double hard_longitudinal_distance_m{};
  double deadline_margin_sec{};
  std::size_t sample_count{48U};
  double target_longitudinal_acceleration_mps2{};
  double target_longitudinal_acceleration_horizon_sec{};
  double target_lateral_velocity_decay_time_sec{
    std::numeric_limits<double>::infinity()};
};

struct OvertakeBodyClearDeadlineResolution
{
  bool valid{false};
  bool checked{false};
  bool feasible{false};
  bool currently_laterally_clear{false};
  double body_clear_time_sec{std::numeric_limits<double>::infinity()};
  double body_clear_distance_m{std::numeric_limits<double>::infinity()};
  double hard_distance_time_sec{std::numeric_limits<double>::infinity()};
};

/// Predict whether the immutable ShiftOut path establishes physical lateral
/// body separation before the current closing motion reaches the longitudinal
/// hard-distance boundary. Target lateral velocity is extrapolated only over
/// the caller-provided short horizon. Disabled policy preserves the legacy
/// candidate search without fabricating a deadline result.
OvertakeBodyClearDeadlineResolution resolve_overtake_body_clear_deadline(
  const OvertakeBodyClearDeadlineRequest & request) noexcept;

struct OvertakeEntryDeadlineMarginRequest
{
  double base_margin_sec{};
  int pass_side_sign{};
  double target_lateral_velocity_mps{};
  double intrusion_gain_sec_per_mps{};
  double maximum_extra_margin_sec{};
};

struct OvertakeEntryDeadlineMarginResolution
{
  bool valid{false};
  double target_intrusion_speed_mps{};
  double extra_margin_sec{};
  double effective_margin_sec{};
};

/// Increase entry-time body-clear reserve only when the target is moving into
/// the selected pass side. A stationary target, or one moving away from that
/// side, keeps the base margin so urgent stopped-vehicle escapes are not
/// disabled by a global front-distance threshold.
OvertakeEntryDeadlineMarginResolution resolve_overtake_entry_deadline_margin(
  const OvertakeEntryDeadlineMarginRequest & request) noexcept;

struct OvertakeKinematicSpeedCapSample
{
  double path_distance_m{};
  double speed_cap_mps{std::numeric_limits<double>::infinity()};
  /// Convert physical ego travel into progress on the reference course.
  /// For a Frenet offset this is approximately 1 / (1 - kappa * e_y).
  double course_progress_ratio{1.0};
};

struct OvertakeKinematicRolloutRequest
{
  bool enabled{false};
  OvertakeMissionPathRequest mission_path;
  /// Target position on the shared reference-course progress axis. This is not
  /// physical distance travelled on the target's own Frenet offset curve.
  double target_longitudinal_m{};
  double current_ego_speed_mps{};
  /// Target speed on the shared reference-course progress axis. Ego physical
  /// travel is converted to that axis by speed_caps.course_progress_ratio.
  double target_speed_mps{};
  double candidate_closing_speed_mps{};
  double maximum_ego_speed_mps{};
  double maximum_acceleration_mps2{};
  double maximum_deceleration_mps2{};
  double control_delay_sec{};
  double target_lateral_m{};
  double target_lateral_velocity_mps{};
  double target_lateral_prediction_horizon_sec{};
  double lateral_clearance_m{};
  double hard_longitudinal_distance_m{};
  double deadline_margin_sec{};
  std::vector<OvertakeKinematicSpeedCapSample> speed_caps;
  double time_step_sec{0.05};
  double maximum_time_sec{15.0};
  bool rear_clear_prediction_enabled{false};
  double rear_clear_distance_m{};
  double target_longitudinal_acceleration_mps2{};
  double target_longitudinal_acceleration_horizon_sec{};
  double target_lateral_velocity_decay_time_sec{
    std::numeric_limits<double>::infinity()};
};

struct OvertakeKinematicRolloutResolution
{
  bool valid{false};
  bool checked{false};
  bool feasible{false};
  bool currently_laterally_clear{false};
  double body_clear_time_sec{std::numeric_limits<double>::infinity()};
  double body_clear_distance_m{std::numeric_limits<double>::infinity()};
  double hard_distance_time_sec{std::numeric_limits<double>::infinity()};
  double deadline_slack_sec{-std::numeric_limits<double>::infinity()};
  double max_required_lateral_accel_mps2{};
  double ego_distance_at_horizon_m{};
  double ego_speed_at_horizon_mps{};
  double minimum_ego_speed_mps{std::numeric_limits<double>::infinity()};
  double shift_complete_time_sec{std::numeric_limits<double>::infinity()};
  double shift_complete_target_longitudinal_m{
    std::numeric_limits<double>::infinity()};
  bool rear_clear_checked{false};
  bool rear_clear_feasible{false};
  double rear_clear_time_sec{std::numeric_limits<double>::infinity()};
  double rear_clear_ego_distance_m{std::numeric_limits<double>::infinity()};
  double rear_clear_mission_distance_m{std::numeric_limits<double>::infinity()};
  double rear_clear_ego_speed_mps{std::numeric_limits<double>::infinity()};
  bool pass_target_clearance_checked{false};
  double minimum_pass_target_surface_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
};

/// Roll out longitudinal acceleration and delayed lateral mission progress on
/// one shared time axis. Path speed caps are interpolated by ego course
/// distance. The returned deadline and lateral-acceleration demand therefore
/// describe the same candidate motion instead of independent constant-speed
/// approximations.
OvertakeKinematicRolloutResolution resolve_overtake_kinematic_rollout(
  const OvertakeKinematicRolloutRequest & request) noexcept;

struct OvertakeDynamicPassDistanceRequest
{
  double shift_distance_m{};
  double minimum_pass_distance_m{};
  double rear_clear_ego_distance_m{};
  double rear_clear_ego_speed_mps{};
  double rear_clear_confirm_sec{};
  double control_delay_sec{};
  /// Keep initial admission consistent with the extra distance reserved by
  /// runtime SafeSeparation after the predicted rear-clear point.
  double runtime_completion_reserve_distance_m{};
  double soft_pass_distance_limit_m{std::numeric_limits<double>::infinity()};
  double hard_pass_distance_limit_m{std::numeric_limits<double>::infinity()};
};

struct OvertakeDynamicPassDistanceResolution
{
  bool valid{false};
  bool feasible{false};
  bool soft_limit_exceeded{false};
  double confirmation_reserve_distance_m{};
  double required_pass_distance_m{};
  double bounded_pass_distance_m{};
};

/// Convert the predicted ego rear-clear position into a Pass hold distance.
/// The confirmation reserve is speed-dependent; the runtime completion
/// reserve aligns initial admission with SafeSeparation. The hard limit is
/// never silently clamped: an over-budget mission is reported infeasible.
OvertakeDynamicPassDistanceResolution resolve_overtake_dynamic_pass_distance(
  const OvertakeDynamicPassDistanceRequest & request) noexcept;

struct OvertakeRuntimeContinuationReserveRequest
{
  double configured_course_role_reserve_distance_m{};
  double revalidation_lead_distance_m{};
  double completion_distance_margin_m{};
};

struct OvertakeRuntimeContinuationReserveResolution
{
  bool valid{false};
  double reserve_distance_m{};
};

/// Extend entry-time course-role evaluation through the distance which may be
/// consumed before runtime revalidation and bounded SafeSeparation complete.
/// This does not relax any physical guard or absolute Pass limit.
OvertakeRuntimeContinuationReserveResolution resolve_overtake_runtime_continuation_reserve(
  const OvertakeRuntimeContinuationReserveRequest & request) noexcept;

struct DynamicPredictionTimingRequest
{
  double planner_now_sec{};
  double source_age_sec{};
  double prediction_horizon_sec{};
};

struct DynamicPredictionTimingResolution
{
  bool valid{false};
  double prediction_epoch_sec{-std::numeric_limits<double>::infinity()};
  double expiry_sec{-std::numeric_limits<double>::infinity()};
  double remaining_sec{};
};

/// Normalize a source-age observation onto the controller clock. Prediction
/// validity starts at the source epoch, not when the planner happens to run.
DynamicPredictionTimingResolution resolve_dynamic_prediction_timing(
  const DynamicPredictionTimingRequest & request) noexcept;

struct CommitClockProjectionRequest
{
  double planner_clock_start_sec{};
  double monotonic_start_sec{};
  double monotonic_commit_sec{};
};

struct CommitClockProjectionResolution
{
  bool valid{false};
  double elapsed_sec{};
  double commit_clock_sec{};
};

/// Measure planner runtime on a monotonic clock, then project only that elapsed
/// duration onto the planner clock. This keeps prediction expiry and commit
/// timestamps comparable without making planner runtime sensitive to ROS time
/// jumps.
CommitClockProjectionResolution resolve_commit_clock_projection(
  const CommitClockProjectionRequest & request) noexcept;

enum class PassHorizonAction
{
  Keep,
  Return,
  RequestSameSideExtension,
  RequestLongitudinalRefresh,
  EnterHold,
  Abort,
};

struct RuntimeRearClearPredictionRequest
{
  bool enabled{false};
  bool fresh_dynamic_horizon_available{false};
  bool completion_prediction_available{false};
  bool completion_prediction_valid{false};
  bool completion_rear_clear_feasible{false};
  double pass_traveled_m{};
  double remaining_lateral_transition_distance_m{};
  double remaining_pass_hold_distance_m{std::numeric_limits<double>::infinity()};
};

struct RuntimeRearClearPredictionResolution
{
  bool valid{false};
  bool checked{false};
  bool feasible{false};
  double required_rear_clear_pass_m{std::numeric_limits<double>::infinity()};
};

/// Convert the single execution-coupled runtime completion prediction into an
/// absolute Pass-origin rear-clear distance.  This resolver deliberately does
/// not perform another rollout: forward-completion admission, SafeSeparation
/// budgets and horizon extension must consume the same longitudinal model.
RuntimeRearClearPredictionResolution resolve_runtime_rear_clear_prediction(
  const RuntimeRearClearPredictionRequest & request) noexcept;

struct RearClearReplanWindowRequest
{
  bool prediction_checked{false};
  bool prediction_feasible{false};
  double required_rear_clear_pass_m{std::numeric_limits<double>::infinity()};
  double static_valid_until_pass_m{};
  double pass_traveled_m{};
  double revalidation_lead_distance_m{};
};

struct RearClearReplanWindowResolution
{
  bool valid{false};
  bool beyond_committed_horizon{false};
  bool replan_due{false};
  double remaining_committed_distance_m{};
};

/// Separate "rear clear lies beyond the current committed path" from "the
/// path is close enough to its end that replacement planning is due". This
/// prevents a long, still-validated Pass corridor from being discarded at
/// Pass entry merely because rear-clear needs a later extension.
RearClearReplanWindowResolution resolve_rear_clear_replan_window(
  const RearClearReplanWindowRequest & request) noexcept;

struct PassHorizonDecisionRequest
{
  bool enabled{false};
  bool pass_active{false};
  bool rear_clear_confirmed{false};
  bool return_corridor_available{false};
  /// A previously released Pass corridor now predicts a persistent body
  /// overlap. Revalidate the same side before the normal horizon margin is
  /// exhausted instead of waiting for the speed cap to be reapplied.
  bool predicted_overlap_replan_required{false};
  /// The admitted rear-clear point has entered the last distance window in
  /// which a bounded same-side replacement can still be planned. This is an
  /// early trigger; it must be disabled after the permitted replacement has
  /// already been committed.
  bool rear_clear_replan_required{false};
  /// A fresh rollout from the measured ego speed can extend the rear-clear
  /// distance without changing the committed side or lateral goal. Unlike a
  /// geometric extension, this remains available after extension_count has
  /// reached maximum_extension_count.
  bool longitudinal_refresh_available{false};
  bool short_horizon_safe{false};
  bool hold_active{false};
  double pass_traveled_m{};
  double pass_elapsed_sec{};
  double static_valid_until_pass_m{};
  double dynamic_valid_until_pass_m{};
  double dynamic_time_remaining_sec{};
  double absolute_distance_limit_m{std::numeric_limits<double>::infinity()};
  double absolute_time_limit_sec{std::numeric_limits<double>::infinity()};
  double revalidation_lead_distance_m{};
  double revalidation_lead_time_sec{};
  double hold_elapsed_sec{};
  double hold_traveled_m{};
  double hold_max_sec{};
  double hold_max_distance_m{};
  int extension_count{};
  int maximum_extension_count{};
};

PassHorizonAction resolve_pass_horizon_action(
  const PassHorizonDecisionRequest & request) noexcept;

enum class PassRefreshFailureReason
{
  None,
  TargetPredictionUnavailable,
  TargetDiscontinuous,
  CourseProgressRejected,
  ExecutionCorridorBlocked,
  PredictedOverlap,
  InvalidInput,
  AbsoluteBudgetExhausted,
  WallOrBodyFault,
  Other,
};

struct StoppedSidePassPredictionLeaseRequest
{
  bool enabled{false};
  bool pass_active{false};
  bool mission_path_frozen{false};
  PassRefreshFailureReason refresh_failure_reason{PassRefreshFailureReason::None};
  bool target_continuous{false};
  bool course_progress_accepted{false};
  bool execution_corridor_clear{false};
  bool current_body_footprints_separated{false};
  bool actual_wall_physical_contact{false};
  bool actual_wall_margin_blocked{false};
  bool actual_wall_sample_unavailable{false};
  bool emergency_brake{false};
  bool solver_recovery_active{false};
  double target_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_stopped_target_speed_mps{};
  double target_longitudinal_m{std::numeric_limits<double>::infinity()};
  double maximum_absolute_target_longitudinal_m{};
  double target_observation_age_sec{std::numeric_limits<double>::infinity()};
  double last_clear_prediction_age_sec{std::numeric_limits<double>::infinity()};
  double lease_elapsed_sec{};
  double lease_traveled_m{};
  double maximum_lease_sec{};
  double maximum_lease_distance_m{};
  double pass_elapsed_sec{};
  double pass_traveled_m{};
  double absolute_pass_time_limit_sec{std::numeric_limits<double>::infinity()};
  double absolute_pass_distance_limit_m{std::numeric_limits<double>::infinity()};
};

/// Keep a frozen same-side Pass through a short behavior/prediction
/// classification loss only for a recently observed stopped target beside the
/// ego. The lease never relaxes current footprint, wall, emergency, solver, or
/// immutable Pass budget gates. A controller must still discard and replan the
/// path after an external reverse maneuver changes the vehicle pose.
bool can_lease_stopped_side_pass_prediction(
  const StoppedSidePassPredictionLeaseRequest & request) noexcept;

const char * to_string(PassRefreshFailureReason reason) noexcept;

struct PassRefreshReplanGraceRequest
{
  bool enabled{false};
  bool pass_active{false};
  bool mission_path_frozen{false};
  PassRefreshFailureReason refresh_failure_reason{PassRefreshFailureReason::None};
  bool target_continuous{false};
  bool course_progress_accepted{false};
  bool fresh_target_prediction_available{false};
  bool short_horizon_safe{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool predicted_overlap_replan_required{false};
  bool execution_corridor_blocked{false};
  bool actual_wall_physical_contact{false};
  bool actual_wall_margin_blocked{false};
  bool actual_wall_sample_unavailable{false};
  bool emergency_brake{false};
  bool solver_recovery_active{false};
  double grace_elapsed_sec{};
  double grace_traveled_m{};
  double maximum_grace_sec{};
  double maximum_grace_distance_m{};
  double pass_elapsed_sec{};
  double pass_traveled_m{};
  double static_valid_until_pass_m{std::numeric_limits<double>::infinity()};
  double absolute_pass_time_limit_sec{std::numeric_limits<double>::infinity()};
  double absolute_pass_distance_limit_m{std::numeric_limits<double>::infinity()};
};

/// Preserve the currently admitted Pass path for a short, bounded replanning
/// window.  This never makes a failed replacement path executable: it only
/// consumes the still-valid prefix of the frozen path while replanning is
/// retried.  Current/predicted overlap and physical wall faults remain hard
/// boundaries.
bool can_hold_pass_during_refresh_replan(
  const PassRefreshReplanGraceRequest & request) noexcept;

struct StoppedPredictionLeaseSpeedRequest
{
  bool active{false};
  double current_speed_mps{};
  double lease_start_speed_mps{};
  double existing_target_velocity_reference_mps{
    std::numeric_limits<double>::infinity()};
  double maximum_speed_mps{std::numeric_limits<double>::infinity()};
};

struct StoppedPredictionLeaseSpeedResolution
{
  bool active{false};
  bool valid{false};
  double target_velocity_reference_mps{
    std::numeric_limits<double>::infinity()};
  double target_velocity_floor_mps{};
};

/// Own longitudinal intent while a stopped-side prediction lease is active.
/// The lease may preserve current forward progress, but must not add positive
/// acceleration or retain a higher full-attack reference while prediction is
/// unavailable.
StoppedPredictionLeaseSpeedResolution resolve_stopped_prediction_lease_speed(
  const StoppedPredictionLeaseSpeedRequest & request) noexcept;

struct PassContinuationPreflightPolicyRequest
{
  bool longitudinal_refresh{false};
  bool short_horizon_safe{false};
  bool target_observation_continuous{false};
  bool current_body_footprints_separated{false};
  bool predicted_body_footprint_available{false};
  bool predicted_body_footprint_sweep_separated{false};
  /// Use the shared debounce result, not a single prediction sample, when
  /// deciding whether semantic center separation must be restored.
  bool predicted_body_overlap_confirmed{false};
};

struct PassContinuationPreflightPolicyResolution
{
  bool footprint_continuation_active{false};
  bool enforce_target_center_separation{true};
  bool enforce_outer_role_continuity{true};
  bool include_return_path{false};
};

/// Replace semantic center-distance and inside/outside labels with measured
/// body-footprint separation only when refreshing the longitudinal extent of
/// an already-admitted, fixed-lateral Pass path. A geometric replacement keeps
/// the original target/outer-role constraints. Every active-Pass continuation
/// validates only the same-side path through rear-clear; Return is regenerated
/// and validated from the current state after rear-clear. Initial admission is
/// outside this policy and keeps the full ShiftOut/Pass/Return preflight.
PassContinuationPreflightPolicyResolution resolve_pass_continuation_preflight_policy(
  const PassContinuationPreflightPolicyRequest & request) noexcept;

struct PassOuterHorizonSample
{
  double path_distance_m{};
  double reference_curvature_radpm{};
};

struct PassOuterHorizonRequest
{
  bool enabled{false};
  /// Initial admission infers an outer strategy from the first significant
  /// curve. A replacement sets this when the original mission already owns an
  /// outer strategy, even if the replacement starts on a straight.
  bool outer_strategy_committed{false};
  bool infer_outer_strategy{false};
  int pass_side_sign{0};
  double significant_curvature_radpm{};
  double validation_distance_m{};
  std::vector<PassOuterHorizonSample> samples;
};

struct PassOuterHorizonResolution
{
  bool valid{false};
  bool feasible{false};
  bool outer_strategy{false};
  bool role_reversal{false};
  double first_significant_curve_distance_m{std::numeric_limits<double>::infinity()};
  double first_role_reversal_distance_m{std::numeric_limits<double>::infinity()};
};

/// Keep a committed outside pass outside until its predicted rear-clear point.
/// Intentional inside attacks are not rejected. Initial admission may infer an
/// outside strategy from the first significant curve; replacement planning
/// carries the already-committed strategy explicitly.
PassOuterHorizonResolution evaluate_pass_outer_horizon(
  const PassOuterHorizonRequest & request) noexcept;

enum class PassSideCourseRole
{
  Unknown,
  Inner,
  Outer,
};

const char * to_string(PassSideCourseRole role) noexcept;

struct PassSideRearClearRoleRequest
{
  int pass_side_sign{0};
  double significant_curvature_radpm{};
  double predicted_rear_clear_distance_m{};
  double reserve_distance_m{};
  std::vector<PassOuterHorizonSample> samples;
};

struct PassSideRearClearRoleResolution
{
  bool valid{false};
  PassSideCourseRole entry_role{PassSideCourseRole::Unknown};
  PassSideCourseRole rear_clear_role{PassSideCourseRole::Unknown};
  bool outer_to_inner_before_rear_clear{false};
  bool inner_to_outer_at_rear_clear{false};
  double first_role_reversal_distance_m{std::numeric_limits<double>::infinity()};
};

/// Classify a fixed Frenet side over the predicted rear-clear horizon. The
/// reserve includes a short section after rear-clear so a curve-sign change at
/// the pass boundary is visible before committing. An outer-to-inner change is
/// the expensive case: preserving an outside strategy would require crossing
/// the track while the target may still be alongside. An inner-to-outer change
/// needs no lateral handoff and is therefore retained as a viable exit path.
PassSideRearClearRoleResolution evaluate_pass_side_rear_clear_role(
  const PassSideRearClearRoleRequest & request) noexcept;

struct ContinuousOuterReplanRequest
{
  bool enabled{false};
  int current_side_sign{0};
  double significant_curvature_radpm{};
  double lookahead_distance_m{};
  double minimum_opposite_curve_distance_m{};
  std::vector<PassOuterHorizonSample> samples;
};

struct ContinuousOuterReplanResolution
{
  bool valid{false};
  bool replan_required{false};
  int desired_outer_side_sign{0};
  double first_opposite_curve_distance_m{std::numeric_limits<double>::infinity()};
  double confirmed_opposite_curve_distance_m{};
};

/// Find a sustained upcoming curve whose outside lies on the opposite side of
/// the currently committed Pass corridor. Straights retain the last observed
/// role, while short opposite-curvature noise is rejected by the continuous
/// distance requirement. This resolver requests only a replan; wall, target
/// and longitudinal crossing safety remain caller-owned hard gates.
ContinuousOuterReplanResolution evaluate_continuous_outer_replan(
  const ContinuousOuterReplanRequest & request) noexcept;

struct ScheduledOuterTransitionRequest
{
  bool enabled{false};
  bool outer_strategy{false};
  bool role_reversal{false};
  int current_side_sign{0};
  /// All distances use the Pass origin, after the initial ShiftOut.
  double body_clear_distance_m{std::numeric_limits<double>::infinity()};
  double role_reversal_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_shift_distance_m{0.5};
  double maximum_shift_distance_m{0.5};
};

struct ScheduledOuterTransitionResolution
{
  bool valid{false};
  bool feasible{false};
  bool transition_required{false};
  int desired_side_sign{0};
  double start_distance_m{std::numeric_limits<double>::infinity()};
  double deadline_distance_m{std::numeric_limits<double>::infinity()};
  double available_shift_distance_m{};
};

/// Convert a full-mission outside-role reversal into a deterministic Pass
/// handoff window. The handoff cannot start before predicted body clearance
/// and must leave at least the minimum lateral-shift distance before the old
/// side becomes inside. Runtime wall/target/acceleration preflight remains the
/// final authority for the replacement corridor.
ScheduledOuterTransitionResolution resolve_scheduled_outer_transition(
  const ScheduledOuterTransitionRequest & request) noexcept;

struct ScheduledOuterTransitionRuntimeBudgetRequest
{
  double remaining_window_distance_m{};
  double admission_nominal_shift_distance_m{};
  double configured_maximum_shift_distance_m{};
  double remaining_absolute_pass_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_shift_distance_m{0.5};
  double remaining_pass_reserve_m{0.5};
};

struct ScheduledOuterTransitionRuntimeBudgetResolution
{
  bool valid{false};
  bool feasible{false};
  double admission_nominal_shift_distance_m{};
  double available_shift_distance_m{};
};

/// Resolve the live maximum distance for a scheduled outside-role handoff.
/// The admission distance is retained only for diagnostics: runtime speed may
/// require a longer ramp, so the remaining frozen window is the governing
/// limit. The continuation planner still performs the acceleration and full
/// path feasibility checks before committing.
ScheduledOuterTransitionRuntimeBudgetResolution
resolve_scheduled_outer_transition_runtime_budget(
  const ScheduledOuterTransitionRuntimeBudgetRequest & request) noexcept;

struct FrozenOuterTransitionGoalRequest
{
  int desired_side_sign{0};
  double source_goal_m{};
  double feasible_lower_m{};
  double feasible_upper_m{};
  double minimum_role_offset_m{0.05};
  double maximum_lateral_adjustment_m{std::numeric_limits<double>::infinity()};
};

struct FrozenOuterTransitionGoalResolution
{
  bool valid{false};
  bool feasible{false};
  double goal_m{std::numeric_limits<double>::quiet_NaN()};
  double lateral_adjustment_m{std::numeric_limits<double>::infinity()};
};

/// Freeze the next outside-role goal without using a later target-lateral
/// observation. Mirroring the admitted goal preserves a minimum-motion line;
/// wall bounds and the requested Frenet role remain hard constraints.
FrozenOuterTransitionGoalResolution resolve_frozen_outer_transition_goal(
  const FrozenOuterTransitionGoalRequest & request) noexcept;

struct DynamicCorridorGoalRequest
{
  bool enabled{false};
  bool pass_active{false};
  int pass_side_sign{0};
  double current_goal_m{};
  double desired_goal_m{};
  double feasible_lower_m{};
  double feasible_upper_m{};
  double minimum_role_offset_m{0.05};
  double minimum_adjustment_m{0.05};
  double maximum_adjustment_m{0.20};
};

struct DynamicCorridorGoalResolution
{
  bool valid{false};
  bool feasible{false};
  bool update_required{false};
  double goal_m{std::numeric_limits<double>::quiet_NaN()};
  double lateral_adjustment_m{std::numeric_limits<double>::infinity()};
};

/// Bound a live same-side Pass goal update. The side is immutable here: this
/// helper only permits a small receding-horizon correction inside the current
/// wall-feasible interval and suppresses sub-threshold goal chatter.
DynamicCorridorGoalResolution resolve_dynamic_corridor_goal(
  const DynamicCorridorGoalRequest & request) noexcept;

struct SameSideExtensionCommitRequest
{
  bool pass_or_hold_active{false};
  bool target_matches{false};
  bool side_matches{false};
  bool replacement_path_valid{false};
  std::uint64_t source_generation{};
  std::uint64_t current_generation{};
  double planner_generated_at_sec{};
  double commit_now_sec{};
  double planner_result_max_age_sec{};
  double prediction_expiry_sec{};
  double current_effective_valid_until_pass_m{};
  double current_pass_hold_distance_m{};
  bool require_pass_distance_advance{true};
  double replacement_static_valid_until_pass_m{};
  double replacement_dynamic_valid_until_pass_m{};
  double replacement_pass_hold_distance_m{};
  double absolute_pass_distance_limit_m{std::numeric_limits<double>::infinity()};
  double current_goal_lateral_m{};
  double replacement_goal_lateral_m{};
  double maximum_lateral_adjustment_m{std::numeric_limits<double>::infinity()};
};

enum class SameSideExtensionCommitReason
{
  Accepted,
  PassInactive,
  TargetMismatch,
  SideMismatch,
  ReplacementPathInvalid,
  GenerationMismatch,
  InvalidInput,
  PlannerResultStale,
  PredictionExpired,
  AbsoluteDistanceExceeded,
  PassDistanceNotAdvanced,
  StaticCoverageInsufficient,
  LateralAdjustmentExceeded,
};

struct SameSideExtensionCommitResolution
{
  bool accepted{false};
  SameSideExtensionCommitReason reason{SameSideExtensionCommitReason::InvalidInput};
};

/// Validate every identity, freshness and range condition immediately before
/// an already-computed same-side replacement mission is committed atomically.
SameSideExtensionCommitResolution evaluate_same_side_extension_commit(
  const SameSideExtensionCommitRequest & request) noexcept;

bool can_commit_same_side_extension(
  const SameSideExtensionCommitRequest & request) noexcept;

const char * to_string(SameSideExtensionCommitReason reason) noexcept;

enum class MissionTotalBudgetAction
{
  Inactive,
  Keep,
  Return,
  Abort,
};

struct MissionTotalBudgetRequest
{
  bool enabled{false};
  bool mission_active{false};
  double elapsed_sec{};
  double maximum_duration_sec{std::numeric_limits<double>::infinity()};
  bool rear_clear_confirmed{false};
  bool return_corridor_available{false};
};

struct MissionTotalBudgetResolution
{
  MissionTotalBudgetAction action{MissionTotalBudgetAction::Inactive};
  bool expired{false};
};

/// Keep the original same-target Mission clock when one exists, otherwise
/// start it at the first active execution phase.  Some dedicated entry paths
/// do not freeze a normal Mission candidate before ShiftOut.
double resolve_mission_total_start_sec(
  bool mission_active, double current_start_sec, double now_sec) noexcept;

/// Limit the complete same-target mission, including ShiftOut, Pass and
/// FollowPrepare. Once expired, rear-clear missions may return; all other
/// missions must leave the pass attempt instead of resetting a local clock.
MissionTotalBudgetResolution resolve_mission_total_budget(
  const MissionTotalBudgetRequest & request) noexcept;

struct MissionCompletionDeadlineExtensionRequest
{
  bool enabled{false};
  bool same_side_pass_continuation{false};
  bool completion_prediction_valid{false};
  double mission_elapsed_sec{};
  double base_maximum_duration_sec{};
  double prior_extension_sec{};
  double predicted_completion_time_sec{};
  double completion_reserve_sec{};
  double maximum_total_extension_sec{};
};

struct MissionCompletionDeadlineExtensionResolution
{
  bool valid{false};
  bool extended{false};
  double extension_sec{};
  double effective_maximum_duration_sec{};
  double remaining_before_extension_sec{};
  double required_remaining_sec{};
};

/// A fresh same-side Pass continuation or an executing Pass may receive an
/// updated finite-horizon completion prediction near the original Mission
/// deadline. Extend only enough to cover that prediction, with a cumulative
/// cap so repeated replans cannot reset the same-target Mission indefinitely.
MissionCompletionDeadlineExtensionResolution
resolve_mission_completion_deadline_extension(
  const MissionCompletionDeadlineExtensionRequest & request) noexcept;

enum class SafeSeparationAction
{
  Inactive,
  KeepSameSide,
  Return,
  RecoverBehind,
  Abort,
};

enum class SafeSeparationReason
{
  None,
  InvalidInput,
  RearClear,
  ShortHorizonUnsafe,
  LocalTimeLimit,
  LocalDistanceLimit,
  AbsoluteTimeLimit,
  AbsoluteDistanceLimit,
  TargetClearAhead,
  ProgressExtension,
  DynamicCompletionExtension,
  RearwardProgressTimeGrace,
  SideBySideRearClearCompletion,
  RearwardProgressLossDisengagement,
  RearwardProgressLossDisengagementTimeout,
  TargetAheadPassContinuation,
};

struct SpeedPreservingTacticalRevalidationRequest
{
  bool enabled{false};
  bool safe_separation_active{false};
  bool pass_committed{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool execution_corridor_blocked{false};
  bool hard_fault{false};
  bool rear_clear_confirmed{false};
  double target_longitudinal_m{};
  double maximum_absolute_longitudinal_m{};
  double elapsed_sec{};
  double traveled_m{};
  double maximum_duration_sec{};
  double maximum_distance_m{};
};

struct SpeedPreservingTacticalRevalidationResolution
{
  bool active{false};
  double remaining_sec{};
  double remaining_distance_m{};
};

/// Admit a bounded speed-preserving Pass lease while current bodies and the
/// wall corridor remain clear. Predicted sweep separation is deliberately not
/// required: bridging a short prediction-only overlap is the purpose of this
/// lease. Hard physical/runtime guards remain fail closed.
SpeedPreservingTacticalRevalidationResolution
resolve_speed_preserving_tactical_revalidation(
  const SpeedPreservingTacticalRevalidationRequest & request) noexcept;

enum class TargetAheadPassContinuationAction
{
  Inactive,
  HoldSameSide,
  GuardClosingSpeed,
  ForwardEscape,
};

struct TargetAheadPassContinuationRequest
{
  bool enabled{false};
  bool safe_separation_active{false};
  bool pass_committed{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool execution_corridor_blocked{true};
  bool hard_fault{false};
  bool rear_clear_confirmed{false};
  bool absolute_budget_available{false};
  double target_longitudinal_m{};
  double maximum_front_distance_m{};
  double closing_speed_guard_distance_m{};
};

/// Classify a committed Pass whose locked target remains ahead. A physically
/// and predictively clear corridor may continue forward; prediction-only
/// overlap retains speed on the same side while replanning. Current-body,
/// wall/corridor and runtime hard faults never receive this lease.
TargetAheadPassContinuationAction resolve_target_ahead_pass_continuation(
  const TargetAheadPassContinuationRequest & request) noexcept;

struct RobustOvertakeClearanceRequest
{
  bool enabled{false};
  double ego_speed_mps{};
  double absolute_curvature_radpm{};
  double physical_target_center_separation_m{};
  double configured_target_center_separation_m{};
  double target_surface_base_m{};
  double target_speed_gain_sec{};
  double target_curvature_gain_m2{};
  double target_surface_max_m{};
  double hard_wall_clearance_m{};
  double wall_base_reserve_m{};
  double wall_speed_gain_sec{};
  double wall_curvature_gain_m2{};
  double wall_reserve_max_m{};
};

struct RobustOvertakeClearanceResolution
{
  bool valid{false};
  double target_surface_clearance_m{};
  double target_center_separation_m{};
  double wall_tracking_reserve_m{};
  double wall_planning_clearance_m{};
};

/// Resolve planning clearance independently from the physical contact guard.
/// The speed/curvature additions are bounded so a noisy sample cannot grow an
/// unbounded exclusion envelope.
RobustOvertakeClearanceResolution resolve_robust_overtake_clearance(
  const RobustOvertakeClearanceRequest & request) noexcept;

struct SafeTrajectoryPrefixLeaseRequest
{
  bool enabled{false};
  bool safe_separation_active{false};
  bool pass_committed{false};
  bool mission_path_frozen{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool execution_corridor_blocked{true};
  bool hard_fault{false};
  bool rear_clear_confirmed{false};
  bool fresh_forward_progress{false};
  double target_longitudinal_m{std::numeric_limits<double>::infinity()};
  double maximum_front_distance_m{};
  double validated_prefix_remaining_m{};
  double minimum_validated_prefix_m{};
  double absolute_elapsed_sec{};
  double absolute_traveled_m{};
  double absolute_maximum_duration_sec{std::numeric_limits<double>::infinity()};
  double absolute_maximum_distance_m{std::numeric_limits<double>::infinity()};
};

/// Retain the currently validated lateral trajectory while a fresh complete
/// Mission is rebuilt. This does not lease an uncertain prediction: current
/// bodies, the predicted sweep, the execution corridor and a statically
/// validated path prefix must all remain clear. Absolute Pass bounds remain
/// immutable and are checked here as well as by SafeSeparation.
bool can_retain_safe_trajectory_prefix(
  const SafeTrajectoryPrefixLeaseRequest & request) noexcept;

struct LatchedForwardEscapeContinuationRequest
{
  bool enabled{false};
  bool safe_separation_active{false};
  bool pass_committed{false};
  bool forward_completion_latched{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool execution_corridor_blocked{false};
  bool hard_fault{false};
  bool rear_clear_confirmed{false};
};

/// Preserve an already committed forward escape across a prediction-only
/// overlap. This is intentionally not time-limited like tactical
/// revalidation: the existing SafeSeparation local and absolute Mission
/// budgets remain the bounds. Every current physical/runtime hard guard stays
/// fail closed.
bool can_continue_latched_forward_escape(
  const LatchedForwardEscapeContinuationRequest & request) noexcept;

struct TacticalRevalidationReturnRequest
{
  bool enabled{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool return_corridor_available{false};
  bool execution_corridor_blocked{false};
  bool hard_fault{false};
  double target_longitudinal_m{};
  double minimum_front_distance_m{};
};

/// A target that is confirmed clear ahead may leave the committed Pass through
/// a smooth Return instead of dropping into FollowPrepare, but only with
/// current and predicted physical separation and a clear Return corridor.
bool can_return_from_tactical_revalidation(
  const TacticalRevalidationReturnRequest & request) noexcept;

struct ConfirmedTargetClearReturnRequest
{
  bool enabled{false};
  bool target_clear_ahead_confirmed{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool return_corridor_available{false};
  bool hard_fault{false};
  double target_longitudinal_m{};
  double minimum_front_distance_m{};
};

/// Once SafeSeparation has already confirmed that the target remains clear
/// ahead, leave the failed Pass through a physically valid Return without
/// requiring the old pass-side prediction to stay clear.  Current overlap,
/// Return-corridor blockage and hard faults still fail closed.
bool can_return_after_confirmed_target_clear(
  const ConfirmedTargetClearReturnRequest & request) noexcept;

struct CommittedPassForwardCompletionRequest
{
  bool enabled{false};
  bool pass_active{false};
  bool minimum_motion_corridor_active{false};
  bool prior_front_cap_release_active{false};
  bool target_seen{false};
  bool target_continuity_valid{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool current_body_footprint_overlap_confirmed{true};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool predicted_body_footprint_overlap_confirmed{true};
  bool actual_wall_contact{false};
  bool wall_sample_unavailable{false};
  bool target_pass_side_intrusion{false};
  bool emergency_brake{false};
  bool solver_recovery_active{false};
  double target_longitudinal_m{};
  double maximum_front_distance_m{};
  /// Live result from resolve_overtake_kinematic_rollout and
  /// resolve_overtake_dynamic_pass_distance. Admission and runtime must use
  /// the same acceleration, control-delay and path-speed-cap model.
  bool completion_prediction_valid{false};
  bool completion_rear_clear_feasible{false};
  double predicted_required_forward_distance_m{
    std::numeric_limits<double>::infinity()};
  double predicted_completion_time_sec{std::numeric_limits<double>::infinity()};
  double maximum_completion_distance_m{};
  bool already_latched{false};
  /// Tactical no-return has already been crossed even if the dedicated
  /// forward-completion latch could not yet be acquired.
  bool side_by_side_committed{false};
};

struct CommittedPassForwardCompletionResolution
{
  bool active{false};
  bool current_overlap_grace_active{false};
  bool predicted_overlap_grace_active{false};
  bool rear_clear_distance_feasible{false};
  double required_forward_distance_m{std::numeric_limits<double>::infinity()};
  double required_completion_time_sec{std::numeric_limits<double>::infinity()};
};

/// Once an already released minimum-motion Pass reaches the bounded
/// side-by-side window, freeze the committed side and finish longitudinally
/// when the predicted sweep remains separated or a latched completion is still
/// inside the bounded predicted-overlap confirmation window, and the estimated
/// acceleration-limited rear-clear completion distance fits inside the
/// bounded local escape.
/// A single current-overlap sample may share the existing confirmation grace;
/// confirmed predicted/current overlap and all physical/runtime guards remain
/// fail closed. Initial acquisition never receives prediction grace.
CommittedPassForwardCompletionResolution resolve_committed_pass_forward_completion(
  const CommittedPassForwardCompletionRequest & request) noexcept;

struct PassCompletionRolloutSpeedRequest
{
  bool enabled{false};
  bool pass_active{false};
  bool full_speed_forward_escape_enabled{false};
  bool front_cap_released{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool execution_corridor_blocked{false};
  bool hard_fault{false};
  double nominal_closing_speed_mps{};
  double target_speed_mps{};
  double maximum_ego_speed_mps{};
};

struct PassCompletionRolloutSpeedResolution
{
  bool valid{false};
  bool full_speed_coupled{false};
  double closing_speed_mps{};
  double ego_speed_target_mps{};
};

/// Keep the runtime rear-clear rollout consistent with the longitudinal
/// policy that will actually execute it. Full-speed coupling is available only
/// on an already separated, prediction-clear Pass; every hard guard falls
/// back to the nominal Mission closing speed.
PassCompletionRolloutSpeedResolution resolve_pass_completion_rollout_speed(
  const PassCompletionRolloutSpeedRequest & request) noexcept;

struct DynamicCompletionExtensionRequest
{
  bool enabled{false};
  bool forward_escape_allowed{false};
  bool fresh_forward_progress{false};
  bool forward_completion_latched{false};
  double required_forward_distance_m{std::numeric_limits<double>::infinity()};
  double forward_speed_mps{};
  double absolute_elapsed_sec{};
  double absolute_traveled_m{};
  double absolute_maximum_duration_sec{std::numeric_limits<double>::infinity()};
  double absolute_maximum_distance_m{std::numeric_limits<double>::infinity()};
};

struct DynamicCompletionExtensionResolution
{
  bool allowed{false};
  double required_completion_time_sec{std::numeric_limits<double>::infinity()};
  double remaining_absolute_time_sec{};
  double remaining_absolute_distance_m{};
};

/// Admit another local forward-completion window only when the live rear-clear
/// estimate still fits inside both immutable absolute Pass bounds. The caller
/// owns all footprint/corridor guards and reports them through
/// forward_escape_allowed/fresh_forward_progress.
DynamicCompletionExtensionResolution resolve_dynamic_completion_extension(
  const DynamicCompletionExtensionRequest & request) noexcept;

struct RearwardPassCompletionContextRequest
{
  bool pass_active{false};
  PassCommitStage commit_stage{PassCommitStage::Selectable};
  bool target_seen{false};
  bool target_matches{false};
  bool target_continuity_valid{false};
  double target_longitudinal_m{std::numeric_limits<double>::infinity()};
  bool forward_completion_latched{false};
  bool fresh_forward_progress{false};
  bool current_body_footprints_separated{false};
  bool execution_corridor_blocked{true};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
};

struct RearwardPassCompletionContextResolution
{
  bool rearward_target{false};
  bool contact_tail_eligible{false};
  bool separated_tail_candidate{false};
  bool separated_tail_physical_safe{false};
  bool separated_tail_progress_allowed{false};
};

/// Classify the common rear-clear completion context shared by bounded side
/// contact and SafeSeparation. Contact-specific lateral/velocity/time guards
/// and SafeSeparation budgets remain in their owning policies.
RearwardPassCompletionContextResolution resolve_rearward_pass_completion_context(
  const RearwardPassCompletionContextRequest & request) noexcept;

struct PassShortHorizonGuardRequest
{
  bool hard_guard_safe{false};
  bool predictive_guard_safe{false};
  bool forward_completion_latched{false};
  bool current_body_footprints_separated{false};
  bool execution_corridor_blocked{true};
  bool fresh_forward_progress{false};
  double predictive_guard_loss_elapsed_sec{std::numeric_limits<double>::infinity()};
  double maximum_prediction_grace_sec{};
  /// A committed target is already at or behind ego and current/predicted
  /// body geometry plus the execution corridor remain physically clear. This
  /// is independent of whether the completion rollout fits the normal budget.
  bool side_by_side_rearward_completion_safe{false};
  /// The same rearward completion is tactically eligible based on current body
  /// geometry and corridor state. While the existing bounded prediction-only
  /// grace is active, this keeps tail ownership without treating an
  /// unconfirmed predicted overlap as physically clear.
  bool side_by_side_rearward_completion_prediction_grace_allowed{false};
  /// The target is already rearward and measured longitudinal progress is
  /// fresh. This admits completion beyond the prediction-only timer while
  /// current bodies and the execution corridor remain physically clear.
  bool side_by_side_rearward_progress_completion_allowed{false};
};

struct PassShortHorizonGuardResolution
{
  bool safe{false};
  bool prediction_grace_active{false};
  bool rearward_completion_active{false};
  bool rearward_completion_prediction_grace_active{false};
  bool rearward_completion_progress_active{false};
};

/// Keep an already-latched forward completion alive through a bounded
/// prediction-only dropout. Physical/runtime faults are represented by
/// hard_guard_safe and can never receive grace. The caller owns the loss timer
/// and measured forward-progress observation. Once a side-by-side committed
/// target is rearward, fresh measured progress may own the bounded rear-clear
/// tail even if the predictive sweep remains conservative. Current-body and
/// corridor guards remain mandatory.
PassShortHorizonGuardResolution resolve_pass_short_horizon_guard(
  const PassShortHorizonGuardRequest & request) noexcept;

/// Return cumulative time spent in Pass. Paused phases contribute only the
/// already accumulated value; an active segment contributes now-start.
double resolve_active_pass_elapsed(
  double accumulated_sec, bool pass_active,
  double active_segment_start_sec, double now_sec) noexcept;

struct SafeSeparationRequest
{
  bool enabled{false};
  bool active{false};
  bool short_horizon_safe{false};
  bool target_seen{false};
  bool rear_clear_confirmed{false};
  bool return_corridor_available{false};
  double target_longitudinal_m{};
  double target_speed_mps{};
  double speed_delta_mps{};
  double maximum_ego_speed_mps{std::numeric_limits<double>::infinity()};
  double front_clear_distance_m{};
  double front_clear_elapsed_sec{};
  double front_clear_confirm_sec{};
  double elapsed_sec{};
  double traveled_m{};
  double maximum_duration_sec{};
  double maximum_distance_m{};
  double ego_speed_mps{};
  bool forward_escape_allowed{false};
  bool target_ahead_hold_allowed{false};
  double target_ahead_hold_maximum_closing_speed_mps{
    std::numeric_limits<double>::infinity()};
  double forward_escape_max_front_distance_m{};
  double absolute_elapsed_sec{};
  double absolute_traveled_m{};
  double absolute_maximum_duration_sec{std::numeric_limits<double>::infinity()};
  double absolute_maximum_distance_m{std::numeric_limits<double>::infinity()};
  bool forward_progress_extension_allowed{false};
  bool forward_completion_latched{false};
  bool dynamic_completion_extension_allowed{false};
  bool full_speed_forward_escape_enabled{false};
  bool rearward_progress_time_grace_enabled{false};
  bool fresh_forward_progress{false};
  PassCommitStage commit_stage{PassCommitStage::Selectable};
  /// Physical rear-clear continuation admitted after tactical no-return. The
  /// normal local window becomes soft, but absolute Pass bounds remain hard.
  bool side_by_side_rearward_completion_allowed{false};
  /// A bounded same-side disengagement may replace a failed forward escape
  /// after tactical no-return. It never authorizes a lateral Return across the
  /// locked target; it lets the target clear ahead before Mission revalidation.
  bool rearward_progress_loss_disengage_enabled{false};
  bool rearward_progress_loss_disengage_active{false};
  bool rearward_progress_loss_physical_hold_safe{false};
  double rearward_progress_loss_progress_age_sec{
    std::numeric_limits<double>::infinity()};
  double rearward_progress_loss_stale_sec{};
  double rearward_progress_loss_regression_m{};
  double rearward_progress_loss_minimum_regression_m{};
  double rearward_progress_loss_disengage_elapsed_sec{};
  double rearward_progress_loss_disengage_max_sec{};
  double rearward_progress_loss_disengage_speed_delta_mps{};
};

struct SafeSeparationResolution
{
  SafeSeparationAction action{SafeSeparationAction::Inactive};
  double target_velocity_reference_mps{std::numeric_limits<double>::infinity()};
  double signed_closing_speed_mps{};
  bool forward_escape_active{false};
  bool full_speed_forward_escape_active{false};
  bool progress_extension_requested{false};
  SafeSeparationReason reason{SafeSeparationReason::None};
};

/// Keep the committed pass side while creating longitudinal separation. An
/// explicitly authorized forward escape chases a target inside its bounded
/// front window; otherwise a target ahead is allowed to pull away. A target
/// behind is driven farther rearward. Lateral Return/Recovery is selected only
/// after separation or a configured bound. An already active forward escape
/// may finish its current local window after the overall Pass bound is reached,
/// but may not re-arm that local window beyond the overall bound.
SafeSeparationResolution resolve_safe_separation(
  const SafeSeparationRequest & request) noexcept;

struct SafeSeparationTacticalReselectRequest
{
  bool enabled{false};
  bool safe_separation_active{false};
  bool forward_escape_allowed{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool execution_corridor_blocked{true};
  bool hard_fault{false};
  bool rear_clear_confirmed{false};
  double target_longitudinal_m{};
  double minimum_front_distance_m{};
};

/// Re-open lateral planning only after a failed SafeSeparation forward escape
/// has left the target clearly ahead.  This never authorizes side-by-side
/// crossing: current and predicted body geometry, corridor state and runtime
/// hard guards must all remain clear.
bool can_reselect_from_safe_separation(
  const SafeSeparationTacticalReselectRequest & request) noexcept;

enum class SoftMissionAbortAction
{
  Recovery,
  SpeedPreservingFollow,
};

struct SoftMissionAbortRequest
{
  bool soft_failure{false};
  bool hard_fault{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool execution_corridor_blocked{true};
  bool rear_clear_confirmed{false};
  double target_longitudinal_m{};
  double minimum_front_distance_m{};
};

/// Leave a failed, physically clear Mission without applying the Recovery
/// velocity cap. Hard faults and uncertain geometry remain fail closed.
SoftMissionAbortAction resolve_soft_mission_abort(
  const SoftMissionAbortRequest & request) noexcept;

struct MissionAlignedSafeSeparationBudgetRequest
{
  bool enabled{false};
  double configured_maximum_duration_sec{};
  double configured_maximum_distance_m{};
  double predicted_rear_clear_pass_m{std::numeric_limits<double>::infinity()};
  double current_pass_traveled_m{};
  double completion_distance_margin_m{};
  double completion_time_margin_sec{};
  double forward_speed_mps{};
  double absolute_elapsed_sec{};
  double absolute_traveled_m{};
  double absolute_maximum_duration_sec{std::numeric_limits<double>::infinity()};
  double absolute_maximum_distance_m{std::numeric_limits<double>::infinity()};
};

struct MissionAlignedSafeSeparationBudgetResolution
{
  bool valid{false};
  bool mission_aligned{false};
  double maximum_duration_sec{};
  double maximum_distance_m{};
};

/// Freeze a local SafeSeparation window which can cover the Mission's current
/// rear-clear estimate without crossing the immutable Pass limits. Invalid or
/// unavailable Mission estimates retain the configured local window.
MissionAlignedSafeSeparationBudgetResolution resolve_mission_aligned_safe_separation_budget(
  const MissionAlignedSafeSeparationBudgetRequest & request) noexcept;

struct RuntimeSafeSeparationBudgetRefreshRequest
{
  bool enabled{false};
  bool safe_separation_active{false};
  bool runtime_prediction_valid{false};
  bool runtime_rear_clear_feasible{false};
  double local_elapsed_sec{};
  double local_traveled_m{};
  double current_maximum_duration_sec{};
  double current_maximum_distance_m{};
  /// Distance and time still required from the current vehicle state.
  double runtime_remaining_duration_sec{};
  double runtime_remaining_distance_m{};
  double absolute_elapsed_sec{};
  double absolute_traveled_m{};
  double absolute_maximum_duration_sec{std::numeric_limits<double>::infinity()};
  double absolute_maximum_distance_m{std::numeric_limits<double>::infinity()};
};

struct RuntimeSafeSeparationBudgetRefreshResolution
{
  bool valid{false};
  bool refreshed{false};
  double maximum_duration_sec{};
  double maximum_distance_m{};
};

/// Grow an active SafeSeparation window to cover a fresh runtime rear-clear
/// rollout. The window is measured from the original SafeSeparation start and
/// therefore includes the already consumed local time/distance. It is never
/// restarted and never exceeds the immutable absolute Pass limits.
RuntimeSafeSeparationBudgetRefreshResolution refresh_runtime_safe_separation_budget(
  const RuntimeSafeSeparationBudgetRefreshRequest & request) noexcept;

struct RecoverableSideContactRequest
{
  bool enabled{false};
  bool pass_active{false};
  bool target_seen{false};
  bool target_continuity_valid{false};
  bool current_body_overlap_confirmed{false};
  /// Near contact only arms impact detection in the caller. It must never
  /// acquire ContactContinuation authority by itself.
  bool near_contact_prearmed{false};
  /// A bounded ego-speed collapse immediately after near-contact Prearm.
  bool impact_event_confirmed{false};
  /// Short post-admission bridge for one-cycle overlap/contact evidence loss.
  bool evidence_dropout_held{false};
  /// Prior-cycle classifier state. This may relax only the lateral-velocity
  /// release threshold and admit the bounded evidence-dropout bridge; it never
  /// bypasses target, heading, wall, geometry, progress or time guards.
  bool previously_active{false};
  bool relative_heading_valid{false};
  double relative_heading_rad{};
  bool wall_clearance_sufficient{false};
  int pass_side_sign{};
  double target_longitudinal_m{};
  double relative_lateral_m{};
  double longitudinal_closing_speed_mps{};
  bool relative_lateral_velocity_valid{false};
  double relative_lateral_velocity_mps{};
  double ego_speed_mps{};
  double contact_elapsed_sec{};
  bool fresh_forward_progress{false};
  bool forward_completion_latched{false};
  PassCommitStage commit_stage{PassCommitStage::Selectable};
  double maximum_duration_sec{};
  double rearward_completion_maximum_duration_sec{};
  double initial_progress_grace_sec{};
  double maximum_absolute_longitudinal_m{};
  double minimum_absolute_lateral_m{};
  double maximum_longitudinal_closing_speed_mps{};
  double maximum_absolute_lateral_velocity_mps{};
  double maximum_release_absolute_lateral_velocity_mps{};
  double minimum_ego_speed_mps{};
  double maximum_absolute_relative_heading_rad{};
  double body_longitudinal_clearance_m{};
  double body_lateral_clearance_m{};
  double lateral_separation_bias_m{};
};

struct RecoverableSideContactResolution
{
  bool active{false};
  bool initial_progress_grace_active{false};
  bool rearward_completion_active{false};
  bool impact_event_used{false};
  bool evidence_dropout_active{false};
  bool lateral_velocity_hysteresis_active{false};
  bool side_contact_normal{false};
  double lateral_separation_bias_m{};
};

/// Treat only a bounded, progressing side contact as a competition-simulation
/// Pass continuation. Once forward completion is latched and the target moves
/// behind, a separately bounded tail may bridge the remaining body overlap to
/// the existing rear-clear policy. Frontal geometry, target discontinuity and
/// stalled contact remain fail-closed in the caller. Near contact is only a
/// Prearm input and cannot activate this classifier. A previously admitted
/// contact uses separate lateral-velocity and evidence-dropout release guards
/// so geometry chatter cannot revoke Pass ownership for one cycle.
RecoverableSideContactResolution resolve_recoverable_side_contact(
  const RecoverableSideContactRequest & request) noexcept;

struct WallBoundedContactSeparationRequest
{
  bool active{false};
  int pass_side_sign{};
  double base_goal_m{};
  double requested_bias_m{};
  bool feasible_interval_available{false};
  double feasible_lower_m{};
  double feasible_upper_m{};
};

struct WallBoundedContactSeparationResolution
{
  bool valid{false};
  bool wall_limited{false};
  double requested_signed_bias_m{};
  double applied_signed_bias_m{};
  double goal_m{};
};

/// Move a committed Pass goal away from the target only within the wall-safe
/// interval. An invalid or unavailable interval keeps the original goal.
WallBoundedContactSeparationResolution resolve_wall_bounded_contact_separation(
  const WallBoundedContactSeparationRequest & request) noexcept;

const char * to_string(SafeSeparationAction action) noexcept;

const char * to_string(SafeSeparationReason reason) noexcept;

struct SameSideReplanShiftDistanceRequest
{
  double current_lateral_m{};
  double goal_lateral_m{};
  double planning_speed_mps{};
  double maximum_lateral_accel_mps2{};
  double minimum_shift_distance_m{0.5};
  double maximum_shift_distance_m{};
  double distance_margin_m{};
};

struct SameSideReplanShiftDistanceResolution
{
  bool valid{false};
  bool feasible{false};
  double lateral_adjustment_m{};
  double required_time_sec{};
  double required_distance_m{};
  double shift_distance_m{};
};

/// Size the replacement lateral ramp from the required motion and the
/// configured lateral-acceleration envelope. The result is a path distance,
/// not a per-cycle goal slew limit.
SameSideReplanShiftDistanceResolution resolve_same_side_replan_shift_distance(
  const SameSideReplanShiftDistanceRequest & request) noexcept;

struct OvertakeMissionDynamicCorridorSample
{
  double path_distance_m{};
  double lower_lateral_m{};
  double upper_lateral_m{};
  bool active{false};
  // Optional course metadata for the curve-aware Frenet DP.  Legacy callers
  // leave course_metadata_valid false and retain the original corridor cost.
  double reference_curvature_radpm{};
  bool target_active{false};
  bool course_metadata_valid{false};
  // Hard bounds are stored in lower_lateral_m/upper_lateral_m.  The preferred
  // interval represents robust wall/target reserve and is a soft DP cost only.
  double preferred_lower_lateral_m{-std::numeric_limits<double>::infinity()};
  double preferred_upper_lateral_m{std::numeric_limits<double>::infinity()};
  bool preferred_bounds_valid{false};
};

struct FrenetDpExecutionCorridorBoundsRequest
{
  int pass_side_sign{};
  double raw_lower_lateral_m{};
  double raw_upper_lateral_m{};
  bool target_active{false};
  double planner_target_separation_m{};
  double physical_target_separation_m{};
  double robust_target_separation_m{};
  double hard_wall_clearance_m{};
  double robust_wall_clearance_m{};
};

struct FrenetDpExecutionCorridorBoundsResolution
{
  bool valid{false};
  double hard_lower_lateral_m{};
  double hard_upper_lateral_m{};
  bool preferred_bounds_valid{false};
  double preferred_lower_lateral_m{};
  double preferred_upper_lateral_m{};
};

/// Convert raw gap-planner bounds into execution-aligned hard bounds and a
/// soft robust-reserve interval.  An unavailable preferred interval never
/// invalidates a physically feasible hard corridor.
FrenetDpExecutionCorridorBoundsResolution resolve_frenet_dp_execution_corridor_bounds(
  const FrenetDpExecutionCorridorBoundsRequest & request) noexcept;

struct FrenetDpHorizonClearanceRequest
{
  double path_distance_m{};
  double hard_horizon_distance_m{};
  double physical_wall_clearance_m{};
  double robust_wall_clearance_m{};
};

struct FrenetDpHorizonClearanceResolution
{
  bool valid{false};
  bool hard_horizon_active{false};
  double hard_wall_clearance_m{};
  double preferred_wall_clearance_m{};
};

/// Keep execution clearance hard only over the near receding horizon.  The
/// same clearance becomes a soft preference farther ahead so a remote narrow
/// corner cannot reject an immediately executable attack.  Raw track and
/// target-side bounds remain hard in both regions.
FrenetDpHorizonClearanceResolution resolve_frenet_dp_horizon_clearance(
  const FrenetDpHorizonClearanceRequest & request) noexcept;

struct FrenetDpExecutionEnvelopeRequest
{
  bool enabled{false};
  OvertakeMissionDynamicCorridorSample sample;
  double current_lateral_m{};
  double current_lateral_velocity_mps{};
  double current_speed_mps{};
  double maximum_lateral_accel_mps2{};
  double lateral_accel_reserve_ratio{1.0};
  bool static_hard_bounds_checked{false};
  bool static_hard_bounds_feasible{false};
  double static_hard_lower_m{};
  double static_hard_upper_m{};
  bool static_preferred_bounds_checked{false};
  bool static_preferred_bounds_feasible{false};
  double static_preferred_lower_m{};
  double static_preferred_upper_m{};
};

struct FrenetDpExecutionEnvelopeResolution
{
  bool valid{false};
  bool feasible{false};
  bool static_wall_constrained{false};
  bool reachability_constrained{false};
  double arrival_time_sec{};
  double effective_maximum_lateral_accel_mps2{};
  double reachable_lower_m{};
  double reachable_upper_m{};
  OvertakeMissionDynamicCorridorSample sample;
};

/// Intersect one DP sample with the connected static-map interval and the
/// lateral interval reachable from the measured state. A planning reserve may
/// keep the generated path inside the execution post-validation boundary.
FrenetDpExecutionEnvelopeResolution resolve_frenet_dp_execution_envelope(
  const FrenetDpExecutionEnvelopeRequest & request) noexcept;

struct FrenetDpTargetConstrainedCorridorRequest
{
  bool enabled{false};
  int pass_side_sign{};
  double nominal_ego_speed_mps{};
  double candidate_ego_speed_mps{};
  double prediction_horizon_sec{};
  double maximum_prediction_time_sec{};
  double target_lateral_now_m{};
  double target_lateral_predicted_m{};
  double target_longitudinal_now_m{};
  double target_longitudinal_predicted_m{};
  double longitudinal_overlap_threshold_m{};
  double physical_center_separation_m{};
  double robust_center_separation_m{};
  std::vector<OvertakeMissionDynamicCorridorSample> samples;
};

struct FrenetDpTargetConstrainedCorridorResolution
{
  bool valid{false};
  bool feasible{false};
  std::size_t constrained_sample_count{};
  std::size_t preferred_constrained_sample_count{};
  std::size_t failure_index{std::numeric_limits<std::size_t>::max()};
  std::vector<OvertakeMissionDynamicCorridorSample> samples;
};

/// Intersect a distance-domain DP corridor with the same time-aligned target
/// body prediction used by runtime promotion. Physical separation is a hard
/// bound; additional robust separation remains a soft preferred interval.
FrenetDpTargetConstrainedCorridorResolution constrain_frenet_dp_corridor_to_target(
  const FrenetDpTargetConstrainedCorridorRequest & request) noexcept;

struct OvertakeMissionDynamicCorridorRequest
{
  OvertakeMissionPathRequest mission_path;
  double candidate_goal_lower_m{-std::numeric_limits<double>::infinity()};
  double candidate_goal_upper_m{std::numeric_limits<double>::infinity()};
  double maximum_validation_distance_m{std::numeric_limits<double>::infinity()};
  std::vector<OvertakeMissionDynamicCorridorSample> samples;
};

struct OvertakeMissionDynamicCorridorResolution
{
  bool valid{false};
  bool observed{false};
  bool feasible{false};
  double goal_lower_m{-std::numeric_limits<double>::infinity()};
  double goal_upper_m{std::numeric_limits<double>::infinity()};
  std::size_t checked_sample_count{};
  std::size_t first_conflict_index{std::numeric_limits<std::size_t>::max()};
  double first_conflict_distance_m{std::numeric_limits<double>::infinity()};
  double first_conflict_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double first_conflict_lower_m{std::numeric_limits<double>::quiet_NaN()};
  double first_conflict_upper_m{std::numeric_limits<double>::quiet_NaN()};
};

/// Convert every time-aligned dynamic free-corridor sample into an admissible
/// interval for the immutable Pass goal. maximum_validation_distance_m lets
/// admission validate ShiftOut and the predicted Pass hold without inventing
/// an early Return; live Return is triggered and checked after rear-clear.
OvertakeMissionDynamicCorridorResolution resolve_overtake_mission_dynamic_corridor(
  const OvertakeMissionDynamicCorridorRequest & request) noexcept;

struct FrenetDpCorridorBranchInput
{
  int side_sign{};
  std::vector<OvertakeMissionDynamicCorridorSample> samples;
};

enum class FrenetDpTacticalStrategy
{
  Legacy,
  StraightDashi,
  InsideDive,
  SweepDive,
  OuterSweep,
};

const char * to_string(FrenetDpTacticalStrategy strategy) noexcept;

struct FrenetDpCorridorRequest
{
  bool enabled{false};
  double current_lateral_m{};
  int current_side_sign{};
  std::size_t lateral_bin_count{12U};
  double maximum_lateral_slope{0.45};
  double current_anchor_weight{2.0};
  double lateral_motion_weight{3.0};
  double previous_path_weight{1.0};
  double corridor_width_weight{0.25};
  double preferred_corridor_weight{4.0};
  double branch_switch_penalty{1.0};
  bool curve_strategy_enabled{false};
  double significant_curvature_radpm{0.08};
  double tactical_horizon_distance_m{std::numeric_limits<double>::infinity()};
  double tactical_reference_weight{1.0};
  double tactical_edge_fraction{0.45};
  double inside_radius_penalty_weight{1.0};
  double tactical_strategy_switch_penalty{1.0};
  int previous_side_sign{};
  FrenetDpTacticalStrategy previous_tactical_strategy{FrenetDpTacticalStrategy::Legacy};
  std::vector<double> previous_path_distances_m;
  std::vector<double> previous_lateral_path_m;
  FrenetDpCorridorBranchInput left;
  FrenetDpCorridorBranchInput right;
};

struct FrenetDpCorridorBranchResolution
{
  bool checked{false};
  bool feasible{false};
  int side_sign{};
  double normalized_cost{std::numeric_limits<double>::infinity()};
  double minimum_corridor_width_m{};
  double maximum_lateral_slope{};
  FrenetDpTacticalStrategy tactical_strategy{FrenetDpTacticalStrategy::Legacy};
  bool curve_observed{false};
  std::size_t tactical_knot_count{};
  double tactical_reference_cost{};
  std::vector<double> path_distances_m;
  std::vector<double> lateral_path_m;
};

struct FrenetDpLongitudinalProfileCandidate
{
  bool checked{false};
  bool feasible{false};
  double ego_speed_mps{};
  double closing_speed_mps{};
  double lateral_normalized_cost{std::numeric_limits<double>::infinity()};
};

struct FrenetDpLongitudinalProfileRequest
{
  bool enabled{false};
  double current_ego_speed_mps{};
  double lateral_cost_slack{};
  std::vector<FrenetDpLongitudinalProfileCandidate> candidates;
};

struct FrenetDpLongitudinalProfileResolution
{
  bool valid{false};
  bool checked{false};
  bool feasible{false};
  std::size_t checked_candidate_count{};
  std::size_t feasible_candidate_count{};
  std::size_t selected_candidate_index{std::numeric_limits<std::size_t>::max()};
  double minimum_lateral_cost{std::numeric_limits<double>::infinity()};
  double selected_ego_speed_mps{};
  double selected_closing_speed_mps{};
  double selected_lateral_cost{std::numeric_limits<double>::infinity()};
};

/// Select one time-aligned longitudinal profile for the lateral Frenet DP.
/// Profiles outside lateral_cost_slack of the best lateral solution are
/// rejected, then the fastest remaining profile wins. This preserves an
/// aggressive pass when it is comparably robust while allowing a short hold
/// or intermediate arrival when the attack timing closes the corridor.
FrenetDpLongitudinalProfileResolution select_frenet_dp_longitudinal_profile(
  const FrenetDpLongitudinalProfileRequest & request) noexcept;

struct FrenetDpCorridorResolution
{
  bool valid{false};
  bool checked{false};
  bool feasible{false};
  int selected_side_sign{};
  FrenetDpCorridorBranchResolution left;
  FrenetDpCorridorBranchResolution right;
};

/// Select a continuous Frenet homotopy before the existing box-constrained
/// lateral optimizer. Left and right are solved independently, so the DP can
/// never cut through the target footprint to change sides. The selected path
/// is a topology/reference result; wall, vehicle and kinematic hard guards are
/// still revalidated by the caller before execution.
FrenetDpCorridorResolution solve_frenet_dp_corridor(
  const FrenetDpCorridorRequest & request) noexcept;

/// Interpolate one admitted DP branch at a path distance. Invalid or
/// infeasible branches return NaN.
double sample_frenet_dp_corridor_path(
  const FrenetDpCorridorBranchResolution & branch,
  double path_distance_m) noexcept;

/// Interpolate a validated distance-domain lateral path. This is shared by
/// the DP execution reference and short-horizon runtime safety prediction so
/// both consumers observe exactly the same frozen path.
double sample_frenet_lateral_path(
  const std::vector<double> & path_distances_m,
  const std::vector<double> & lateral_path_m,
  double path_distance_m) noexcept;

/// Validate a distance-domain lateral reference produced by the Frenet DP.
/// Distances must be finite, non-negative and strictly increasing.
bool is_valid_frenet_dp_execution_path(
  const std::vector<double> & path_distances_m,
  const std::vector<double> & lateral_path_m) noexcept;

struct FrenetDpTransitionPathRequest
{
  double start_lateral_m{};
  double goal_lateral_m{};
  double transition_distance_m{};
  std::vector<double> path_distances_m;
};

struct FrenetDpTransitionPathResolution
{
  bool valid{false};
  std::vector<double> path_distances_m;
  std::vector<double> lateral_path_m;
};

/// Build a current-state distance-domain DP reference for a validated lateral
/// transition.  The smoothstep shape matches the explicit OvertakeLine ramp
/// and holds the goal after transition_distance_m.
FrenetDpTransitionPathResolution build_frenet_dp_transition_path(
  const FrenetDpTransitionPathRequest & request) noexcept;

struct FrenetDpExecutionReferenceRequest
{
  bool enabled{false};
  double traveled_distance_m{};
  std::size_t minimum_covered_sample_count{2U};
  std::vector<double> source_path_distances_m;
  std::vector<double> source_lateral_path_m;
  std::vector<double> horizon_path_distances_m;
  std::vector<double> fallback_lateral_targets_m;
};

struct FrenetDpExecutionReferenceResolution
{
  bool valid{false};
  bool active{false};
  bool coverage_complete{false};
  std::size_t covered_sample_count{};
  double remaining_distance_m{};
  std::vector<double> lateral_targets_m;
};

/// Align a frozen DP path with the current receding horizon. Covered samples
/// use the interpolated DP reference; an uncovered tail retains the caller's
/// legacy lateral targets. Once the useful prefix is exhausted, the complete
/// result falls back without extrapolating the last DP point indefinitely.
FrenetDpExecutionReferenceResolution resolve_frenet_dp_execution_reference(
  const FrenetDpExecutionReferenceRequest & request) noexcept;

struct FrenetDpExecutionTrustEnvelopeRequest
{
  bool enabled{false};
  double maximum_lateral_adjustment_m{};
  std::vector<double> candidate_lateral_targets_m;
  std::vector<double> nominal_lateral_targets_m;
};

struct FrenetDpExecutionTrustEnvelopeResolution
{
  bool valid{false};
  bool active{false};
  bool adjusted{false};
  double maximum_applied_adjustment_m{};
  std::vector<double> lateral_targets_m;
};

/// Bound an optimizer-produced execution profile around the already admitted
/// Mission profile. A topology change larger than this envelope must be
/// promoted as a new Mission instead of leaking through a per-stage override.
FrenetDpExecutionTrustEnvelopeResolution resolve_frenet_dp_execution_trust_envelope(
  const FrenetDpExecutionTrustEnvelopeRequest & request) noexcept;

struct FrenetDpExecutionRefreshStitchRequest
{
  double current_lateral_m{};
  double active_traveled_distance_m{};
  double preserved_prefix_distance_m{};
  double blend_end_distance_m{};
  bool measured_state_reachability_enabled{false};
  double current_lateral_velocity_mps{};
  double current_speed_mps{};
  double maximum_lateral_accel_mps2{};
  std::vector<double> active_path_distances_m;
  std::vector<double> active_lateral_path_m;
  std::vector<double> candidate_path_distances_m;
  std::vector<double> candidate_lateral_path_m;
};

struct FrenetDpExecutionRefreshStitchResolution
{
  bool valid{false};
  bool used_active_path{false};
  bool measured_state_reachability_used{false};
  bool lateral_reachability_constrained{false};
  double maximum_unconstrained_lateral_accel_mps2{};
  std::vector<double> path_distances_m;
  std::vector<double> lateral_path_m;
};

/// Rebase a rolling DP candidate on the measured state and the unconsumed
/// prefix of the last feasible path.  The old prefix is preserved briefly,
/// then smoothstep-blended into the newly optimized downstream path.  When
/// measured-state reachability is enabled, the blend is projected onto the
/// same constant-acceleration envelope used by DP sample generation.  The
/// caller must still run all hard execution validation before atomically
/// promoting the result.
FrenetDpExecutionRefreshStitchResolution stitch_frenet_dp_execution_refresh_path(
  const FrenetDpExecutionRefreshStitchRequest & request) noexcept;

struct FrenetDpExecutionRefreshRequest
{
  bool enabled{false};
  bool active_execution{false};
  bool target_matches{false};
  bool prediction_fresh{false};
  int active_side_sign{};
  int candidate_side_sign{};
  double now_sec{};
  double last_refresh_sec{-std::numeric_limits<double>::infinity()};
  double minimum_refresh_interval_sec{};
  double candidate_generated_at_sec{-std::numeric_limits<double>::infinity()};
  double last_source_generated_at_sec{-std::numeric_limits<double>::infinity()};
  std::vector<double> candidate_path_distances_m;
  std::vector<double> candidate_lateral_path_m;
};

struct FrenetDpExecutionRefreshResolution
{
  bool valid{false};
  bool refresh{false};
};

/// Admit an atomic same-target/same-side refresh of the active DP execution
/// reference.  A rejected refresh never invalidates the current feasible path.
FrenetDpExecutionRefreshResolution resolve_frenet_dp_execution_refresh(
    const FrenetDpExecutionRefreshRequest &request) noexcept;

struct FrenetDpTargetBoundHorizonRequest
{
  bool enabled{false};
  int pass_side_sign{};
  double required_center_separation_m{};
  std::vector<double> candidate_lateral_path_m;
  std::vector<double> target_lateral_path_m;
  std::vector<bool> target_separation_active;
};

struct FrenetDpTargetBoundHorizonResolution
{
  bool valid{false};
  bool feasible{false};
  std::size_t constrained_sample_count{};
  std::size_t failure_index{std::numeric_limits<std::size_t>::max()};
  double minimum_signed_separation_m{std::numeric_limits<double>::infinity()};
};

/// Verify that a pending distance-domain DP execution prefix remains on the
/// committed side of every time-aligned target body-overlap sample. This is a
/// hard physical-body check performed before atomic refresh promotion; robust
/// clearance remains a soft receding-horizon preference.
FrenetDpTargetBoundHorizonResolution validate_frenet_dp_target_bound_horizon(
  const FrenetDpTargetBoundHorizonRequest & request) noexcept;

struct FrenetDpAtomicRefreshPromotionRequest {
  bool refresh_requested{false};
  bool reference_valid{false};
  bool reference_active{false};
  // The fresh source prefix may use a previous feasible/Mission tail. This bit
  // describes the combined executable horizon, not source-path coverage.
  bool executable_horizon_complete{false};
  bool execution_horizon_feasible{false};
  bool target_bound_horizon_feasible{false};
  bool target_matches{false};
  bool target_continuous{false};
  bool target_position_jump{false};
  bool target_course_progress_rejected{false};
  bool target_prediction_valid{false};
  bool current_body_separated{false};
  bool recoverable_side_contact{false};
  bool hard_fault{false};
};

struct FrenetDpAtomicRefreshPromotionResolution {
  bool valid{false};
  bool promote{false};
  bool reference_ready{false};
  bool target_ready{false};
  bool target_bound_ready{false};
  bool hard_fault_free{false};
};

/// Promote a pending rolling-refresh path only after the combined fresh prefix
/// and last-feasible tail earned a complete current-state control-horizon
/// validation. A rejected pending candidate must never revoke or mutate the
/// previously validated active execution path.
FrenetDpAtomicRefreshPromotionResolution
resolve_frenet_dp_atomic_refresh_promotion(
    const FrenetDpAtomicRefreshPromotionRequest &request) noexcept;

struct FrenetDpMeasuredRebaseRetryRequest {
  bool enabled{false};
  bool refresh_requested{false};
  bool normal_candidate_promoted{false};
  bool target_matches{false};
  bool target_continuous{false};
  bool target_position_jump{false};
  bool target_course_progress_rejected{false};
  bool target_prediction_valid{false};
  bool current_body_separated{false};
  bool recoverable_side_contact{false};
  bool hard_fault{false};
};

struct FrenetDpMeasuredRebaseRetryResolution {
  bool valid{false};
  bool retry{false};
};

/// Decide whether a rejected rolling refresh may be rebuilt directly from
/// measured e_y. This never relaxes promotion constraints: the rebuilt path
/// must subsequently pass the same wall, kinematic and target-bound checks.
FrenetDpMeasuredRebaseRetryResolution
resolve_frenet_dp_measured_rebase_retry(
    const FrenetDpMeasuredRebaseRetryRequest &request) noexcept;

struct FrenetDpExecutionAuthorityRequest {
  bool enabled{false};
  bool active_execution{false};
  bool rolling_replan_pause_active{false};
  bool target_matches{false};
  bool target_continuous{false};
  bool target_position_jump{false};
  bool target_course_progress_rejected{false};
  bool path_side_matches{false};
  bool current_body_separated{false};
  bool target_prediction_valid{false};
  bool predicted_body_sweep_separated{false};
  bool recoverable_side_contact{false};
  bool actual_wall_physical_contact{false};
  bool wall_margin_blocked{false};
  bool wall_sample_unavailable{false};
  bool emergency_front_risk{false};
  bool solver_recovery_active{false};
  bool forbidden_waypoint{false};
  double now_sec{};
  double last_refresh_sec{-std::numeric_limits<double>::infinity()};
  double maximum_path_age_sec{};
  double last_runtime_validation_sec{-std::numeric_limits<double>::infinity()};
  double maximum_runtime_validation_age_sec{};
  double traveled_distance_m{};
  double minimum_remaining_distance_m{};
  std::vector<double> path_distances_m;
  std::vector<double> lateral_path_m;
  bool execution_tracking_safe{true};
  bool solver_degraded{false};
};

struct FrenetDpExecutionAuthorityResolution
{
  bool valid{false};
  bool authority_active{false};
  bool source_fresh{false};
  bool runtime_validation_fresh{false};
  bool authority_from_runtime_validation{false};
  double path_age_sec{std::numeric_limits<double>::infinity()};
  double runtime_validation_age_sec{std::numeric_limits<double>::infinity()};
  double remaining_distance_m{};
};

/// Decide whether an already admitted same-target/same-side DP prefix owns
/// bounded ShiftOut/Pass execution or a soft rolling-replan pause. Optimizer
/// source age is an absolute execution limit: runtime validation may bridge
/// target-prediction jitter but may not extend an expired source path. Wall,
/// body, target-continuity, tracking, emergency and solver faults revoke
/// authority immediately.
FrenetDpExecutionAuthorityResolution resolve_frenet_dp_execution_authority(
  const FrenetDpExecutionAuthorityRequest & request) noexcept;

struct SolvedExecutionSourceHandoffRequest
{
  bool enabled{false};
  bool active_execution{false};
  bool current_execution_authority_active{false};
  bool context_matches{false};
  bool physically_validated{false};
  bool trust_envelope_validated{false};
  bool hard_fault{false};
  double now_sec{};
  double source_solved_sec{-std::numeric_limits<double>::infinity()};
  double last_promoted_source_solved_sec{-std::numeric_limits<double>::infinity()};
  double last_execution_refresh_sec{-std::numeric_limits<double>::infinity()};
  double minimum_refresh_interval_sec{};
  double maximum_source_age_sec{};
  std::vector<double> path_distances_m;
  std::vector<double> lateral_path_m;
};

struct SolvedExecutionSourceHandoffResolution
{
  bool valid{false};
  bool source_newer{false};
  bool refresh_due{false};
  bool path_valid{false};
  bool promote{false};
  double source_age_sec{std::numeric_limits<double>::infinity()};
};

/// Decide whether a freshly solved and physically revalidated progress-
/// contouring trajectory may atomically replace the active execution prefix.
/// Reusing the same solved source can never renew its absolute execution age.
SolvedExecutionSourceHandoffResolution resolve_solved_execution_source_handoff(
  const SolvedExecutionSourceHandoffRequest & request) noexcept;

enum class OvertakeMissionCorridorSource
{
  None,
  DynamicObservation,
  StaticWallFallback,
};

struct OvertakeMissionCorridorAdmissionRequest
{
  /// The behavior-level entry guards admitted this side for full mission
  /// evaluation. The dynamic planner may still have no active sample when its
  /// short horizon has not reached the candidate path yet.
  bool entry_gap_available{false};
  OvertakeMissionDynamicCorridorResolution dynamic_corridor;
  double static_goal_lower_m{-std::numeric_limits<double>::infinity()};
  double static_goal_upper_m{std::numeric_limits<double>::infinity()};
};

struct OvertakeMissionCorridorAdmissionResolution
{
  bool valid{false};
  bool feasible{false};
  OvertakeMissionCorridorSource source{OvertakeMissionCorridorSource::None};
  double goal_lower_m{-std::numeric_limits<double>::infinity()};
  double goal_upper_m{std::numeric_limits<double>::infinity()};
};

/// Prefer an observed dynamic corridor. A valid but not-yet-observed dynamic
/// horizon may use the static wall interval so later body, kinematic and full
/// mission preflights can decide feasibility. An observed dynamic conflict is
/// never hidden by the fallback.
OvertakeMissionCorridorAdmissionResolution resolve_overtake_mission_corridor_admission(
  const OvertakeMissionCorridorAdmissionRequest & request) noexcept;

const char * to_string(OvertakeMissionCorridorSource source) noexcept;

struct StaticFallbackEntryMotionAdmissionRequest
{
  bool guard_enabled{false};
  bool new_mission_entry{false};
  OvertakeMissionCorridorSource corridor_source{OvertakeMissionCorridorSource::None};
  double lateral_shift_m{};
  double maximum_lateral_shift_m{};
};

struct StaticFallbackEntryMotionAdmissionResolution
{
  bool valid{false};
  bool admitted{false};
  bool guard_applied{false};
};

/// Bound only a new Mission's lateral motion while its vehicle corridor is
/// still unobserved. Dynamic candidates and active-Mission replans retain
/// their dedicated feasibility/atomic replacement policies.
StaticFallbackEntryMotionAdmissionResolution
resolve_static_fallback_entry_motion_admission(
  const StaticFallbackEntryMotionAdmissionRequest & request) noexcept;

struct RollingSameSideLateralAdmissionRequest
{
  bool guard_enabled{false};
  double current_lateral_m{};
  double candidate_goal_lateral_m{};
  double maximum_lateral_adjustment_m{};
};

struct RollingSameSideLateralAdmissionResolution
{
  bool valid{false};
  bool admitted{false};
  bool guard_applied{false};
  double lateral_adjustment_m{};
};

/// Bound a tactical same-side continuation to a small correction from the
/// current measured lateral position. This guard is deliberately independent
/// of pass-side sign so left and right continuations remain symmetric.
RollingSameSideLateralAdmissionResolution
resolve_rolling_same_side_lateral_admission(
  const RollingSameSideLateralAdmissionRequest & request) noexcept;

struct RecedingExecutionPrefixAssessmentRequest
{
  bool enabled{false};
  bool shadow_assessment{false};
  bool tactical_replan_pause{false};
  bool active_shiftout_or_pass{false};
  bool side_matches{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool target_course_progress_rejected{false};
  bool current_body_separated{false};
  bool recoverable_side_contact{false};
  bool target_prediction_valid{false};
  bool forbidden_waypoint{false};
  bool emergency_front_risk{false};
  bool solver_recovery_active{false};
};

struct RecedingExecutionPrefixAssessmentResolution
{
  bool valid{false};
  bool admitted{false};
  bool primary_execution{false};
};

/// Decide whether the same-side current-state prefix planner may run. The
/// planner is intentionally active during a healthy ShiftOut/Pass as well as
/// during a tactical replan pause. Target intrusion is not a rejection here:
/// the fresh target bounds decide whether a new prefix is feasible.
RecedingExecutionPrefixAssessmentResolution
resolve_receding_execution_prefix_assessment(
  const RecedingExecutionPrefixAssessmentRequest & request) noexcept;

struct OvertakeMissionCandidate
{
  bool feasible{false};
  bool direct_pass{false};
  double shift_distance_m{};
  double goal_lateral_m{};
  double lateral_shift_m{};
  double max_required_lateral_accel_mps2{};
  bool body_clear_deadline_checked{false};
  bool body_clear_deadline_feasible{true};
  double predicted_body_clear_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_hard_distance_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_body_clear_distance_m{std::numeric_limits<double>::infinity()};
  double closing_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  // Selection-transparent metadata travels with the ranked candidate. Keeping
  // it here avoids a second, index-coupled metadata vector in the controller.
  int pass_side_sign{0};
  /// Instantaneous inflated target-to-wall room on this side at entry. It is
  /// only a ranking input for an unlocked straight entry; all candidates have
  /// already passed their wall, target and completion admission checks.
  double entry_side_clearance_m{std::numeric_limits<double>::infinity()};
  bool current_position_clear{false};
  double body_clear_deadline_slack_sec{std::numeric_limits<double>::quiet_NaN()};
  bool rear_clear_prediction_checked{false};
  bool rear_clear_prediction_feasible{true};
  double predicted_rear_clear_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_rear_clear_ego_distance_m{std::numeric_limits<double>::infinity()};
  double predicted_rear_clear_speed_mps{std::numeric_limits<double>::infinity()};
  double predicted_minimum_ego_speed_mps{std::numeric_limits<double>::infinity()};
  bool horizon_progress_checked{false};
  double horizon_progress_score{std::numeric_limits<double>::quiet_NaN()};
  double horizon_progress_time{std::numeric_limits<double>::quiet_NaN()};
  double horizon_progress_distance{std::numeric_limits<double>::quiet_NaN()};
  double horizon_progress_retained_speed{std::numeric_limits<double>::quiet_NaN()};
  double pass_hold_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double return_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double static_valid_until_pass_m{std::numeric_limits<double>::quiet_NaN()};
  double dynamic_valid_until_pass_m{std::numeric_limits<double>::quiet_NaN()};
  double planner_generated_at_sec{-std::numeric_limits<double>::infinity()};
  double prediction_source_age_sec{std::numeric_limits<double>::infinity()};
  double prediction_epoch_sec{-std::numeric_limits<double>::infinity()};
  double prediction_horizon_sec{};
  double dynamic_valid_until_sec{-std::numeric_limits<double>::infinity()};
  bool outer_strategy_committed{false};
  OvertakeMissionCorridorSource corridor_source{OvertakeMissionCorridorSource::None};
  bool outer_transition_required{false};
  /// Provenance for an outer transition frozen into this candidate. The
  /// controller sets it only after the transition-to-Return preflight passes.
  bool outer_transition_preflight_validated{false};
  int outer_transition_side_sign{0};
  double outer_transition_start_pass_m{std::numeric_limits<double>::infinity()};
  double outer_transition_deadline_pass_m{std::numeric_limits<double>::infinity()};
  double outer_transition_goal_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double outer_transition_shift_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_path_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double minimum_path_corridor_width_m{std::numeric_limits<double>::infinity()};
  double minimum_return_wall_clearance_m{std::numeric_limits<double>::infinity()};
  bool rear_clear_course_role_checked{false};
  PassSideCourseRole entry_course_role{PassSideCourseRole::Unknown};
  PassSideCourseRole rear_clear_course_role{PassSideCourseRole::Unknown};
  bool full_track_transition_before_rear_clear{false};
  bool inner_to_outer_at_rear_clear{false};
  double first_course_role_reversal_distance_m{std::numeric_limits<double>::infinity()};
  // Selection-transparent metadata used by the controller's per-side
  // straight/outer clearance tier. Global side ranking intentionally ignores it.
  double straight_outer_clearance_bias_applied_m{};
  // A new-entry-only Mission whose local ShiftOut/body-clear is validated,
  // while rear-clear and Return are intentionally deferred to rolling replan.
  // It must never be used as an atomic replacement for an active Mission.
  bool progressive_entry{false};
  /// Minimum predicted physical surface clearance to the target after the
  /// lateral ShiftOut completes and until rear-clear (or rollout end).
  bool pass_target_clearance_checked{false};
  double predicted_minimum_pass_target_surface_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  /// A stage-wise Frenet corridor can admit a receding prefix even when no
  /// immutable pass_lateral value exists across the complete horizon.
  bool frenet_dp_corridor_checked{false};
  bool frenet_dp_corridor_feasible{false};
  bool frenet_dp_prefix_bridge{false};
  double frenet_dp_normalized_cost{std::numeric_limits<double>::quiet_NaN()};
  FrenetDpTacticalStrategy frenet_dp_tactical_strategy{
    FrenetDpTacticalStrategy::Legacy};
  std::size_t frenet_dp_tactical_knot_count{};
  std::vector<double> frenet_dp_path_distances_m{};
  std::vector<double> frenet_dp_lateral_path_m{};
  /// New-entry reserve needed to become laterally body-clear before closing
  /// consumes the longitudinal clearance. This is selection-transparent
  /// metadata for diagnostics; active Mission replans do not reapply it.
  bool entry_front_distance_reserve_applied{false};
  double required_entry_front_distance_m{};
  /// Minimum ego speed requirement used by the snapshot which generated this
  /// rollout. It travels atomically with async tactical candidates so live
  /// admission never compares old prediction metadata with a new threshold.
  double planning_minimum_ego_speed_requirement_mps{
    std::numeric_limits<double>::quiet_NaN()};
};

enum class ExtendedMpccBranchCandidateSource
{
  None,
  CompleteSelectedMission,
  RecedingPrefix,
  SelectedProgressivePrefix,
};

const char * to_string(ExtendedMpccBranchCandidateSource source) noexcept;

struct ExtendedMpccBranchCandidateRequest
{
  int side_sign{};
  std::optional<OvertakeMissionCandidate> selected_mission;
  std::optional<OvertakeMissionCandidate> receding_mission;
};

struct ExtendedMpccBranchCandidateResolution
{
  bool valid{false};
  bool prefix_only{false};
  ExtendedMpccBranchCandidateSource source{
    ExtendedMpccBranchCandidateSource::None};
  std::optional<OvertakeMissionCandidate> candidate;
};

/// Resolve the executable candidate presented to one isolated extended-MPCC
/// branch. A complete selected Mission has precedence, followed by a fresh
/// receding prefix and finally a selected progressive prefix. Prefix metadata
/// is retained so downstream admission never mistakes it for rear-clear and
/// Return authority.
ExtendedMpccBranchCandidateResolution resolve_extended_mpcc_branch_candidate(
  const ExtendedMpccBranchCandidateRequest & request) noexcept;

/// A Mission which changes from outer to inner before rear-clear may only be
/// admitted when the required full-track handoff has itself been planned and
/// preflighted.  Otherwise the candidate is executable only up to the role
/// reversal, not through completion.
bool is_full_track_transition_admitted(
  bool full_track_transition_before_rear_clear,
  bool scheduled_transition_validated) noexcept;

struct OvertakePassPlanRequest
{
  OvertakeMissionCandidate candidate;
  double start_lateral_m{};
  double return_lateral_m{};
};

struct OvertakePassPlan
{
  bool valid{false};
  OvertakeMissionCandidate mission;
  OvertakeMissionPathRequest path;
};

/// Validate and freeze the selected side, speed profile and complete
/// ShiftOut/Pass/Return ey(s) path as one execution plan.
OvertakePassPlan build_overtake_pass_plan(
  const OvertakePassPlanRequest & request) noexcept;

struct OvertakeMissionHorizonProgressWeights
{
  double rear_clear_time{3.0};
  double rear_clear_distance{1.0};
  double retained_speed{1.5};
  double closing_speed{0.5};
  double lateral_motion_penalty{0.5};
  double lateral_accel_penalty{0.25};
};

enum class OvertakeMissionHorizonProgressRejectReason
{
  None,
  Disabled,
  InvalidInput,
  RearClearUnchecked,
  RearClearInfeasible,
  RearClearTimeBudget,
  RearClearDistanceBudget,
};

struct OvertakeMissionHorizonProgressRequest
{
  bool enabled{false};
  OvertakeMissionCandidate candidate;
  double rear_clear_time_budget_sec{};
  double rear_clear_distance_budget_m{};
  double reference_speed_mps{};
  double maximum_closing_speed_mps{};
  double lateral_motion_scale_m{};
  double maximum_lateral_accel_mps2{};
  OvertakeMissionHorizonProgressWeights weights;
};

struct OvertakeMissionHorizonProgressEvaluation
{
  bool valid{false};
  bool checked{false};
  bool hard_feasible{false};
  OvertakeMissionHorizonProgressRejectReason reject_reason{
    OvertakeMissionHorizonProgressRejectReason::InvalidInput};
  double score{-std::numeric_limits<double>::infinity()};
  double rear_clear_time_progress{};
  double rear_clear_distance_progress{};
  double retained_speed{};
  double closing_speed_progress{};
  double lateral_motion_cost{};
  double lateral_accel_cost{};
};

/// Rank a physically admitted mission by its ability to reach rear-clear while
/// retaining speed. Static wall, dynamic corridor and body-clear violations
/// remain hard constraints in admission; this score only orders candidates
/// that passed those constraints.
OvertakeMissionHorizonProgressEvaluation evaluate_overtake_mission_horizon_progress(
  const OvertakeMissionHorizonProgressRequest & request) noexcept;

/// Build a small deterministic longitudinal candidate set from the existing
/// ShiftOut bounds. Invalid bounds produce no candidates; equal bounds produce
/// one candidate. The midpoint adds a useful distance-preserving option without
/// introducing another tuning parameter.
std::vector<double> build_overtake_closing_speed_candidates(
  double minimum_closing_speed_mps, double maximum_closing_speed_mps) noexcept;

struct OvertakeMissionCandidateSelectionRequest
{
  std::vector<OvertakeMissionCandidate> candidates;
  double minimum_deadline_slack_sec{};
  bool horizon_progress_enabled{false};
  double horizon_progress_time_budget_sec{};
  double horizon_progress_distance_budget_m{};
  double horizon_progress_reference_speed_mps{};
  double horizon_progress_maximum_closing_speed_mps{};
  double horizon_progress_lateral_motion_scale_m{};
  double horizon_progress_maximum_lateral_accel_mps2{};
  OvertakeMissionHorizonProgressWeights horizon_progress_weights;
  /// Only let physical path reserve override racing progress when the
  /// difference is material. This preserves aggressive ordering for ties.
  double minimum_clearance_advantage_m{};
  /// At an unlocked straight entry, prefer materially larger current
  /// target-to-wall room before comparing racing progress. Curve and active
  /// Mission selection leave this disabled and continue to use the full
  /// horizon course-role policy.
  bool entry_side_clearance_selection_enabled{false};
  /// Prefer a side that can reach rear-clear without crossing the full track.
  /// Entry inner/outer labels alone are intentionally not selection rules.
  bool rear_clear_side_selection_enabled{false};
  /// Let a materially larger balanced target/wall reserve override progress.
  /// Smaller differences retain the existing aggressive progress ordering.
  double minimum_interaction_clearance_advantage_m{};
};

struct OvertakeMissionCandidateSelection
{
  bool valid{false};
  bool found{false};
  /// Candidate-local numeric faults are isolated instead of poisoning the
  /// complete left/right search. This counter is diagnostic only.
  std::size_t invalid_candidate_count{};
  std::size_t selected_index{std::numeric_limits<std::size_t>::max()};
  OvertakeMissionCandidate candidate;
  OvertakeMissionHorizonProgressEvaluation horizon_progress;
};

/// Select a deterministic executable mission. Evaluated, deadline-feasible
/// candidates and the configured slack reserve precede racing-line retention
/// and body-clear time. When rear-clear side selection is enabled, a candidate
/// that needs no full-track transition before rear-clear precedes an entry-outer
/// candidate that becomes inner. Legacy unchecked candidates retain the prior
/// geometric ordering. Otherwise-identical candidates prefer the higher closing
/// speed, then an outside role at rear-clear.
OvertakeMissionCandidateSelection select_overtake_mission_candidate(
  const OvertakeMissionCandidateSelectionRequest & request) noexcept;

enum class MpccLiteShadowBranch
{
  None,
  Left,
  Right,
  CurrentSideHold,
  Return,
};

const char * to_string(MpccLiteShadowBranch branch) noexcept;

enum class MpccLiteShadowRejectReason
{
  None,
  Disabled,
  InvalidRequest,
  Unavailable,
  PlanningUnavailable,
  HardConstraint,
  InvalidCandidate,
  MissionInfeasible,
  BodyClearUnchecked,
  BodyClearInfeasible,
  ProgressiveEntryIncomplete,
  RearClearUnchecked,
  RearClearInfeasible,
  RearClearTimeBudget,
  RearClearDistanceBudget,
  TargetClearanceUnchecked,
  OuterTransitionUnvalidated,
  RuntimeHardFault,
  MissionTotalTimeBudget,
  SafeSeparationTimeBudget,
  SafeSeparationDistanceBudget,
  ReturnNotAdmitted,
  ReturnCorridorBlocked,
  FrenetCorridorInfeasible,
};

const char * to_string(MpccLiteShadowRejectReason reason) noexcept;

struct MpccLiteShadowCandidate
{
  MpccLiteShadowBranch branch{MpccLiteShadowBranch::None};
  bool assessed{false};
  bool available{false};
  bool hard_feasible{false};
  MpccLiteShadowRejectReason admission_reject_reason{
    MpccLiteShadowRejectReason::None};
  bool rear_clear_required{true};
  bool rear_clear_feasible{false};
  double predicted_rear_clear_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_rear_clear_distance_m{std::numeric_limits<double>::infinity()};
  double predicted_minimum_speed_mps{};
  double minimum_wall_clearance_m{};
  double minimum_target_clearance_m{};
  double maximum_lateral_accel_mps2{};
  double lateral_motion_m{};
  bool frenet_dp_corridor_checked{false};
  bool frenet_dp_corridor_feasible{false};
  bool frenet_dp_prefix_bridge{false};
  double frenet_dp_normalized_cost{};
};

struct MpccLiteShadowMissionCandidateRequest
{
  MpccLiteShadowBranch branch{MpccLiteShadowBranch::None};
  bool assessed{false};
  std::optional<OvertakeMissionCandidate> mission;
  bool runtime_hard_fault{false};
  double fallback_wall_clearance_m{};
  double fallback_target_clearance_m{};
  bool mission_time_budget_active{false};
  double mission_time_remaining_sec{std::numeric_limits<double>::infinity()};
  bool safe_separation_budget_active{false};
  double safe_separation_time_remaining_sec{std::numeric_limits<double>::infinity()};
  double safe_separation_distance_remaining_m{std::numeric_limits<double>::infinity()};
};

/// Convert one planner Mission into a typed shadow candidate. This function
/// preserves the first admission failure so runtime logs can distinguish an
/// incomplete progressive horizon from physical and remaining-budget faults.
MpccLiteShadowCandidate build_mpcc_lite_shadow_mission_candidate(
  const MpccLiteShadowMissionCandidateRequest & request) noexcept;

struct MpccLiteRecedingPrefixCandidateRequest
{
  MpccLiteShadowBranch branch{MpccLiteShadowBranch::None};
  bool assessed{false};
  std::optional<OvertakeMissionCandidate> mission;
  bool runtime_hard_fault{false};
  double fallback_wall_clearance_m{};
  double fallback_target_clearance_m{};
  /// Terminal progress beyond body-clear. This prevents a short prefix from
  /// receiving the same score as a completed rear-clear trajectory.
  double terminal_time_sec{};
  double terminal_distance_m{};
};

/// Convert a locally preflighted ShiftOut/body-clear prefix into an MPCC-lite
/// candidate. Unlike a complete Mission this deliberately does not require
/// rear-clear or Return to be inside the current horizon. Wall, target,
/// body-clear and runtime faults remain hard constraints.
MpccLiteShadowCandidate build_mpcc_lite_receding_prefix_candidate(
  const MpccLiteRecedingPrefixCandidateRequest & request) noexcept;

enum class MpccLitePrefixExecutionRejectReason
{
  None,
  Inactive,
  NoReturn,
  SafeSeparation,
  NotProgressive,
  CandidateInfeasible,
  BodyClearUnchecked,
  BodyClearInfeasible,
  TargetClearanceUnchecked,
  TargetClearanceInfeasible,
  InvalidPrediction,
  WallReserveInsufficient,
  TimeBudgetExceeded,
  DistanceBudgetExceeded,
  MinimumSpeedInsufficient,
  Admitted,
};

struct MpccLitePrefixExecutionRequest
{
  bool active_execution{false};
  /// A fresh Idle/Follow entry may execute a locally hard-feasible prefix
  /// before a complete rear-clear/Return Mission exists. It remains distinct
  /// from an active Mission replacement so no-return semantics stay explicit.
  bool new_entry_context{false};
  bool before_no_return{false};
  bool safe_separation_active{false};
  /// A runtime completion replan may replace SafeSeparation with a freshly
  /// validated same-side prefix while the target is still ahead. The caller
  /// must re-check target continuity, predicted separation and hard faults;
  /// this flag never authorizes an opposite-side prefix by itself.
  bool safe_separation_tactical_rearmed{false};
  bool candidate_progressive{false};
  bool candidate_feasible{false};
  bool body_clear_deadline_checked{false};
  bool body_clear_deadline_feasible{false};
  bool target_clearance_checked{false};
  double minimum_target_surface_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  double predicted_body_clear_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_body_clear_distance_m{std::numeric_limits<double>::infinity()};
  double predicted_minimum_ego_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_ego_speed_mps{};
  double minimum_path_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double minimum_required_path_wall_clearance_m{};
  double remaining_time_budget_sec{std::numeric_limits<double>::infinity()};
  double remaining_distance_budget_m{std::numeric_limits<double>::infinity()};
  bool pass_phase{false};
  double planning_minimum_ego_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_speed_tolerance_mps{0.02};
};

struct MpccLitePrefixExecutionResolution
{
  bool valid{false};
  bool admitted{false};
  bool restart_shiftout{false};
  MpccLitePrefixExecutionRejectReason reason{
    MpccLitePrefixExecutionRejectReason::None};
};

/// Admit a short, hard-feasible receding-horizon prefix without pretending it
/// contains a rear-clear/Return solution. A prefix may start a fresh entry or
/// replace an active pre-no-return Mission. SafeSeparation remains fail-closed
/// unless the caller explicitly re-arms a Pass-phase same-side runtime
/// completion replacement. Its ShiftOut and short continuation must fit the
/// remaining runtime budget and retain the speed and physical reserves
/// required at the commit point.
MpccLitePrefixExecutionResolution resolve_mpcc_lite_prefix_execution(
  const MpccLitePrefixExecutionRequest & request) noexcept;

const char * to_string(MpccLitePrefixExecutionRejectReason reason) noexcept;

struct MpccLiteShadowReturnAdmissionRequest
{
  bool phase_relevant{false};
  bool return_active{false};
  bool rear_clear_confirmed{false};
  bool return_corridor_blocked{false};
  bool runtime_hard_fault{false};
};

/// Return execution has already crossed the rear-clear admission boundary.
/// Recheck its live corridor and runtime faults without requiring a latch that
/// may be cleared as part of the FSM transition itself.
MpccLiteShadowRejectReason resolve_mpcc_lite_shadow_return_admission(
  const MpccLiteShadowReturnAdmissionRequest & request) noexcept;

struct MpccLiteShadowWeights
{
  double rear_clear_time{3.0};
  double rear_clear_distance{1.0};
  double retained_speed{1.5};
  double wall_clearance{1.0};
  double target_clearance{1.0};
  double lateral_motion_penalty{0.5};
  double lateral_accel_penalty{0.25};
  double branch_switch_penalty{0.10};
  double frenet_dp_corridor_penalty{1.0};
};

struct MpccLiteShadowRequest
{
  bool enabled{false};
  std::vector<MpccLiteShadowCandidate> candidates;
  MpccLiteShadowBranch active_branch{MpccLiteShadowBranch::None};
  double rear_clear_time_budget_sec{};
  double rear_clear_distance_budget_m{};
  double reference_speed_mps{};
  double reference_wall_clearance_m{};
  double reference_target_clearance_m{};
  double lateral_motion_scale_m{};
  double maximum_lateral_accel_mps2{};
  MpccLiteShadowWeights weights;
};

struct MpccLiteShadowEvaluation
{
  bool valid{false};
  bool checked{false};
  bool hard_feasible{false};
  MpccLiteShadowRejectReason reject_reason{MpccLiteShadowRejectReason::InvalidCandidate};
  MpccLiteShadowCandidate candidate;
  double score{-std::numeric_limits<double>::infinity()};
  double rear_clear_time_progress{};
  double rear_clear_distance_progress{};
  double retained_speed{};
  double wall_clearance_reserve{};
  double target_clearance_reserve{};
  double lateral_motion_cost{};
  double lateral_accel_cost{};
  double branch_switch_cost{};
  double frenet_dp_corridor_cost{};
};

struct MpccLiteShadowResolution
{
  bool valid{false};
  bool found{false};
  bool agrees_with_active_branch{false};
  std::vector<MpccLiteShadowEvaluation> evaluations;
  std::optional<MpccLiteShadowEvaluation> active_evaluation;
  MpccLiteShadowEvaluation best;
};

enum class MpccLiteCompletionPredictionSource
{
  None,
  CompleteRearClear,
  RecedingPrefix,
};

struct MpccLiteCompletionPredictionRequest
{
  bool fresh_resolution{false};
  bool hard_feasible{false};
  bool complete_rear_clear_mission{false};
  bool receding_prefix_execution_admitted{false};
  int side_sign{0};
  double predicted_completion_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_completion_distance_m{std::numeric_limits<double>::infinity()};
};

struct MpccLiteCompletionPredictionResolution
{
  bool valid{false};
  MpccLiteCompletionPredictionSource source{
    MpccLiteCompletionPredictionSource::None};
  int side_sign{0};
  double predicted_completion_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_completion_distance_m{std::numeric_limits<double>::infinity()};
};

/// Convert a selected MPCC-lite branch into typed completion evidence. A
/// complete rear-clear rollout and an admitted receding prefix remain
/// distinguishable; stale or hard-infeasible shadow results never gain runtime
/// deadline authority.
MpccLiteCompletionPredictionResolution resolve_mpcc_lite_completion_prediction(
  const MpccLiteCompletionPredictionRequest & request) noexcept;

const char * to_string(MpccLiteCompletionPredictionSource source) noexcept;

/// Score tactical left/right/current-side/Return branches on one finite-horizon
/// scale without changing the executing FSM. The caller supplies candidates
/// already checked against the real path and vehicle geometry. This Phase-1
/// evaluator remains fail closed and is intentionally free of controller state.
MpccLiteShadowResolution evaluate_mpcc_lite_shadow(
  const MpccLiteShadowRequest & request) noexcept;

struct MpccLiteShadowLeaseRequest
{
  bool has_last_feasible{false};
  bool target_matches{false};
  bool mission_generation_matches{false};
  bool phase_matches{false};
  bool side_matches{false};
  double now_sec{};
  double last_feasible_sec{};
  double maximum_age_sec{};
};

/// A shadow result is diagnostic state for one exact tactical context. Do not
/// leak a Return or opposite-side result into another phase or Mission.
bool can_reuse_mpcc_lite_shadow_last_feasible(
  const MpccLiteShadowLeaseRequest & request) noexcept;

enum class MpccLiteAuthorityAction
{
  None,
  SelectEntry,
  KeepCurrent,
  BeginReturn,
  ReplaceActive,
};

const char * to_string(MpccLiteAuthorityAction action) noexcept;

struct MpccLiteAuthorityRequest
{
  bool enabled{false};
  bool resolution_valid{false};
  bool resolution_found{false};
  bool runtime_hard_fault{false};
  bool new_entry_context{false};
  bool active_mission{false};
  bool return_active{false};
  bool return_admitted{false};
  bool same_side_replan_admitted{false};
  bool cross_side_replan_admitted{false};
  bool selected_mission_available{false};
  bool selected_mission_complete{false};
  bool selected_prefix_execution_admitted{false};
  int active_side_sign{0};
  MpccLiteShadowBranch selected_branch{MpccLiteShadowBranch::None};
};

struct MpccLiteAuthorityResolution
{
  bool valid{false};
  MpccLiteAuthorityAction action{MpccLiteAuthorityAction::None};
  int selected_side_sign{0};
};

/// Give the finite-horizon winner bounded control authority. A fresh entry may
/// use either a complete Mission or an independently admitted progressive
/// prefix. During active execution, that prefix may own one bounded same-side
/// or cross-side rolling replacement before no-return.
MpccLiteAuthorityResolution resolve_mpcc_lite_authority(
  const MpccLiteAuthorityRequest & request) noexcept;

struct MpccLiteSameSideReplanAdmissionRequest
{
  bool base_admitted{false};
  bool active_hold_feasible{false};
  double candidate_score_advantage{};
  double minimum_score_advantage{};
  double seconds_since_selected_mission{};
  double minimum_replacement_interval_sec{};
};

/// Prevent a feasible current Mission from being replaced for numerical score
/// noise or multiple times in one short rolling-planning interval. A candidate
/// may still replace a hard-infeasible hold without a score advantage.
bool should_admit_mpcc_lite_same_side_replan(
  const MpccLiteSameSideReplanAdmissionRequest & request) noexcept;

enum class OvertakeSideRetryFailureClass
{
  /// No executable candidate was observed in the current planning sample.
  /// The scene may open on the next sample, so this must not suppress search.
  PlanningSearchMiss,
  /// A selected/committed maneuver reached a physical, execution or bounded
  /// mission failure. A short same-target/side retry block prevents chatter.
  PhysicalOrCommittedFailure,
};

/// Keep opportunistic entry search live across one-sample planning misses,
/// while retaining cooldown after a real maneuver/execution failure.
bool should_arm_overtake_side_retry_block(
  OvertakeSideRetryFailureClass failure_class) noexcept;

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

enum class RuntimeWallPreplanAction
{
  None,
  RequestFreshSameSideCandidate,
  ReplaceWithFreshSameSide,
  ContractTowardCenter,
  HoldCurrentSide,
  ExitCurrentMission,
  ReturnToBaseLine,
};

struct RuntimeWallPreplanRequest
{
  bool enabled{false};
  bool active_execution{false};
  bool warning_margin_blocked{false};
  bool hard_wall_fault{false};
  bool target_continuous{false};
  bool current_body_separated{false};
  bool target_prediction_valid{false};
  bool fresh_candidate_available{false};
  bool center_contraction_evaluated{false};
  bool center_contraction_available{false};
  bool speed_preserving_return_available{false};
  bool rear_clear_confirmed{false};
  int mission_side_sign{0};
  int candidate_side_sign{0};
  double now_sec{};
  double last_replan_sec{-std::numeric_limits<double>::infinity()};
  double cooldown_sec{};
  double warning_elapsed_sec{};
  double fallback_delay_sec{};
  int replan_count{};
  int maximum_replan_count{};
};

struct RuntimeWallPreplanResolution
{
  bool valid{false};
  RuntimeWallPreplanAction action{RuntimeWallPreplanAction::None};
};

/// Detect the warning band outside the existing hard wall margin. A warning
/// can request a new candidate or atomically replace with a fresh same-side
/// Mission, but can never override physical contact or the hard wall guard.
RuntimeWallPreplanResolution resolve_runtime_wall_preplan(
  const RuntimeWallPreplanRequest & request) noexcept;

struct RuntimeWallEscapePrefixHorizonRequest
{
  double configured_shift_distance_m{};
  double nominal_hold_distance_m{};
  double current_speed_mps{};
  bool prediction_warning{false};
  double predicted_wall_ttc_sec{std::numeric_limits<double>::infinity()};
};

struct RuntimeWallEscapePrefixHorizonResolution
{
  bool valid{false};
  double shift_distance_m{};
  double hold_distance_m{};
  double total_distance_m{};
  double available_distance_m{std::numeric_limits<double>::infinity()};
};

/// Fit a local centerward transition inside the distance remaining to the
/// predicted wall-warning footprint. Hard wall and lateral-acceleration
/// feasibility remain the caller's responsibility.
RuntimeWallEscapePrefixHorizonResolution resolve_runtime_wall_escape_prefix_horizon(
  const RuntimeWallEscapePrefixHorizonRequest & request) noexcept;

struct RuntimeWallCenterContractionGoalRequest
{
  int pass_side_sign{0};
  bool current_body_footprints_separated{false};
  double current_ego_lateral_m{};
  double current_target_lateral_m{};
  double predicted_target_lateral_m{};
  double previous_goal_m{};
  double physical_target_center_separation_m{};
  double nominal_target_center_separation_m{};
  double wall_lower_bound_m{};
  double wall_upper_bound_m{};
  double maximum_centerward_adjustment_m{};
};

struct RuntimeWallCenterContractionGoalResolution
{
  bool valid{false};
  bool used_physical_clearance{false};
  double goal_m{};
  double guarded_target_lateral_m{};
  double applied_target_center_separation_m{};
  std::string reason;
};

/// Move an executing same-side Pass away from an approaching wall. Prefer the
/// nominal target clearance; when it cannot produce any centerward motion,
/// permit the physical body boundary only if the current bodies are separated
/// and the ego remains on the selected side of the target. The returned goal
/// still requires wall/kinematic preflight for the executable local prefix by
/// the caller; it does not validate the remaining Pass or Return path.
RuntimeWallCenterContractionGoalResolution resolve_runtime_wall_center_contraction_goal(
  const RuntimeWallCenterContractionGoalRequest & request) noexcept;

enum class PassEntryPhysicalGateAction
{
  Inactive,
  HoldForReplan,
  Reselect,
};

struct PassEntryPhysicalGateRequest
{
  bool enabled{false};
  bool inside_entry_window{false};
  bool warning_margin_blocked{false};
  bool hard_wall_fault{false};
  double hold_elapsed_sec{};
  double hold_traveled_m{};
  double maximum_hold_sec{};
  double maximum_hold_distance_m{};
};

struct PassEntryPhysicalGateResolution
{
  bool valid{false};
  PassEntryPhysicalGateAction action{PassEntryPhysicalGateAction::Inactive};
};

/// Keep the vehicle at its last physically valid lateral position when the
/// warning band predicts a near-term wall conflict at the ShiftOut boundary or
/// during a bounded early-Pass lease. An expired hold requests a new Mission.
PassEntryPhysicalGateResolution resolve_pass_entry_physical_gate(
  const PassEntryPhysicalGateRequest & request) noexcept;

struct CrossSideReplacementRetryThrottleRequest
{
  bool side_changed{false};
  int candidate_side_sign{0};
  int rejected_side_sign{0};
  double candidate_goal_lateral_m{};
  double rejected_goal_lateral_m{};
  double now_sec{};
  double rejected_at_sec{-std::numeric_limits<double>::infinity()};
  double cooldown_sec{};
  double goal_change_tolerance_m{};
};

/// Suppress repeated evaluation of the same rejected cross-side candidate.
/// A materially changed goal is evaluated immediately.
bool should_throttle_cross_side_replacement_retry(
  const CrossSideReplacementRetryThrottleRequest & request) noexcept;

/// A static-wall clamp may move a target after the normal lateral-acceleration
/// limiter has run. Abort the executing line when that adjusted target again
/// exceeds the configured acceleration limit instead of publishing an
/// unrealizable heading jump.
bool static_wall_clamp_requires_overtake_recovery(
  bool active_execution_phase, bool static_target_adjusted,
  double required_lateral_accel_mps2, double maximum_lateral_accel_mps2) noexcept;

struct ReachableLateralTargetRequest
{
  double current_lateral_m{};
  double desired_lateral_m{};
  double time_to_target_sec{};
  double maximum_lateral_accel_mps2{};
  double initial_lateral_velocity_mps{};
};

struct ReachableLateralTargetResolution
{
  bool valid{false};
  bool limited{false};
  double target_lateral_m{};
  double required_lateral_accel_mps2{std::numeric_limits<double>::infinity()};
};

/// Project one lateral target onto the acceleration-reachable interval from
/// the current offset and lateral velocity. Static-map users must validate the
/// returned target again after projection; reachability alone does not
/// establish wall clearance.
ReachableLateralTargetResolution resolve_reachable_lateral_target(
  const ReachableLateralTargetRequest & request) noexcept;

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

struct PassLateralGoalPolicyRequest
{
  int pass_side_sign{};
  double base_lateral_offset_m{};
  double pass_goal_target_lateral_m{};
  double separation_target_lateral_m{};
  double minimum_separation_m{};
  std::optional<double> fixed_lateral_goal_m;
  bool feasible_interval_available{false};
  double feasible_lower_bound_m{};
  double feasible_upper_bound_m{};
  bool enforce_target_separation{false};
};

struct PassLateralGoalPolicyResolution
{
  double preferred_goal_m{};
  double execution_goal_m{};
  bool target_separation_feasible{false};
};

/// Compose the latched or target-relative pass goal with the current
/// wall-feasible interval. This keeps path-goal precedence out of the ROS/model
/// orchestration layer without changing the existing target-separation policy.
PassLateralGoalPolicyResolution resolve_pass_lateral_goal_policy(
  const PassLateralGoalPolicyRequest & request) noexcept;

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

struct MinimumLateralMotionGoalRequest
{
  double base_line_lateral_m{};
  double current_lateral_m{};
  double feasible_lower_bound_m{};
  double feasible_upper_bound_m{};
  int pass_side_sign{};
  double preferred_target_clearance_buffer_m{};
};

struct MinimumLateralMotionGoalResolution
{
  bool valid{false};
  bool base_line_clear{false};
  bool current_position_clear{false};
  double goal_m{};
  double required_shift_m{std::numeric_limits<double>::infinity()};
  double applied_target_clearance_buffer_m{};
};

/// Keep the base racing line when it lies inside the vehicle/wall-inflated
/// corridor after reserving the preferred target-side clearance. Otherwise
/// return the closest point in that buffered corridor. The buffer is bounded
/// to half the available width so it cannot consume all wall-side freedom.
MinimumLateralMotionGoalResolution resolve_minimum_lateral_motion_goal(
  const MinimumLateralMotionGoalRequest & request) noexcept;

struct MinimumMotionDirectPassRequest
{
  bool base_line_direct_pass{false};
  bool tiny_shift_enabled{false};
  bool current_position_clear{false};
  bool body_clear_at_entry{false};
  double lateral_shift_m{};
  double maximum_tiny_shift_m{};
};

struct MinimumMotionDirectPassResolution
{
  bool valid{false};
  bool direct_pass{false};
  bool base_line_direct_pass{false};
  bool tiny_shift_direct_pass{false};
};

/// Classify a fully preflighted new Mission as DirectPass when either the
/// legacy base line is already usable or the ego is physically clear and only
/// a bounded same-side lateral correction remains. This does not perform or
/// replace wall, body-clear, rear-clear, or Return validation.
MinimumMotionDirectPassResolution resolve_minimum_motion_direct_pass(
  const MinimumMotionDirectPassRequest & request) noexcept;

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

struct OvertakeEntryFrontDistanceReserveRequest
{
  bool enabled{false};
  bool lateral_body_separation_established{false};
  double front_distance_m{};
  double configured_minimum_front_distance_m{};
  double body_longitudinal_clearance_m{};
  double reserve_distance_m{};
  double current_closing_speed_mps{};
  double planned_closing_speed_mps{};
  double predicted_body_clear_time_sec{};
  double prediction_margin_time_sec{};
};

struct OvertakeEntryFrontDistanceReserveResolution
{
  bool valid{false};
  bool applied{false};
  bool admitted{false};
  double closing_speed_for_budget_mps{};
  double closing_distance_budget_m{};
  double required_front_distance_m{};
};

/// Require enough center-to-center distance for the selected longitudinal
/// profile to establish physical lateral body separation before consuming the
/// protected front reserve. This entry-only policy complements the kinematic
/// rollout with an explicit execution-error budget.
OvertakeEntryFrontDistanceReserveResolution
resolve_overtake_entry_front_distance_reserve(
  const OvertakeEntryFrontDistanceReserveRequest & request) noexcept;

struct LateralClearanceClosingReserveRequest
{
  bool lateral_body_separation_established{false};
  double target_longitudinal_m{};
  double current_closing_speed_limit_mps{};
  double moving_front_hard_distance_m{};
  double body_longitudinal_clearance_m{};
  double reserve_distance_m{};
  double remaining_lateral_execution_distance_m{};
  double ego_speed_mps{};
  double minimum_speed_mps{};
  double minimum_time_sec{};
  double limiting_tolerance_mps{};
};

struct LateralClearanceClosingReserveResolution
{
  bool eligible{false};
  bool limited{false};
  double closing_speed_limit_mps{};
  double protected_front_distance_m{};
};

/// Reduce only the positive closing-speed budget until physical lateral body
/// separation is established. Longitudinal non-overlap alone must not disable
/// this protection while ego is still approaching directly behind the target.
/// The returned limit never asks ego to travel below target speed; zero means
/// speed matching.
LateralClearanceClosingReserveResolution resolve_lateral_clearance_closing_reserve(
  const LateralClearanceClosingReserveRequest & request);

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

struct OpponentMotionFilterRequest
{
  bool previous_estimate_valid{false};
  double previous_velocity_x_mps{};
  double previous_velocity_y_mps{};
  double previous_acceleration_x_mps2{};
  double previous_acceleration_y_mps2{};
  double observed_velocity_x_mps{};
  double observed_velocity_y_mps{};
  double sample_interval_sec{};
  double velocity_gain{1.0};
  double acceleration_gain{1.0};
  double maximum_acceleration_mps2{std::numeric_limits<double>::infinity()};
};

struct OpponentMotionFilterResolution
{
  bool valid{false};
  double velocity_x_mps{};
  double velocity_y_mps{};
  double acceleration_x_mps2{};
  double acceleration_y_mps2{};
};

/// Smooth a finite-difference V2X velocity and estimate bounded acceleration.
/// The first valid observation initializes velocity and leaves acceleration at
/// zero. Invalid input is fail-closed so the tracker can reset its estimate.
OpponentMotionFilterResolution update_opponent_motion_filter(
  const OpponentMotionFilterRequest & request) noexcept;

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
  /// A current ShiftOut/Pass/Return candidate has passed body-clear,
  /// rear-clear and full-path preflight. This is a stronger admission result
  /// than the coarse distance-to-next-hard-curve estimate.
  bool validated_full_mission{false};
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

struct OvertakeEntrySpeedReadinessRequest
{
  bool monitor_active{false};
  bool same_target{false};
  double now_sec{};
  double ready_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double ego_speed_mps{};
  double target_speed_mps{};
  double minimum_relative_speed_mps{};
  double confirm_sec{};
};

struct OvertakeEntrySpeedReadiness
{
  bool ready{false};
  double ready_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double stable_sec{};
  double relative_speed_mps{std::numeric_limits<double>::quiet_NaN()};
};

/// Confirm a measured entry-speed condition continuously for one target.
/// Invalid input, a target change, or a relative-speed deficit resets confirmation.
OvertakeEntrySpeedReadiness update_overtake_entry_speed_readiness(
  const OvertakeEntrySpeedReadinessRequest & request) noexcept;

struct OvertakeEntryPrearmWindowRequest
{
  bool monitor_active{false};
  bool same_target{false};
  double now_sec{};
  double start_sec{std::numeric_limits<double>::quiet_NaN()};
  double last_update_sec{std::numeric_limits<double>::quiet_NaN()};
  double traveled_m{};
  double ego_speed_mps{};
  double maximum_duration_sec{};
  double maximum_distance_m{};
  double maximum_observation_gap_sec{0.5};
};

struct OvertakeEntryPrearmWindowResolution
{
  bool active{false};
  bool timed_out{false};
  double start_sec{std::numeric_limits<double>::quiet_NaN()};
  double last_update_sec{std::numeric_limits<double>::quiet_NaN()};
  double traveled_m{};
  double elapsed_sec{};
};

/// Bound longitudinal pre-arm to one continuously observed target. Lateral
/// candidate changes do not reset the window; a target change does.
/// Missing/invalid input clears the window.
OvertakeEntryPrearmWindowResolution update_overtake_entry_prearm_window(
  const OvertakeEntryPrearmWindowRequest & request) noexcept;

struct OvertakeEngagementLeaseRequest
{
  bool enabled{false};
  bool current_target_relevant{false};
  bool prior_target_engaged{false};
  bool hard_guard_clear{false};
  bool explicit_disengage{false};
  double now_sec{};
  double last_relevant_sec{-std::numeric_limits<double>::infinity()};
  double maximum_hold_sec{};
};

struct OvertakeEngagementLeaseResolution
{
  bool active{false};
  bool hold_active{false};
  bool clear_target{false};
  double last_relevant_sec{-std::numeric_limits<double>::infinity()};
  double remaining_sec{};
};

/// Keep target identity and target-scoped entry-speed evidence across one
/// short front/side classification dropout. A held lease never marks a stale
/// target as a current front/side vehicle and therefore cannot authorize a
/// lateral Mission or stale Follow cap.
OvertakeEngagementLeaseResolution resolve_overtake_engagement_lease(
  const OvertakeEngagementLeaseRequest & request) noexcept;

struct OvertakeEntryPrearmValidationLeaseRequest
{
  bool current_mission_validated{false};
  bool same_target{false};
  bool hard_guard_clear{false};
  double now_sec{};
  double last_validated_sec{-std::numeric_limits<double>::infinity()};
  double maximum_hold_sec{};
};

struct OvertakeEntryPrearmValidationLeaseResolution
{
  bool monitor_active{false};
  bool hold_active{false};
  double last_validated_sec{-std::numeric_limits<double>::infinity()};
  double remaining_sec{};
};

/// Preserve only target-scoped entry-speed observation across a brief setup or
/// full-Mission planning miss. A held lease never authorizes lateral
/// execution; the caller must still require a current complete Mission for
/// handoff.
OvertakeEntryPrearmValidationLeaseResolution resolve_overtake_entry_prearm_validation_lease(
  const OvertakeEntryPrearmValidationLeaseRequest & request) noexcept;

struct OvertakeEntryPrearmHoldRequest
{
  bool validation_lease_active{false};
  bool prearm_window_active{false};
  bool same_target{false};
  bool hard_guard_clear{false};
  bool front_vehicle_seen{false};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_front_distance_m{};
  double cached_closing_speed_mps{std::numeric_limits<double>::quiet_NaN()};
};

/// Keep only longitudinal pre-arm ownership across a short same-target
/// candidate-generation miss. This never authorizes lateral execution.
bool can_hold_overtake_entry_prearm(
  const OvertakeEntryPrearmHoldRequest & request) noexcept;

struct OvertakeEntrySetupPrearmRequest
{
  bool setup_candidate_available{false};
  bool complete_mission_available{false};
  bool monitor_active{false};
  bool hard_guard_clear{false};
  bool front_vehicle_seen{false};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_front_distance_m{};
};

/// Admit only bounded longitudinal preparation from a body-clear-feasible
/// setup candidate. A complete Mission takes the normal entry path instead;
/// no setup result authorizes a lateral target.
bool can_use_overtake_entry_setup_prearm(
  const OvertakeEntrySetupPrearmRequest & request) noexcept;

struct NewOvertakeEntryAdmissionRequest
{
  bool overtake_requested{false};
  bool execution_committed{false};
  bool behavior_handoff_active{false};
  /// Far observation and longitudinal setup may remain active outside this
  /// window, but a fresh lateral Mission must not take execution ownership.
  bool entry_commit_window_open{false};
  bool entry_speed_ready{false};
  /// A complete ShiftOut/Pass/Return mission has passed the current geometry,
  /// body-clear deadline and rear-clear rollout checks. This authorizes
  /// longitudinal pre-arm, not lateral execution by itself.
  bool validated_mission_ready{false};
  /// Start-grid breakout owns a separate observation/corridor handshake and
  /// must not wait for a relative-speed advantage shared by equally launching
  /// vehicles.
  bool immediate_execution_override{false};
  /// A current executable Mission has already proven its body-clear timing
  /// and longitudinal closing reserve. This is deliberately distinct from
  /// validated_mission_ready: the latter may own longitudinal pre-arm only.
  bool validated_mission_execution_override{false};
};

struct NewOvertakeEntryAdmissionResolution
{
  bool execution_allowed{false};
  bool prearm_active{false};
};

/// Resolve a fresh Behavior -> Overtake admission without treating a
/// constant-target-speed mission rollout as proof of measured closing ability.
/// A validated mission with insufficient relative speed stays on the base line
/// and receives longitudinal pre-arm ownership until the measured gate passes.
NewOvertakeEntryAdmissionResolution resolve_new_overtake_entry_admission(
  const NewOvertakeEntryAdmissionRequest & request) noexcept;

struct ValidatedMissionEntryOverrideRequest
{
  bool enabled{false};
  bool mission_available{false};
  bool hard_guard_clear{false};
  bool entry_commit_window_open{false};
  bool current_position_clear{false};
  bool body_clear_deadline_checked{false};
  bool body_clear_deadline_feasible{false};
  bool entry_front_distance_reserve_applied{false};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double required_front_distance_m{std::numeric_limits<double>::infinity()};
  double planned_closing_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_planned_closing_speed_mps{};
};

/// Allow a current executable Mission to start lateral motion before measured
/// relative speed has settled. The Mission must already prove body clearance
/// and the longitudinal distance consumed by its planned closing profile.
/// Current hard guards and the normal 15 m commit window remain mandatory.
bool can_use_validated_mission_entry_override(
  const ValidatedMissionEntryOverrideRequest & request) noexcept;

struct StationaryBlockerEntryOverrideRequest
{
  bool enabled{false};
  bool validated_mission_ready{false};
  bool hard_guard_clear{false};
  bool front_vehicle_seen{false};
  bool stopped_evidence_matches_target{false};
  int stopped_observation_count{0};
  int required_stopped_observation_count{1};
  double front_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_stopped_speed_mps{};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_entry_distance_m{};
};

/// Bypass only the measured closing-speed confirmation for a confirmed
/// stationary blocker.  The caller must provide a current complete Mission
/// and all normal hard guards; invalid geometry fails closed.
bool can_override_entry_speed_for_stationary_blocker(
  const StationaryBlockerEntryOverrideRequest & request) noexcept;

struct SlowBlockerUrgentEntryOverrideRequest
{
  bool enabled{false};
  bool validated_mission_ready{false};
  bool hard_guard_clear{false};
  bool front_vehicle_seen{false};
  double relative_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_relative_speed_mps{};
  double stable_sec{};
  double minimum_stable_sec{};
  double front_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_front_speed_mps{};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_entry_distance_m{};
  double maximum_entry_distance_m{};
};

/// Shorten, but never eliminate, measured-speed confirmation when a complete
/// current Mission is about to lose its entry window behind a genuinely slow
/// front vehicle.  This does not weaken Mission, emergency, solver, wall, or
/// minimum-distance guards owned by the caller.
bool can_use_urgent_entry_for_slow_blocker(
  const SlowBlockerUrgentEntryOverrideRequest & request) noexcept;

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
  InnerPreference,
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

struct MinimumLateralMotionSideCandidate
{
  PassSide side{PassSide::None};
  bool feasible{false};
  bool base_line_clear{false};
  double required_shift_m{std::numeric_limits<double>::infinity()};
  double corridor_width_m{0.0};
  double continuous_open_distance_m{0.0};
};

struct MinimumLateralMotionSideSelectionRequest
{
  PassSide preferred{PassSide::None};
  MinimumLateralMotionSideCandidate left;
  MinimumLateralMotionSideCandidate right;
  PassSide inner_side{PassSide::None};
  double inner_preference_max_extra_shift_m{0.0};
  double inner_preference_min_corridor_width_m{0.0};
  double inner_preference_min_open_distance_m{0.0};
};

/// Prefer an executable side that preserves the base line. If neither side
/// does, prefer the curve-inside candidate when its extra lateral motion is
/// bounded; otherwise select the side requiring less lateral movement. Exact
/// ties without a known curve side retain the existing preferred-side policy.
SideSelection select_minimum_lateral_motion_side(
  const MinimumLateralMotionSideSelectionRequest & request) noexcept;

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

enum class OpponentSideReplanAction
{
  Inactive,
  KeepCurrent,
  WaitForStability,
  ReplaceWithAlternate,
  FallbackSameSide,
  BlockedByNoReturn,
};

struct PassManeuverCandidateRequest
{
  PassSide side{PassSide::None};
  std::optional<OvertakeMissionCandidate> mission;
  bool gap_available{false};
  bool execution_allowed{false};
  bool side_conflict{false};
  bool runtime_sweep_clear{false};
};

struct PassManeuverCandidateAssessment
{
  bool valid{false};
  PassSide side{PassSide::None};
  bool plan_available{false};
  bool feasible{false};
  double physical_reserve_m{-std::numeric_limits<double>::infinity()};
  double predicted_rear_clear_time_sec{std::numeric_limits<double>::infinity()};
  double predicted_minimum_speed_mps{std::numeric_limits<double>::infinity()};
  double horizon_progress_score{-std::numeric_limits<double>::infinity()};
  std::optional<OvertakeMissionCandidate> mission;
};

/// Project one controller-side Pass assessment into a common maneuver
/// candidate. This keeps admission unchanged while exposing the longitudinal
/// metrics required by the next ranking stage.
PassManeuverCandidateAssessment assess_pass_maneuver_candidate(
  const PassManeuverCandidateRequest & request) noexcept;

struct OpponentSideManeuverComparisonRequest
{
  PassManeuverCandidateAssessment current;
  PassManeuverCandidateAssessment alternate;
  double minimum_reserve_advantage_m{0.0};
  bool dynamic_ranking_enabled{false};
  double minimum_rear_clear_time_advantage_sec{1.0};
  double minimum_progress_score_advantage{0.35};
  double maximum_reserve_regression_m{0.05};
  double maximum_rear_clear_time_regression_sec{0.25};
  double maximum_minimum_speed_regression_mps{0.25};
};

enum class OpponentSideManeuverPreferenceReason
{
  None,
  CurrentInfeasible,
  RearClearTimeAdvantage,
  HorizonProgressAdvantage,
  PhysicalReserveAdvantage,
};

struct OpponentSideManeuverComparison
{
  bool valid{false};
  bool current_feasible{false};
  bool alternate_feasible{false};
  bool alternate_preferred{false};
  bool dynamic_metrics_compared{false};
  double physical_reserve_advantage_m{-std::numeric_limits<double>::infinity()};
  double rear_clear_time_advantage_sec{-std::numeric_limits<double>::infinity()};
  double minimum_speed_advantage_mps{-std::numeric_limits<double>::infinity()};
  double horizon_progress_score_advantage{
    -std::numeric_limits<double>::infinity()};
  OpponentSideManeuverPreferenceReason preference_reason{
    OpponentSideManeuverPreferenceReason::None};
};

/// Compare the current and opposite Pass candidates. The dynamic policy ranks
/// physically admitted missions by rear-clear time, the common horizon score,
/// retained minimum speed and physical reserve. It accepts a switch only when
/// the winning dimension is material and the other dimensions do not regress
/// beyond their configured bounds. no-return and target-continuity remain in
/// resolve_opponent_side_replan().
OpponentSideManeuverComparison compare_opponent_side_maneuvers(
  const OpponentSideManeuverComparisonRequest & request) noexcept;

const char * to_string(OpponentSideManeuverPreferenceReason reason) noexcept;

struct SideReplanDebounceRequest
{
  bool opportunity_active{false};
  PassSide candidate_side{PassSide::None};
  PassSide pending_side{PassSide::None};
  double pending_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double now_sec{};
};

struct SideReplanDebounceResolution
{
  bool valid{false};
  bool changed{false};
  PassSide pending_side{PassSide::None};
  double pending_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double stable_sec{0.0};
};

/// Update the existing continuous-stability debounce without owning any ROS
/// or controller state. A later change can add dropout hysteresis here without
/// duplicating FSM logic.
SideReplanDebounceResolution update_side_replan_debounce(
  const SideReplanDebounceRequest & request) noexcept;

enum class OpponentSideReplanReason
{
  None,
  Disabled,
  InvalidInput,
  InactivePhase,
  TargetInvalid,
  BodyOverlap,
  PredictedOverlap,
  TargetTooClose,
  RearClear,
  ReplacementLimit,
  AlternateUnavailable,
  CurrentPlanRetained,
  StabilityPending,
  CurrentPlanInfeasible,
  RearClearTimeAdvantage,
  HorizonProgressAdvantage,
  PhysicalReserveAdvantage,
};

struct OpponentSideReplanRequest
{
  bool enabled{false};
  bool frozen_execution_active{false};
  bool target_continuous{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool rear_clear{false};
  double target_front_distance_m{std::numeric_limits<double>::infinity()};
  double no_return_front_distance_m{0.0};
  int replacement_count{0};
  int maximum_replacements{0};
  PassSide current_side{PassSide::None};
  PassSide alternate_side{PassSide::None};
  bool current_plan_feasible{false};
  bool alternate_plan_feasible{false};
  double current_physical_reserve_m{-std::numeric_limits<double>::infinity()};
  double alternate_physical_reserve_m{-std::numeric_limits<double>::infinity()};
  double minimum_reserve_advantage_m{0.0};
  double candidate_stable_sec{0.0};
  double required_stable_sec{0.0};
  bool maneuver_ranking_checked{false};
  bool alternate_maneuver_preferred{false};
  OpponentSideManeuverPreferenceReason maneuver_preference_reason{
    OpponentSideManeuverPreferenceReason::None};
};

struct OpponentSideReplanResolution
{
  OpponentSideReplanAction action{OpponentSideReplanAction::Inactive};
  OpponentSideReplanReason reason{OpponentSideReplanReason::None};
  bool eligible{false};
  bool replacement_requested{false};
  double physical_reserve_advantage_m{-std::numeric_limits<double>::infinity()};
};

/// Reconsider a frozen pass side only before the longitudinal no-return point.
/// The caller owns candidate generation and debounce state; this pure policy
/// decides whether a complete alternate plan may atomically replace the
/// current one. A predicted overlap on the current side makes that plan
/// infeasible and triggers alternate evaluation; invalid prediction or actual
/// body overlap remains fail closed. It never permits a second replacement or
/// a side change while current footprints overlap the target.
OpponentSideReplanResolution resolve_opponent_side_replan(
  const OpponentSideReplanRequest & request) noexcept;

const char * to_string(OpponentSideReplanAction action) noexcept;
const char * to_string(OpponentSideReplanReason reason) noexcept;

enum class LastFeasibleManeuverAction
{
  Inactive,
  ReuseCurrent,
  ReuseAlternate,
  Unavailable,
  Stale,
  BlockedByHardFault,
  BlockedByNoReturn,
};

struct LastFeasibleManeuverRequest
{
  bool enabled{false};
  bool soft_failure{false};
  bool hard_fault{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool before_no_return{false};
  /// Set only by the SafeSeparation tactical reselection gate after the
  /// target has moved clearly ahead and all runtime geometry guards are clear.
  bool tactical_no_return_rearmed{false};
  bool current_candidate_available{false};
  double current_candidate_age_sec{std::numeric_limits<double>::infinity()};
  bool current_candidate_motion_fresh{true};
  bool alternate_candidate_available{false};
  bool alternate_candidate_stable{false};
  /// A fresh, complete alternate may skip temporal debounce only when the
  /// caller has independently admitted a physically separated tactical
  /// reselection window and will rerun full preflight before commit.
  bool allow_unstable_alternate_reselection{false};
  double alternate_candidate_age_sec{std::numeric_limits<double>::infinity()};
  bool alternate_candidate_motion_fresh{true};
  double maximum_candidate_age_sec{0.0};
};

struct LastFeasibleManeuverResolution
{
  LastFeasibleManeuverAction action{LastFeasibleManeuverAction::Inactive};
  bool replacement_requested{false};
  bool alternate_selected{false};
  double selected_candidate_age_sec{std::numeric_limits<double>::infinity()};
};

/// Reuse the newest complete, preflighted Mission when the active Mission
/// encounters a soft continuation failure. Cross-track replacement remains
/// bounded by the longitudinal no-return point, except for an explicitly
/// validated SafeSeparation tactical re-arm. A fresh same-side candidate may
/// refresh the frozen Mission after no-return. Hard faults, target
/// discontinuity, and current body overlap always fail closed.
LastFeasibleManeuverResolution resolve_last_feasible_maneuver(
  const LastFeasibleManeuverRequest & request) noexcept;

const char * to_string(LastFeasibleManeuverAction action) noexcept;

struct LastFeasibleCacheUpdateRequest
{
  bool identity_matches{false};
  bool hard_invalid{false};
  bool candidate_feasible{false};
};

struct LastFeasibleCacheUpdateResolution
{
  bool clear_existing{false};
  bool store_candidate{false};
  bool retain_existing{false};
};

/// Separate a transient planner miss from an identity/safety invalidation.
/// A candidate observed after an identity change may be stored immediately,
/// but a hard-invalid observation is never cached.
LastFeasibleCacheUpdateResolution resolve_last_feasible_cache_update(
  const LastFeasibleCacheUpdateRequest & request) noexcept;

struct LockedTargetGeometryObservationRequest
{
  bool shiftout_phase{false};
  bool pass_phase{false};
  bool follow_prepare_phase{false};
  bool mission_path_frozen{false};
  bool target_id_available{false};
  bool vehicle_matches_target{false};
};

/// Keep target geometry observable while a frozen Mission is paused, without
/// granting FollowPrepare any of the ShiftOut/Pass speed or brake exemptions.
bool should_observe_locked_target_geometry(
  const LockedTargetGeometryObservationRequest & request) noexcept;

struct LockedTargetNearFieldContinuityRequest
{
  bool course_geometry_valid{false};
  bool position_jump{false};
  double local_longitudinal_m{std::numeric_limits<double>::infinity()};
  double local_lateral_m{std::numeric_limits<double>::infinity()};
  double euclidean_distance_m{std::numeric_limits<double>::infinity()};
  double maximum_distance_m{};
  double maximum_lateral_m{};
};

struct LockedTargetNearFieldContinuityResolution
{
  bool geometry_valid{false};
  bool local_fallback_used{false};
};

/// Preserve a physically close locked target across a transient course
/// projection miss. The fallback is bounded in both Euclidean/course-local
/// distance and never overrides a reported position jump.
LockedTargetNearFieldContinuityResolution resolve_locked_target_near_field_continuity(
  const LockedTargetNearFieldContinuityRequest & request) noexcept;

struct DynamicMissionWaitAdmissionRequest
{
  bool enabled{false};
  bool active_execution_phase{false};
  bool mission_path_frozen{false};
  bool target_id_available{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool same_side_replacement_allowed{false};
  bool before_no_return{false};
  bool replacement_count_available{false};
  bool hard_fault{false};
  bool rear_clear_confirmed{false};
};

/// Enter a bounded dynamic wait while a fresh complete Mission can still be
/// assessed and committed. Cross-side replacement remains pre-no-return only;
/// same-side replacement may refresh a failed path after no-return.
bool can_enter_dynamic_mission_wait(
  const DynamicMissionWaitAdmissionRequest & request) noexcept;

struct DynamicMissionWaitRuntimeOwnershipRequest
{
  bool behavior_overtake{false};
  bool tactical_rolling_replan_runtime_active{false};
  bool dynamic_mission_wait_active{false};
};

/// Route an already-active dynamic wait to its execution owner without
/// broadening authority to an ordinary tactical rolling replan.  Fully
/// preflighted Mission replacement branches are evaluated before this
/// predicate by the controller.
bool should_execute_dynamic_mission_wait_runtime(
  const DynamicMissionWaitRuntimeOwnershipRequest & request) noexcept;

enum class DynamicMissionWaitAction
{
  Inactive,
  Hold,
  ResumeCurrent,
  ReplaceWithCurrent,
  ReplaceWithAlternate,
  ReleaseForFreshSearch,
  Return,
  Recovery,
};

enum class DynamicMissionWaitReason
{
  None,
  Disabled,
  WaitingForAssessment,
  BothPlansUnavailable,
  CurrentPlanRecovered,
  CurrentMissionInvalidated,
  AlternatePlanReady,
  TerminalBudgetExpired,
  RearClear,
  TargetInvalid,
  BodyOverlap,
  HardFault,
};

struct DynamicMissionWaitRequest
{
  bool enabled{false};
  bool wait_active{false};
  bool target_continuous{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool hard_fault{false};
  bool rear_clear_confirmed{false};
  bool current_mission_invalidated{false};
  bool alternate_replacement_allowed{false};
  bool assessment_completed{false};
  bool current_plan_feasible{false};
  bool current_replacement_ready{false};
  bool alternate_replacement_ready{false};
  /// The previous Pass generation consumed its immutable absolute time or
  /// distance budget. It must not be revived by ordinary current-side
  /// feasibility alone.
  bool terminal_budget_abort{false};
  /// A fresh same-side progressive prefix was revalidated against the target,
  /// wall, speed and a new bounded local execution lease.
  bool current_replacement_tactical_rearmed{false};
  /// Current overlap was accepted by the independently bounded
  /// ContactContinuation classifier.
  bool recoverable_side_contact_active{false};
};

struct DynamicMissionWaitResolution
{
  DynamicMissionWaitAction action{DynamicMissionWaitAction::Inactive};
  DynamicMissionWaitReason reason{DynamicMissionWaitReason::None};
};

/// Hold a paused overtake target while both complete Mission candidates are
/// unavailable. Only a fresh current-side assessment may resume a still-valid
/// plan; an invalidated generation can only be replaced atomically or ended.
/// A terminal budget abort additionally requires an explicitly re-armed
/// current-side prefix. Once one left/right assessment completes without an
/// admissible replacement, it releases the failed side for a fresh search
/// instead of waiting for the Mission-wide deadline.
/// Alternate replacement is separately gated so no-return cannot cause a
/// side-by-side full-track crossing.
/// Actual overlap, target discontinuity and controller/geometry hard faults
/// remain fail closed unless the bounded ContactContinuation classifier
/// accepts the current side contact.
DynamicMissionWaitResolution resolve_dynamic_mission_wait(
  const DynamicMissionWaitRequest & request) noexcept;

const char * to_string(DynamicMissionWaitAction action) noexcept;
const char * to_string(DynamicMissionWaitReason reason) noexcept;

struct DynamicMissionWaitForwardPrefixRequest
{
  bool enabled{false};
  bool wait_active{false};
  bool target_continuous{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool prefix_wall_feasible{false};
  bool hard_fault{false};
  double current_ego_speed_mps{};
  double target_speed_mps{};
  double mission_closing_speed_mps{};
  double unlatched_closing_speed_mps{};
  double maximum_closing_speed_mps{};
  double maximum_vehicle_speed_mps{};
  /// A bounded side-contact continuation may retain the current-side forward
  /// prefix without a separated-body prediction.
  bool recoverable_side_contact_active{false};
};

struct DynamicMissionWaitForwardPrefixResolution
{
  bool valid{false};
  bool active{false};
  bool full_closing_authority{false};
  bool speed_floor_active{false};
  double closing_speed_mps{};
  double target_velocity_reference_mps{};
  double target_velocity_floor_mps{};
};

/// Keep a rolling replan on its freshly wall-validated current-side prefix.
/// A clear or confirmation-filtered acceptable predicted footprint path
/// retains the frozen Mission closing request. A confirmed prediction
/// conflict keeps lateral authority but falls back to the bounded unlatched
/// closing request without a speed floor.
DynamicMissionWaitForwardPrefixResolution resolve_dynamic_mission_wait_forward_prefix(
  const DynamicMissionWaitForwardPrefixRequest & request) noexcept;

struct DynamicMissionWaitForwardAuthorityRequest
{
  bool wait_active{false};
  /// The previous runtime prefix was wall-feasible. Prediction authority is
  /// checked independently so a bounded prefix can recover immediately when
  /// its predicted conflict clears.
  bool wall_validated_forward_prefix_active{false};
  bool target_continuous{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  /// True for a clear sweep or for an unconfirmed transient predicted
  /// overlap. A continuously confirmed overlap must set this false.
  bool predicted_body_footprint_path_acceptable{false};
};

/// Carry a wall-validated DynamicMissionWait forward prefix across the
/// Behavior arbitration and a same-side Pass-plan replacement. Current target
/// and body geometry must still be valid; the prior prefix alone is never
/// sufficient to retain authority.
bool can_handoff_dynamic_mission_wait_forward_authority(
  const DynamicMissionWaitForwardAuthorityRequest & request) noexcept;

struct OvertakeMissionOwnershipRequest
{
  bool shiftout_phase{false};
  bool pass_phase{false};
  bool follow_prepare_phase{false};
  bool return_phase{false};
  bool recovery_phase{false};
  bool previous_behavior_overtake{false};
  bool front_matches_locked_target{false};
  /// DynamicMissionWait is a rolling tactical replan, not an ordinary Follow
  /// pause. Keep longitudinal ownership while a fresh lateral prefix is found.
  bool rolling_replan_phase{false};
  /// The tactical pause interrupted Pass rather than pre-commit ShiftOut.
  bool follow_prepare_origin_pass{false};
};

struct OvertakeMissionOwnershipResolution
{
  bool mission_active{false};
  bool committed_execution_active{false};
  bool committed_pass_active{false};
  /// Narrow contact-classifier context. This does not broadly restore Pass
  /// behavior ownership while FollowPrepare is active.
  bool committed_pass_contact_context_active{false};
  bool paused_mission_active{false};
  bool rolling_replan_active{false};
  bool behavior_continuation_assessment_active{false};
  bool behavior_entry_assessment_active{true};
  bool overtake_line_owns_locked_target_speed{false};
  bool generic_follow_owns_locked_target_speed{true};
};

/// Classify the current OvertakeLine mission and longitudinal speed owner.
///
/// This is intentionally policy-neutral: it preserves the existing distinction
/// between Behavior entry/continuation assessment and committed line execution.
/// Hard guards remain the responsibility of their existing callers.
OvertakeMissionOwnershipResolution resolve_overtake_mission_ownership(
  const OvertakeMissionOwnershipRequest & request) noexcept;

struct CommittedBehaviorOwnershipGuardRequest
{
  bool locked_target_seen{false};
  bool target_identity_continuous{false};
  bool locked_target_position_jump{false};
  bool locked_target_course_progress_rejected{false};
  bool locked_target_pass_side_intrusion{false};
  bool explicit_forbidden_waypoint{false};
  bool emergency_front_risk{false};
  bool solver_recovery_requested{false};
};

struct CommittedBehaviorOwnershipGuardResolution
{
  bool target_identity_available{false};
  bool hard_fault_present{false};
  bool ownership_allowed{false};
};

/// Resolve the target-continuity and hard-fault guards shared by committed
/// ShiftOut and Pass behavior ownership. Phase-specific path and geometry
/// authority is deliberately handled by the respective caller.
CommittedBehaviorOwnershipGuardResolution resolve_committed_behavior_ownership_guards(
  const CommittedBehaviorOwnershipGuardRequest & request) noexcept;

struct CommittedPassGeometryOwnershipRequest
{
  bool lateral_exclusion_latched{false};
  bool minimum_motion_front_cap_release_latched{false};
  bool current_body_footprints_separated{false};
  bool current_body_footprint_overlap_confirmed{true};
  bool committed_pass_attack_mode_enabled{false};
  bool body_clear_handoff_active{false};
  /// Result of the bounded side-contact classifier. The classifier owns its
  /// duration, progress, velocity and side-geometry limits.
  bool recoverable_side_contact_active{false};
};

struct CommittedPassGeometryOwnershipResolution
{
  bool pass_release_latched{false};
  bool body_clear_handoff_owns_pass{false};
  bool current_overlap_grace_active{false};
  bool recoverable_side_contact_owns_pass{false};
  bool pass_authority_available{false};
  bool current_geometry_acceptable{false};
  bool ownership_allowed{false};
};

/// Resolve the current geometry sources which may own a validated Pass. A
/// confirmed physical overlap is accepted only when the independently bounded
/// ContactContinuation classifier marks it recoverable.
CommittedPassGeometryOwnershipResolution resolve_committed_pass_geometry_ownership(
  const CommittedPassGeometryOwnershipRequest & request) noexcept;

struct CommittedPassBehaviorOwnershipRequest
{
  bool committed_pass_active{false};
  bool validated_fixed_line{false};
  bool mission_side_valid{false};
  bool lateral_exclusion_latched{false};
  bool minimum_motion_front_cap_release_latched{false};
  bool locked_target_seen{false};
  /// The frozen Mission target still matches the current front/side target.
  /// This bridges one course-relative geometry classification miss; it must
  /// never be asserted for a stale or different target ID.
  bool target_identity_continuous{false};
  bool locked_target_position_jump{false};
  bool locked_target_course_progress_rejected{false};
  bool current_body_footprints_separated{false};
  bool current_body_footprint_overlap_confirmed{true};
  bool committed_pass_attack_mode_enabled{false};
  bool recoverable_side_contact_active{false};
  bool locked_target_pass_side_intrusion{false};
  bool explicit_forbidden_waypoint{false};
  bool emergency_front_risk{false};
  bool solver_recovery_requested{false};
  /// A freshly admitted fixed path may own the ShiftOut-to-Pass phase handoff
  /// before the ordinary lateral/front-cap latch is established. The caller
  /// must bound this with the predicted hard-distance deadline.
  bool body_clear_handoff_active{false};
};

/// Keep Behavior in Overtake while a validated Pass owns execution.
///
/// Entry-only gap, curve, completion-distance and candidate-quality decisions
/// are intentionally absent. Live corridor, wall, lateral-acceleration and
/// solver checks still execute downstream in OvertakeLine. Confirmed current
/// overlap may retain ownership only through bounded ContactContinuation;
/// target-continuity and hard-fault failures always release it.
bool can_preserve_committed_pass_behavior(
  const CommittedPassBehaviorOwnershipRequest & request) noexcept;

struct CommittedShiftOutBehaviorOwnershipRequest
{
  bool committed_shiftout_active{false};
  bool validated_fixed_line{false};
  bool mission_side_valid{false};
  bool body_clear_handoff_active{false};
  bool locked_target_seen{false};
  /// See CommittedPassBehaviorOwnershipRequest::target_identity_continuous.
  bool target_identity_continuous{false};
  bool locked_target_position_jump{false};
  bool locked_target_course_progress_rejected{false};
  bool locked_target_pass_side_intrusion{false};
  bool explicit_forbidden_waypoint{false};
  bool emergency_front_risk{false};
  bool solver_recovery_requested{false};
  bool body_clear_deadline_checked{false};
};

/// Keep Behavior in Overtake while a deadline-feasible, immutable ShiftOut
/// owns execution. Entry-only gap and candidate-quality re-evaluation cannot
/// revoke it, while target continuity, emergency, explicit map prohibition and
/// solver recovery remain hard ownership-release conditions. Live path, wall
/// and lateral-acceleration checks remain downstream responsibilities.
bool can_preserve_committed_shiftout_behavior(
  const CommittedShiftOutBehaviorOwnershipRequest & request) noexcept;

struct BodyClearExecutionHandoffRequest
{
  bool committed_execution_active{false};
  bool body_clear_deadline_checked{false};
  bool body_clear_deadline_feasible{false};
  double now_sec{};
  double hard_deadline_sec{-std::numeric_limits<double>::infinity()};
  bool current_body_footprints_separated{false};
  bool current_body_footprint_overlap_confirmed{true};
  /// The ordinary Pass/front-cap latch has accepted longitudinal ownership.
  /// Once true, the special phase-boundary handoff must end immediately.
  bool ordinary_pass_ownership_latched{false};
  /// Live time until the configured hard longitudinal gap. Infinity means
  /// unavailable or non-closing and preserves the frozen absolute deadline.
  double live_hard_gap_ttc_sec{std::numeric_limits<double>::infinity()};
  double execution_margin_sec{};
};

enum class BodyClearExecutionHandoffReleaseReason
{
  None,
  InactiveExecution,
  InvalidDeadline,
  NormalPassOwnership,
  CurrentOverlap,
  Expired,
};

const char * to_string(BodyClearExecutionHandoffReleaseReason reason) noexcept;

struct BodyClearExecutionHandoffResolution
{
  bool valid{false};
  bool active{false};
  bool satisfied{false};
  bool expired{false};
  bool live_deadline_contracted{false};
  double remaining_sec{};
  double effective_deadline_sec{-std::numeric_limits<double>::infinity()};
  BodyClearExecutionHandoffReleaseReason release_reason{
    BodyClearExecutionHandoffReleaseReason::None};
};

/// Resolve the bounded longitudinal-ownership handoff between a candidate's
/// predicted body-clear event and the ordinary Pass latches. The observed
/// separation is reported independently. The handoff ends as soon as the
/// ordinary Pass latch owns execution, on confirmed current overlap, or at the
/// earlier of the frozen and live-TTC hard-distance deadlines.
BodyClearExecutionHandoffResolution resolve_body_clear_execution_handoff(
  const BodyClearExecutionHandoffRequest & request) noexcept;

struct BodyClearHandoffSpeedReferenceRequest
{
  bool handoff_active{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  double current_speed_mps{};
  double target_speed_mps{std::numeric_limits<double>::infinity()};
  double allowed_closing_speed_mps{};
  double maximum_speed_mps{std::numeric_limits<double>::infinity()};
};

struct BodyClearHandoffSpeedReferenceResolution
{
  bool valid{false};
  bool hold_active{false};
  double target_velocity_reference_mps{std::numeric_limits<double>::infinity()};
};

/// Keep Mission ownership during a bounded body-clear handoff, but do not add
/// closing speed while its future footprint sweep is unknown or overlapping.
/// This shapes the velocity reference only; it does not add a hard MPC bound.
BodyClearHandoffSpeedReferenceResolution resolve_body_clear_handoff_speed_reference(
  const BodyClearHandoffSpeedReferenceRequest & request) noexcept;

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
/// A FollowPrepare resume keeps the side owned by the committed mission. A
/// Behavior revalidation may confirm that side, but cannot redirect the vehicle
/// across the target to the opposite side in the same mission.
OvertakeExecutionSideResolution resolve_overtake_execution_side(
  const OvertakeExecutionSideRequest & request) noexcept;

/// Keep a paused mission's candidate goal from retreating toward the target.
/// Invalid inputs preserve the finite current lateral position.
double clamp_paused_resume_goal_outward(
  int mission_side_sign, double current_lateral_m, double candidate_goal_lateral_m) noexcept;

struct PausedPassDirectResumeRequest
{
  bool resuming_paused_mission{false};
  int mission_side_sign{0};
  int behavior_side_sign{0};
  bool execution_corridor_valid{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool target_lateral_prediction_valid{false};
  double target_relative_lateral_m{0.0};
  double target_predicted_relative_lateral_m{0.0};
  double required_lateral_clearance_m{0.0};
  double current_lateral_m{0.0};
  double goal_lateral_m{0.0};
};

/// Skip a redundant ShiftOut only after the committed side, current and
/// predicted lateral separation, non-inward goal, and the revalidated
/// execution corridor all agree. Longitudinal acceleration belongs to the
/// Pass speed policy after this lateral-safety admission.
bool can_resume_paused_pass_directly(
  const PausedPassDirectResumeRequest & request) noexcept;

enum class PausedExecutionOrigin
{
  None,
  ShiftOut,
  Pass,
  Recovery,
};

enum class PausedReplacementExecutionMode
{
  RestartShiftOut,
  ContinuePass,
};

/// A fresh same-side Mission selected while a Pass is paused is a remaining
/// Pass trajectory, not a new overtake entry. Cross-side replacement and a
/// paused ShiftOut still need ShiftOut execution semantics.
PausedReplacementExecutionMode resolve_paused_replacement_execution_mode(
  PausedExecutionOrigin origin, bool side_changed) noexcept;
const char * to_string(PausedReplacementExecutionMode mode) noexcept;

enum class PausedExecutionResumeAction
{
  Hold,
  ResumeShiftOut,
  ResumePass,
};

struct PausedExecutionResumeRequest
{
  bool safety_brake_pause{false};
  PausedExecutionOrigin origin{PausedExecutionOrigin::None};
  bool dynamic_mission_wait_active{false};
  bool validated_frozen_path{false};
  bool mission_side_valid{false};
  bool body_clear_deadline_checked{false};
  bool body_clear_deadline_feasible{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool target_course_progress_discontinuity{false};
  bool target_pass_side_intrusion{false};
  bool forbidden_waypoint{false};
  bool emergency_front_risk{false};
  bool solver_recovery_requested{false};
  bool mission_invalidated{false};
  bool physical_path_hard_fault{false};
  bool direct_pass_lateral_clear{false};
};

/// Resume only a transient SafetyBrake pause of an already validated frozen
/// execution. A pause before lateral separation returns to ShiftOut; once the
/// direct-pass clearance contract is satisfied it may resume Pass immediately.
PausedExecutionResumeAction resolve_paused_execution_resume(
  const PausedExecutionResumeRequest & request) noexcept;
const char * to_string(PausedExecutionResumeAction action) noexcept;

enum class OvertakeLineTransitionAction
{
  None,
  RecoverPhysicalWallContact,
  RejectEntryWallMargin,
  ResumePassForReturnCorridorBlocker,
  HoldPassForRearClearBeforeWallMarginRecovery,
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
  bool committed_overtake_execution_active{false};
  bool committed_overtake_handoff_safe{false};
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

struct DynamicObstacleCruiseAuthorityRequest
{
  bool enabled{false};
  bool dynamic_corridor_vehicle_is_nearest{false};
  bool activation_predicted{false};
  bool candidate_confirmation_ready{false};
  bool continuing_legacy_low_speed_avoidance{false};
};

struct DynamicObstacleCruiseAuthorityResolution
{
  /// Promote the wider course-corridor observation into the ordinary front
  /// target used by the receding-horizon left/right Mission evaluation.
  bool promote_to_dynamic_front{false};
  /// A new stopped/slow encounter must not be consumed by the legacy local
  /// bypass before the ordinary dynamic-obstacle planner gets first refusal.
  bool defer_legacy_low_speed_entry{false};
};

/// Give the ordinary all-V2X receding-horizon planner first authority over a
/// predicted course blocker. Existing legacy low-speed control is kept until
/// it completes so enabling the policy cannot switch lateral owners in the
/// middle of an already executing maneuver.
DynamicObstacleCruiseAuthorityResolution resolve_dynamic_obstacle_cruise_authority(
  const DynamicObstacleCruiseAuthorityRequest & request) noexcept;

struct DynamicObstacleCruiseActivationRequest
{
  bool enabled{false};
  bool start_grid_grace_active{false};
  bool candidate_present{false};
  bool velocity_observation_valid{false};
  bool future_path_overlap{false};
  bool position_jump{false};
  double forward_distance_m{std::numeric_limits<double>::infinity()};
  double maximum_scan_distance_m{std::numeric_limits<double>::infinity()};
  double ego_speed_mps{};
  double target_speed_mps{};
  double entry_front_reserve_m{};
  double activation_horizon_sec{};
  double minimum_closing_speed_mps{};
};

struct DynamicObstacleCruiseActivationResolution
{
  bool active{false};
  bool inside_entry_reserve{false};
  double closing_speed_mps{};
  double time_to_entry_sec{std::numeric_limits<double>::infinity()};
};

/// Start tactical evaluation when the target is predicted to enter the
/// overtake-entry reserve within the configured time horizon. The scan
/// distance only bounds computation; it does not independently activate the
/// planner. A target already inside the reserve is admitted even when both
/// vehicles are stationary, which lets a confirmed stopped blocker be routed
/// to the ordinary dynamic planner after race start.
DynamicObstacleCruiseActivationResolution resolve_dynamic_obstacle_cruise_activation(
  const DynamicObstacleCruiseActivationRequest & request) noexcept;

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
  bool validated_overtake_mission_available{false};
  bool has_front_vehicle{false};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double front_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_stopped_speed_mps{};
  double stopped_detection_distance_m{};
};

/// Yield the generic OvertakeLine only when the stopped-vehicle bypass owns
/// the lateral plan. A candidate without a feasible local path must not erase
/// a committed generic pass mission or a complete generic Mission that is
/// ready for immediate execution.
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

/// Re-evaluate the opposite stopped-vehicle pass side whenever the currently
/// selected side is no longer executable. This deliberately applies after
/// direct control has started as well as at initial admission; the caller must
/// still run the complete V2X and static-wall preflight before committing it.
bool should_try_alternate_low_speed_pass_side(
  bool automatic_side_selection, bool primary_side_feasible,
  int primary_side_sign) noexcept;

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

/// Direct stopped-vehicle control always starts in Shift. Corridor membership
/// alone does not prove that ego has settled on the pass line or that the
/// locked vehicle footprint remains separated through the prediction horizon.
LowSpeedDirectControlPhase resolve_low_speed_direct_control_entry_phase(
  bool pass_corridor_entered) noexcept;

struct LowSpeedDirectPassAdmissionRequest
{
  LowSpeedDirectControlPhase phase{LowSpeedDirectControlPhase::Shift};
  bool pass_corridor_entered{false};
  bool pose_settled{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
};

/// Advance from Shift to Pass only after the physical pose and the locked
/// target's current/predicted body geometry agree with the validated corridor.
bool can_enter_low_speed_direct_pass(
  const LowSpeedDirectPassAdmissionRequest & request) noexcept;

/// A stopped-vehicle direct pass may select another fully validated side only
/// while it is still shifting toward the pass corridor. Once Pass owns the
/// maneuver, crossing the target to a newly selected side is forbidden until
/// the mission has completed and a fresh mission is admitted.
bool can_update_low_speed_direct_pass_side(
  LowSpeedDirectControlPhase phase, int current_side_sign,
  int candidate_side_sign) noexcept;

enum class LowSpeedRetainedPassRejectReason
{
  None,
  StaticPathInfeasible,
  TargetIdentityUnavailable,
  TargetNotSeen,
  TargetPositionJump,
  CurrentBodyOverlap,
  PredictionUnavailable,
  PredictedFootprintOverlap,
  SideOrderingConflict,
  InvalidGeometry,
};

const char * to_string(LowSpeedRetainedPassRejectReason reason) noexcept;

struct LowSpeedRetainedPassValidationRequest
{
  bool static_path_feasible{false};
  bool target_identity_available{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  int pass_side_sign{0};
  double target_relative_lateral_m{std::numeric_limits<double>::infinity()};
  double predicted_target_relative_lateral_m{
    std::numeric_limits<double>::infinity()};
  double ordering_margin_m{0.0};
};

struct LowSpeedRetainedPassValidationResult
{
  bool valid{false};
  LowSpeedRetainedPassRejectReason reason{
    LowSpeedRetainedPassRejectReason::StaticPathInfeasible};
};

/// Validate the target-aware continuation used after a stopped target moves
/// from the forward-only local planner into the side-by-side region. Static
/// wall feasibility alone is insufficient: the same observed target must
/// remain physically and predictively separated on the committed side.
LowSpeedRetainedPassValidationResult resolve_low_speed_retained_pass_validation(
  const LowSpeedRetainedPassValidationRequest & request) noexcept;

struct LowSpeedDirectCorridorStopRequest
{
  bool direct_control_active{false};
  bool rejoin_active{false};
  LowSpeedDirectControlPhase phase{LowSpeedDirectControlPhase::Shift};
  bool local_path_active{false};
  bool local_path_feasible{false};
  bool has_front_vehicle{false};
  bool has_side_vehicle{false};
  bool has_clearance_vehicle{false};
  bool retained_pass_path_feasible{false};
};

/// Stop an active stopped-vehicle direct maneuver when its live local corridor
/// is unavailable. A validated Pass may finish moving a target from front to
/// side/rear even after the forward-only local planner becomes inactive.
/// Rejoin is protected by independent wall guards and no longer depends on the
/// passed vehicle corridor.
bool should_stop_low_speed_direct_control_for_corridor(
  const LowSpeedDirectCorridorStopRequest & request) noexcept;

/// Select the bounded direct-control speed without handing ownership to MPC
/// inside a stopped-vehicle pack.
double resolve_low_speed_direct_control_velocity(
  LowSpeedDirectControlPhase phase,
  double shift_velocity_mps,
  double pass_velocity_mps,
  double rejoin_velocity_mps,
  double maximum_velocity_mps);

struct LowSpeedDirectControlEntryFeasibilityRequest
{
  double current_speed_mps{};
  double shift_speed_mps{};
  double maximum_deceleration_mps2{};
  double forward_distance_m{};
  double front_reserve_m{};
  double control_latency_sec{};
};

struct LowSpeedDirectControlEntryFeasibility
{
  bool valid{false};
  bool feasible{false};
  double available_distance_m{};
  double required_distance_m{};
};

/// Direct low-speed steering may take ownership only when ego can reach the
/// Shift speed before consuming the reserved front clearance. When this is
/// false the same local path remains available to the horizon MPC.
LowSpeedDirectControlEntryFeasibility
resolve_low_speed_direct_control_entry_feasibility(
  const LowSpeedDirectControlEntryFeasibilityRequest & request) noexcept;

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

struct LowSpeedDirectSteeringBounds
{
  double lower_rad{0.0};
  double upper_rad{0.0};
};

/// Bound direct-shift steering around the nominal curve command, intersected
/// with the steering-rate interval reachable from the previous command. If
/// the intervals do not overlap, move one rate-limited step toward nominal.
LowSpeedDirectSteeringBounds resolve_low_speed_direct_steering_bounds(
  double previous_steering_rad, double nominal_curve_steering_rad,
  double maximum_steering_rad, double maximum_steering_step_rad,
  double current_speed_mps, double wheelbase_m,
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
  /// Rear clearance was confirmed from the last continuous course-progress
  /// observation during the bounded target-hold window.
  bool rear_clear_from_last_observation{false};
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
  /// The Return owner permits the current Mission to resume Pass. A tactical
  /// disengagement owns Return until completion and sets this false so a
  /// stale pre-Return behavior sample cannot immediately reverse the choice.
  bool return_owner_allows_reacquire{false};
  bool stable_target_id{false};
  bool same_target{false};
  bool same_side{false};
  bool gap_available{false};
  bool execution_allowed{false};
  double return_elapsed_sec{};
  double reacquire_window_sec{};
  double return_progress{};
  double max_return_progress{};
  bool rear_clear_confirmed_latched{false};
};

/// Allow Return -> Pass only when the Return owner permits it, for the same
/// stable target and pass side early in Return, and never after rear clearance
/// has completed the pass mission.
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
  bool replacement_mission_available{false};
  bool replacement_deadline_checked{false};
  bool replacement_deadline_feasible{false};
  bool replacement_goal_available{false};
};

/// Allow Recovery -> ShiftOut only when the same executable pass opportunity
/// has a complete, freshly evaluated mission replacement.
bool can_reacquire_during_recovery(const RecoveryReacquireRequest & request) noexcept;

struct CompletedTargetReacquireSuppressionRequest
{
  bool completed_target_block_active{false};
  bool candidate_matches_completed_target{false};
  bool committed_mission_active{false};
  double now_sec{};
  double block_until_sec{};
};

/// Suppress only a new overtake entry for a just-completed target. Collision,
/// Follow and SafetyBrake ownership remain with the caller.
bool should_suppress_completed_target_reacquire(
  const CompletedTargetReacquireSuppressionRequest & request) noexcept;

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

struct ReturnHandoffConvergenceRequest
{
  bool return_active{false};
  bool phase_hold_elapsed{false};
  bool return_corridor_blocked{false};
  bool solver_ready{false};
  double now_sec{};
  double convergence_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double lateral_error_m{};
  double heading_error_rad{};
  double lateral_tolerance_m{};
  double heading_tolerance_rad{};
  double confirmation_sec{};
};

struct ReturnHandoffConvergenceResolution
{
  bool observation_valid{false};
  bool instantaneously_converged{false};
  bool handoff_confirmed{false};
  double convergence_since_sec{std::numeric_limits<double>::quiet_NaN()};
  double converged_duration_sec{};
};

/// Confirm that Return has physically converged to the base path before
/// releasing lateral ownership. A distance-only Return completion must not
/// hand an offset or misaligned vehicle back to the ordinary path tracker.
ReturnHandoffConvergenceResolution update_return_handoff_convergence(
  const ReturnHandoffConvergenceRequest & request);

enum class PausedMissionExpiryReason
{
  Active,
  TimeLimit,
  DistanceLimit,
};

struct PausedMissionExpiryRequest
{
  bool follow_prepare_active{false};
  double elapsed_sec{};
  double traveled_distance_m{};
  double timeout_sec{};
  double maximum_distance_m{};
};

/// Bound one paused mission. Disabled (zero) limits are ignored, while invalid
/// measurements never expire a mission by themselves.
PausedMissionExpiryReason resolve_paused_mission_expiry(
  const PausedMissionExpiryRequest & request) noexcept;

enum class PausedMissionTerminalAction
{
  Hold,
  Return,
  Recovery,
  Expire,
};

enum class PausedMissionTerminalReason
{
  None,
  TimeLimit,
  DistanceLimit,
  TargetPositionJump,
  TargetCourseProgressDiscontinuity,
  TargetStale,
  ForbiddenWaypoint,
  RearClear,
  RearClearPendingAfterLimit,
};

struct PausedMissionTerminalRequest
{
  bool follow_prepare_active{false};
  double elapsed_sec{};
  double traveled_distance_m{};
  double timeout_sec{};
  double maximum_distance_m{};
  bool target_position_jump{false};
  bool target_course_progress_discontinuity{false};
  bool target_stale{false};
  bool forbidden_waypoint{false};
  bool rear_clear_confirmed{false};
  bool retain_until_rear_clear_on_expiry{false};
};

struct PausedMissionTerminalResolution
{
  PausedMissionTerminalAction action{PausedMissionTerminalAction::Hold};
  PausedMissionTerminalReason reason{PausedMissionTerminalReason::None};
};

/// Resolve terminal handling for one paused pass mission.
///
/// Expiry keeps its historical priority unless rear-clear retention is
/// explicitly requested. In that mode, target faults still recover and
/// rear-clear still returns, but an otherwise healthy expired pause is held for
/// the controller's total Mission budget and replan path.
PausedMissionTerminalResolution resolve_paused_mission_terminal(
  const PausedMissionTerminalRequest & request) noexcept;
const char * to_string(PausedMissionTerminalReason reason) noexcept;

struct DynamicMissionWaitRetentionRequest
{
  bool tactical_wait_active{false};
  bool pass_origin{false};
  bool committed_execution{false};
  bool forward_prefix_active{false};
  /// The latest wall-validated prefix owns the complete Mission closing
  /// request instead of the bounded unlatched fallback.
  bool full_closing_authority{false};
  /// A continuous lateral path remains valid under its short runtime lease.
  bool continuous_dp_execution_active{false};
  /// Current measured wall/target/controller state has no hard execution fault.
  bool runtime_hard_fault{false};
  /// Target longitudinal separation improved recently enough that extending
  /// the short re-selection lease is still advancing the pass.
  bool target_progress_recent{false};
};

/// Allow a short DynamicMissionWait lease to outlive its re-selection limit
/// only when execution has already committed and a physically validated
/// forward prefix still owns useful execution authority while longitudinal
/// progress remains recent. Pre-commit ShiftOut waits and stale/degraded Pass
/// prefixes therefore expire into a fresh left/right search instead of
/// consuming the Mission-wide time budget as a passive FollowPrepare.
bool can_retain_dynamic_mission_wait_until_rear_clear(
  const DynamicMissionWaitRetentionRequest & request) noexcept;

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

struct RecoveryMissionRetentionRequest
{
  bool normal_recovery_complete{false};
  bool solver_recovery_active{false};
  bool actual_wall_physical_contact{false};
  bool locked_target_seen{false};
  bool target_position_jump{false};
  bool overtake_forbidden_waypoint{false};
  double target_longitudinal_m{};
  double return_clear_distance_m{};
  bool mission_retention_forbidden{false};
};

/// Preserve the current side/target mission after an ordinary lateral
/// Recovery only while the same target still requires completion. A terminal
/// mission abort (for example the whole-Mission time budget) must not be
/// resurrected after lateral Recovery completes. This is the existing
/// lifecycle policy extracted from the controller for isolated tests.
bool should_retain_pass_mission_after_recovery(
  const RecoveryMissionRetentionRequest & request) noexcept;

/// A hard runtime or future execution-path fault that survives a completed
/// Recovery invalidates the retained Mission; sending the same frozen path
/// through Recovery again only loops.
bool should_terminate_recovery_retained_mission(
  bool recovery_retention_active, bool continuation_hard_infeasible) noexcept;

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

struct FrontHazardTargetContinuityRequest
{
  bool held_target_matches{false};
  bool observation_valid{false};
  bool course_progress_valid{false};
  double course_longitudinal_m{};
  double local_longitudinal_m{};
  double self_distance_m{};
  double course_relative_lateral_m{};
  double local_relative_lateral_m{};
  double rear_clear_distance_m{};
  double danger_lateral_range_m{};
};

struct FrontHazardTargetContinuityResolution
{
  bool near_field_conflict{false};
  bool rear_clear{false};
};

/// Resolve continuity for the target protected by the short front-hazard hold.
///
/// A close target that still overlaps the inflated lateral danger band is not rear-clear merely
/// because a rapidly rotating local tangent reports it behind the ego vehicle. Course progress is
/// preferred for true rear-clear classification when available. Invalid observations fail open to
/// the bounded timer; they never extend the hold indefinitely.
FrontHazardTargetContinuityResolution resolve_front_hazard_target_continuity(
  const FrontHazardTargetContinuityRequest & request) noexcept;

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

struct CommittedCorridorFrontDangerSuppressionRequest
{
  bool enabled{false};
  bool active_shiftout_or_pass{false};
  /// A wall-validated DynamicMissionWait prefix owns forward motion while the
  /// lateral plan is being atomically replaced in FollowPrepare. This already
  /// includes the shared predicted-overlap confirmation result.
  bool dynamic_wait_forward_authority_active{false};
  /// The current or held hazard identity selected by the caller matches the
  /// locked Mission target. Resolve each source separately, never by speed ownership.
  bool nearest_front_matches_locked_target{false};
  bool validated_fixed_corridor{false};
  bool inter_vehicle_corridor{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool current_body_footprint_overlap_confirmed{true};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool prior_front_cap_release_active{false};
  bool predicted_body_footprint_overlap_confirmed{true};
  bool minimum_motion_side_by_side_escape_active{false};
  /// Pass or a Pass-origin DynamicMissionWait contact context.
  bool pass_phase{false};
  bool committed_pass_attack_mode_enabled{false};
  bool recoverable_side_contact_active{false};
  /// A frozen Mission proved body clear before the hard longitudinal gap and
  /// its runtime handoff has not expired. This permits the validated path to
  /// own a prediction-only approach across the ShiftOut-to-Pass boundary.
  bool validated_body_clear_handoff_active{false};
};

/// Suppress a longitudinal-only front danger stop only after a normal committed
/// overtake has a validated fixed corridor and current 2D body footprints are
/// separated. An already released competition-simulation Pass may debounce a
/// single current-overlap sample. A previously released minimum-motion Pass may
/// also share the bounded predicted-overlap confirmation used by its front-cap policy. In the
/// optional competition-simulation attack mode, an already released Pass may
/// ignore future overlap while the current footprints and target continuity
/// remain valid. A frozen ShiftOut with a feasible body-clear deadline may do
/// the same while current bodies remain separated. The bounded body-clear
/// handoff may span ShiftOut and early Pass, but never hides current overlap.
/// A Pass-origin DynamicMissionWait may bootstrap this authority only through
/// the independently bounded recoverable side-contact classifier.
/// Wall/path execution guards remain owned by OvertakeLine.
bool can_suppress_committed_corridor_front_danger(
  const CommittedCorridorFrontDangerSuppressionRequest & request) noexcept;

struct FrontDangerTargetIdentityRequest
{
  std::string locked_target_id;
  std::string nearest_front_id;
  bool hazard_hold_active{false};
  std::string hazard_hold_target_id;
};

struct FrontDangerTargetIdentityResolution
{
  bool current_front_matches_locked_target{false};
  bool held_hazard_matches_locked_target{false};
};

/// Match current and held front-danger ownership independently by vehicle
/// identity, never by longitudinal speed-policy ownership.
FrontDangerTargetIdentityResolution resolve_front_danger_target_identity(
  const FrontDangerTargetIdentityRequest & request) noexcept;

struct CommittedPassBodyGeometryRequest
{
  bool shiftout_phase{false};
  bool pass_phase{false};
  bool validated_shiftout_body_clear_deadline{false};
  bool minimum_motion_corridor_active{false};
  bool prior_front_cap_release_active{false};
  bool target_seen{false};
  bool target_position_jump{false};
  bool current_body_footprints_separated{false};
  bool footprint_prediction_valid{false};
  bool predicted_body_footprint_sweep_separated{false};
  bool forward_completion_latched{false};
  double target_longitudinal_m{};
  double ego_vehicle_length_m{};
  double target_vehicle_length_m{};
};

struct CommittedPassBodyGeometryResolution
{
  double body_longitudinal_clearance_m{};
  bool side_by_side_escape_active{false};
  bool raw_predicted_body_overlap{false};
  bool predicted_overlap_confirmation_eligible{false};
};

/// Resolve the body geometry shared by behavior-level front danger and the
/// OvertakeLine front-cap policy. An already released validated ShiftOut may
/// share the same bounded predicted-overlap confirmation as Pass. A latched
/// side-by-side forward completion may also use that confirmation; an unlatched
/// side-by-side candidate may not. Timer ownership remains with the caller.
CommittedPassBodyGeometryResolution resolve_committed_pass_body_geometry(
  const CommittedPassBodyGeometryRequest & request) noexcept;

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
  double target_steering_rad{};
  double max_steering_rad{};
  double steer_rate_radps{};
  double step_sec{};
};

/// Move a solver-fallback steering command toward a bounded target without exceeding the rate.
double rate_limit_solver_fallback_steering_toward_target(
  const SolverFallbackSteeringRequest & request);

struct SolverFallbackSteeringHoldRequest
{
  int consecutive_failures{0};
  int steering_hold_cycles{0};
  bool force_release{false};
};

/// Release the last-feasible steering hold after a bounded failure window.
///
/// force_release bypasses the hold window. Negative counters throw std::invalid_argument.
bool should_release_solver_fallback_steering_hold(
  const SolverFallbackSteeringHoldRequest & request);

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
