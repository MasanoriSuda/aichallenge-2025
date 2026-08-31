#ifndef MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_

#include <multi_purpose_mpc_ros/mpcc_execution_contract.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::race_mpcc_foundation
{

enum class Homotopy
{
  Left,
  Right,
  Hold,
  Return,
  None,
};

const char * homotopy_name(Homotopy homotopy) noexcept;

enum class TargetProvenanceStage
{
  None,
  Observed,
  Locked,
};

const char * target_provenance_stage_name(
  TargetProvenanceStage stage) noexcept;

struct TargetProvenance
{
  bool valid{false};
  std::string target_id;
  double source_stamp_sec{-std::numeric_limits<double>::infinity()};
  double receipt_sec{-std::numeric_limits<double>::infinity()};
  double course_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double course_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t observation_generation{0U};
  TargetProvenanceStage stage{TargetProvenanceStage::None};
};

enum class TargetProvenanceRejectReason
{
  None,
  InvalidExpected,
  InvalidCurrent,
  TargetMismatch,
  SourceRegression,
  ReceiptRegression,
  GenerationRegression,
  StageRegression,
  ProgressDelta,
  LateralDelta,
};

const char * target_provenance_reject_reason_name(
  TargetProvenanceRejectReason reason) noexcept;

struct TargetProvenanceValidationRequest
{
  TargetProvenance expected;
  TargetProvenance current;
  bool circular{false};
  double path_length_m{};
  double maximum_backward_progress_m{};
  double maximum_forward_progress_m{};
  double maximum_lateral_change_m{};
};

struct TargetProvenanceValidation
{
  bool valid{false};
  bool same_observation{false};
  double progress_delta_m{std::numeric_limits<double>::quiet_NaN()};
  double lateral_delta_m{std::numeric_limits<double>::quiet_NaN()};
  TargetProvenanceRejectReason reject_reason{
    TargetProvenanceRejectReason::InvalidExpected};
};

TargetProvenanceValidation validate_target_provenance(
  const TargetProvenanceValidationRequest & request) noexcept;

/// Immutable pose-state artifact used by a physical wall certificate for one
/// canonical MPCC solve.  The vectors describe stages 1..N in the same
/// order.  Keeping lag, heading and solved progress here prevents a later
/// consumer from silently rebuilding a different pose sequence from lateral
/// samples alone.
struct ExactPhysicalExecutionTrajectory
{
  double progress_origin_m{std::numeric_limits<double>::quiet_NaN()};
  /// Sample time from the common control/prediction origin.  Retained and
  /// dynamic-obstacle consumers must use this same clock rather than
  /// reconstructing stage timing from the affine QP states.
  std::vector<double> elapsed_time_sec;
  std::vector<double> path_distance_m;
  std::vector<double> lateral_m;
  std::vector<double> lag_m;
  std::vector<double> heading_offset_rad;
  std::vector<double> velocity_mps;
  std::vector<double> progress_m;
  std::vector<double> lateral_lower_m;
  std::vector<double> lateral_upper_m;
  double minimum_lateral_bound_reserve_m{
    std::numeric_limits<double>::quiet_NaN()};
  /// Maximum solver-certified backwards progress admitted between adjacent
  /// raw states. The world-pose samples are never clamped or rewritten.
  double progress_regression_tolerance_m{};
  /// Maximum solver-certified residual below the semantic zero-velocity
  /// bound. Raw solved states remain unchanged for physical proof.
  double velocity_lower_bound_tolerance_mps{};
  /// A maximum-braking Stop remains a temporal trajectory after the vehicle
  /// reaches zero speed: actuator response and the publisher clock still
  /// advance while physical path distance remains constant.  Normal MPCC
  /// trajectories leave this false and retain a strictly increasing distance
  /// certificate.
  bool stationary_path_suffix_allowed{false};
  double stationary_velocity_tolerance_mps{};
  /// Solver-certified physical tolerance for the lateral corridor rows.
  /// The nonlinear replay remains the wall-proof trajectory; this tolerance
  /// only prevents an already accepted numerical row residual from being
  /// reinterpreted as a new hard failure by a downstream strict comparison.
  double lateral_bound_tolerance_m{};
};

enum class ExactPhysicalExecutionTrajectoryReason
{
  Accepted,
  TooFewStages,
  InvalidProgressOrigin,
  InvalidProgressRegressionTolerance,
  InvalidVelocityLowerBoundTolerance,
  InvalidStationaryVelocityTolerance,
  InvalidLateralBoundTolerance,
  InvalidMinimumLateralReserve,
  TimeShapeMismatch,
  LateralShapeMismatch,
  LagShapeMismatch,
  HeadingShapeMismatch,
  VelocityShapeMismatch,
  ProgressShapeMismatch,
  LowerBoundShapeMismatch,
  UpperBoundShapeMismatch,
  InvalidElapsedTime,
  InvalidPathDistance,
  NonFiniteLateral,
  NonFiniteLag,
  NonFiniteHeading,
  InvalidVelocity,
  ProgressRegressed,
  InvalidLateralBounds,
};

struct ExactPhysicalExecutionTrajectoryValidation
{
  bool complete{false};
  ExactPhysicalExecutionTrajectoryReason reason{
    ExactPhysicalExecutionTrajectoryReason::TooFewStages};
  int stage{-1};
  double rejected_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double rejected_lateral_lower_m{std::numeric_limits<double>::quiet_NaN()};
  double rejected_lateral_upper_m{std::numeric_limits<double>::quiet_NaN()};
};

const char * exact_physical_execution_trajectory_reason_name(
  ExactPhysicalExecutionTrajectoryReason reason) noexcept;

ExactPhysicalExecutionTrajectoryValidation
validate_exact_physical_execution_trajectory(
  const ExactPhysicalExecutionTrajectory & trajectory) noexcept;

/// A certificate without every canonical pose/progress field is not exact
/// and must not admit or revalidate an Overtake execution path.
bool exact_physical_execution_trajectory_complete(
  const ExactPhysicalExecutionTrajectory & trajectory) noexcept;

struct ShadowCandidate
{
  Homotopy homotopy{Homotopy::None};
  bool attempted{false};
  bool feasible{false};
  bool warm_start_applied{false};
  bool solver_context_reset{false};
  std::uint64_t solver_context_solve_count{};
  double objective{std::numeric_limits<double>::infinity()};
  double terminal_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_wall_reserve_m{std::numeric_limits<double>::quiet_NaN()};
  double solve_ms{};
  int iterations{};
  std::string reason{"not-evaluated"};
};

struct ShadowDecision
{
  std::uint64_t context_epoch{};
  std::string target_id;
  TargetProvenance target_provenance{};
  bool stage_geometry_valid{false};
  std::size_t stage_count{};
  double horizon_distance_m{};
  std::array<ShadowCandidate, 4U> candidates{};
  Homotopy selected{Homotopy::None};
  std::string selection_reason{"not-evaluated"};
};

std::string format_shadow_decision(const ShadowDecision & decision);

enum class TrackCruiseShadowEligibilityReason
{
  Eligible,
  LiveProgressAlreadyActive,
  TacticalSnapshot,
  IntentNotTrackCruise,
};

const char * track_cruise_shadow_eligibility_reason_name(
  TrackCruiseShadowEligibilityReason reason) noexcept;

struct TrackCruiseShadowEligibilityRequest
{
  bool live_progress_active{false};
  bool tactical_snapshot{false};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
};

struct TrackCruiseShadowEligibility
{
  bool eligible{false};
  TrackCruiseShadowEligibilityReason reason{
    TrackCruiseShadowEligibilityReason::IntentNotTrackCruise};
};

TrackCruiseShadowEligibility resolve_track_cruise_shadow_eligibility(
  const TrackCruiseShadowEligibilityRequest & request) noexcept;

enum class RejoinShadowEligibilityReason
{
  Eligible,
  LiveProgressAlreadyActive,
  TacticalSnapshot,
  IntentNotRejoin,
};

const char * rejoin_shadow_eligibility_reason_name(
  RejoinShadowEligibilityReason reason) noexcept;

struct RejoinShadowEligibilityRequest
{
  bool live_progress_active{false};
  bool tactical_snapshot{false};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
};

struct RejoinShadowEligibility
{
  bool eligible{false};
  RejoinShadowEligibilityReason reason{
    RejoinShadowEligibilityReason::IntentNotRejoin};
};

/// Observe line Recovery with the canonical six-state formulation without
/// granting it production authority. Rejoin deliberately has no target or
/// pass-side identity; its semantic goal is the base racing line.
RejoinShadowEligibility resolve_rejoin_shadow_eligibility(
  const RejoinShadowEligibilityRequest & request) noexcept;

enum class FollowShadowEligibilityReason
{
  Eligible,
  LiveProgressAlreadyActive,
  TacticalSnapshot,
  IntentNotFollow,
  NoCoherentFrontObservation,
};

const char * follow_shadow_eligibility_reason_name(
  FollowShadowEligibilityReason reason) noexcept;

struct FollowShadowEligibilityRequest
{
  bool live_progress_active{false};
  bool tactical_snapshot{false};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  /// True only when the current behavior cycle selected a physically front
  /// vehicle and its finite longitudinal observation belongs to the same
  /// target provenance. A retained Follow label alone is not sufficient.
  bool coherent_front_observation{false};
};

struct FollowShadowEligibility
{
  bool eligible{false};
  FollowShadowEligibilityReason reason{
    FollowShadowEligibilityReason::IntentNotFollow};
};

/// Request canonical Follow metadata only for a coherent current-world front
/// observation. Production authority is resolved separately from this gate.
FollowShadowEligibility resolve_follow_shadow_eligibility(
  const FollowShadowEligibilityRequest & request) noexcept;

enum class OvertakeCanonicalFreshShadowEligibilityReason
{
  Eligible,
  ProgressContouringInactive,
  IntentNotOvertakeExecution,
  ExecutionContextUnavailable,
  LateralBoundsInvalid,
};

const char * overtake_canonical_fresh_shadow_eligibility_reason_name(
  OvertakeCanonicalFreshShadowEligibilityReason reason) noexcept;

struct OvertakeCanonicalFreshShadowEligibilityRequest
{
  bool progress_contouring_active{false};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  bool execution_context_available{false};
  bool lateral_bounds_valid{false};
};

struct OvertakeCanonicalFreshShadowEligibility
{
  bool eligible{false};
  OvertakeCanonicalFreshShadowEligibilityReason reason{
    OvertakeCanonicalFreshShadowEligibilityReason::ProgressContouringInactive};
};

/// Gate the exact fresh-chain observation to the live overtake execution
/// intents. This is telemetry-only and grants no production authority.
OvertakeCanonicalFreshShadowEligibility
resolve_overtake_canonical_fresh_shadow_eligibility(
  const OvertakeCanonicalFreshShadowEligibilityRequest & request) noexcept;

enum class ReturnTransitionAdmissionReason
{
  Inactive,
  GeometricPreflightUnavailable,
  ProposalIncomplete,
  IntentMismatch,
  TargetMismatch,
  MissionGenerationMismatch,
  SideMismatch,
  CurrentWorldRejected,
  Admitted,
};

const char * return_transition_admission_reason_name(
  ReturnTransitionAdmissionReason reason) noexcept;

struct ReturnTransitionAdmissionRequest
{
  bool pass_active{false};
  bool geometric_preflight_valid{false};
  bool proposal_complete{false};
  bool current_world_certified{false};
  mpcc_execution_contract::ControlIntent proposal_intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  std::string current_target_id;
  std::string proposal_target_id;
  std::uint64_t current_mission_generation{};
  std::uint64_t proposal_mission_generation{};
  int current_side_sign{};
  int proposal_side_sign{};
};

struct ReturnTransitionAdmission
{
  bool admitted{false};
  ReturnTransitionAdmissionReason reason{
    ReturnTransitionAdmissionReason::Inactive};
};

/// A geometric rejoin path is necessary but cannot transfer normal authority.
/// Pass may become Return only when the same current-world certified
/// seven-state artifact carries the exact encounter identity consumed by the
/// canonical publisher.
ReturnTransitionAdmission resolve_return_transition_admission(
  const ReturnTransitionAdmissionRequest & request) noexcept;

enum class PassTransitionAdmissionReason
{
  Inactive,
  DynamicHorizonUnavailable,
  PhysicalHorizonUnavailable,
  ProposalIncomplete,
  IntentMismatch,
  TargetMismatch,
  MissionGenerationMismatch,
  SideMismatch,
  CurrentWorldRejected,
  Admitted,
};

const char * pass_transition_admission_reason_name(
  PassTransitionAdmissionReason reason) noexcept;

struct PassTransitionBoundaryState
{
  bool observed{false};
  std::string target_id;
  std::uint64_t mission_generation{};
  int side_sign{};
};

struct PassTransitionBoundaryRequest
{
  bool shiftout_active{false};
  bool completion_observed{false};
  std::string target_id;
  std::uint64_t mission_generation{};
  int side_sign{};
};

struct PassTransitionBoundaryResolution
{
  PassTransitionBoundaryState state;
  bool ready{false};
};

/// Rendezvous state for a physical ShiftOut boundary and its asynchronous
/// current-world Pass successor. It retains only the identity-scoped boundary
/// fact; no path, certificate, timeout or execution authority is retained.
PassTransitionBoundaryResolution resolve_pass_transition_boundary(
  const PassTransitionBoundaryState & previous,
  const PassTransitionBoundaryRequest & request);

struct PassTransitionAdmissionRequest
{
  bool shiftout_boundary_ready{false};
  bool dynamic_horizon_available{false};
  bool physical_horizon_available{false};
  bool proposal_complete{false};
  bool current_world_certified{false};
  mpcc_execution_contract::ControlIntent proposal_intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  std::string current_target_id;
  std::string proposal_target_id;
  std::uint64_t current_mission_generation{};
  std::uint64_t proposal_mission_generation{};
  int current_side_sign{};
  int proposal_side_sign{};
};

struct PassTransitionAdmission
{
  bool admitted{false};
  PassTransitionAdmissionReason reason{
    PassTransitionAdmissionReason::Inactive};
};

/// Tactical horizons are necessary but cannot transfer normal authority.
/// ShiftOut may become Pass only when a matching current-world certified
/// seven-state successor already exists.
PassTransitionAdmission resolve_pass_transition_admission(
  const PassTransitionAdmissionRequest & request) noexcept;

enum class FollowProductionAction
{
  NotOwned,
  PublishCanonical,
  SolveTransitionAdmission,
  EmergencyStop,
};

/// Follow is an exclusive normal-authority boundary after promotion. A
/// requested transition may synchronously complete Gate A with the same
/// canonical producer. Once Follow has published, a missing same-intent
/// selection fails closed and can never borrow another normal formulation.
FollowProductionAction resolve_follow_production_action(
  mpcc_execution_contract::ControlIntent intent,
  bool complete_canonical_selection,
  mpcc_execution_contract::ControlIntent last_published_canonical_intent)
noexcept;

enum class StopAuthorityAction
{
  NotOwned,
  EmergencyStop,
};

/// The currently proven Stop producer is SafetyBrake, an emergency supervisor
/// action. It must terminate normal routing instead of borrowing a normal
/// solver for lateral control before braking is applied downstream.
StopAuthorityAction resolve_stop_authority_action(
  mpcc_execution_contract::ControlIntent intent) noexcept;

enum class StopLateralAction
{
  HoldAtRest,
  TrackReferencePath,
  Neutralize,
};

struct StopLateralActionRequest
{
  double current_speed_mps{};
  bool reference_path_target_available{false};
};

/// Emergency Stop owns the complete wire command. While the vehicle is still
/// moving, a stale constant steering command is not a valid lateral policy:
/// braking from race speed still traverses several metres. Prefer the current
/// base-path feedback target and otherwise converge toward neutral. Holding is
/// reserved for standstill, where changing steering only creates chatter.
StopLateralAction resolve_stop_lateral_action(
  const StopLateralActionRequest & request) noexcept;

const char * stop_lateral_action_name(StopLateralAction action) noexcept;

/// One immutable moving-Stop lateral policy shared by the retained terminal
/// certificate and the emergency publisher.  Values are expressed in the raw
/// controller steering coordinates used by the seven-state model; the final
/// wire gain is applied only at publication.
struct StopPathTrackingPolicy
{
  double wheelbase_m{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double maximum_lateral_acceleration_mps2{};
  double steering_command_gain{};
  double lateral_gain{};
  double heading_gain{};
};

struct StopPathTrackingCommandRequest
{
  StopPathTrackingPolicy policy;
  double current_lateral_m{};
  double current_heading_error_rad{};
  double reference_curvature_radpm{};
  double current_speed_mps{};
  double current_steering_rad{};
  double step_sec{};
  /// Course-frame lateral reference owned by this immutable Stop policy.
  /// Ordinary Emergency Stop targets the racing line (zero). Architecture
  /// audits may provide a candidate-declared offset without creating a second
  /// feedback law.
  double target_lateral_m{};
};

struct StopPathTrackingCommand
{
  double unconstrained_target_steering_rad{};
  double target_steering_rad{};
  double steering_rad{};
  double steering_rate_radps{};
};

/// Resolve the exact rate-limited path-feedback command used while a moving
/// vehicle executes Emergency Stop. Returning nullopt is a policy/input
/// contract failure; callers must not substitute a different lateral law and
/// still claim the same terminal certificate.
std::optional<StopPathTrackingCommand> resolve_stop_path_tracking_command(
  const StopPathTrackingCommandRequest & request) noexcept;

enum class StopShadowIntentReason
{
  NotStop,
  InterruptedOvertake,
  InterruptedRejoin,
  CoherentFront,
  RaceCruise,
  PreRaceTrack,
};

const char * stop_shadow_intent_reason_name(
  StopShadowIntentReason reason) noexcept;

struct StopShadowIntentRequest
{
  mpcc_execution_contract::ControlIntent active_intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  mpcc_execution_contract::ControlIntent last_published_normal_intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  bool interrupted_overtake_context_available{false};
  bool interrupted_rejoin_context_available{false};
  bool coherent_front_observation{false};
  bool race_session_active{false};
};

struct StopShadowIntentResolution
{
  bool requested{false};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  StopShadowIntentReason reason{StopShadowIntentReason::NotStop};
};

/// SafetyBrake remains the only Stop command owner. While it owns the wire,
/// this resolver chooses the normal intent whose *shadow successor* must stay
/// warm from the actually published braking command. The result never grants
/// normal production authority during Stop.
StopShadowIntentResolution resolve_stop_shadow_intent(
  const StopShadowIntentRequest & request) noexcept;

enum class FollowLongitudinalContractReason
{
  Accepted,
  IntentNotFollow,
  InvalidTargetIdentity,
  InvalidTargetObservation,
  StaleTargetObservation,
  InvalidTargetKinematics,
  InvalidProgressOrigin,
  InvalidConfiguration,
  InvalidHorizon,
  InitialHardGapViolation,
};

const char * follow_longitudinal_contract_reason_name(
  FollowLongitudinalContractReason reason) noexcept;

/// Inputs required to turn one fresh front-target observation into the
/// longitudinal portion of a canonical Follow horizon. All progress values
/// are relative to the current MPCC progress origin.
struct FollowLongitudinalContractRequest
{
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  std::string target_id;
  std::uint64_t target_observation_generation{};
  double target_observation_age_sec{std::numeric_limits<double>::infinity()};
  double maximum_target_observation_age_sec{};
  double current_target_relative_progress_m{
    std::numeric_limits<double>::quiet_NaN()};
  /// Current ego progress relative to the MPCC progress origin. Target
  /// progress is expressed in that same frame as this offset plus the
  /// ego-relative target distance.
  double current_ego_progress_offset_m{
    std::numeric_limits<double>::quiet_NaN()};
  double current_ego_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double target_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double moving_target_speed_threshold_mps{};
  double desired_gap_m{};
  double hard_gap_m{};
  double maximum_closing_speed_mps{};
  double maximum_recovery_speed_mps{};
  double distance_gain_per_sec{};
  double slow_target_velocity_cap_mps{};
  double braking_deceleration_mps2{};
  double maximum_velocity_mps{};
  std::vector<double> stage_dt_sec;
  std::vector<double> base_progress_reference_m;
  std::vector<double> base_progress_upper_m;
  std::vector<double> base_velocity_reference_mps;
  std::vector<double> base_velocity_upper_mps;
};

struct FollowLongitudinalContract
{
  bool valid{false};
  FollowLongitudinalContractReason reason{
    FollowLongitudinalContractReason::InvalidHorizon};
  std::string target_id;
  std::uint64_t target_observation_generation{};
  double current_target_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double current_ego_progress_offset_m{
    std::numeric_limits<double>::quiet_NaN()};
  double target_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double planning_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double hard_gap_m{std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> elapsed_time_sec;
  std::vector<double> target_progress_m;
  std::vector<double> progress_reference_m;
  std::vector<double> progress_lower_m;
  std::vector<double> progress_upper_m;
  std::vector<double> velocity_reference_mps;
  std::vector<double> velocity_upper_mps;
};

/// Build a fail-closed stage-wise Follow contract. This function has no
/// authority side effects and never repairs an already violated current hard
/// gap. A stopped target is approached using the configured ego braking limit;
/// a moving target uses the existing signed distance-gain policy.
FollowLongitudinalContract build_follow_longitudinal_contract(
  const FollowLongitudinalContractRequest & request) noexcept;

/// Physical longitudinal certificate for the extended Frenet state. The
/// vehicle's along-track position is theta + e_lag, not theta alone.
struct FollowEffectiveGapCertificate
{
  bool valid{false};
  bool satisfied{false};
  int worst_stage{-1};
  double minimum_gap_m{std::numeric_limits<double>::infinity()};
  double maximum_violation_m{};
};

FollowEffectiveGapCertificate evaluate_follow_effective_gap(
  const std::vector<double> & target_progress_m,
  const std::vector<double> & solved_progress_m,
  const std::vector<double> & solved_lag_m,
  double hard_gap_m, double tolerance_m) noexcept;

}  // namespace multi_purpose_mpc_ros::race_mpcc_foundation

#endif  // MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_
