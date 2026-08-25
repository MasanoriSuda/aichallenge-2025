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
/// five-state MPCC solve.  The vectors describe stages 1..N in the same
/// order.  Keeping lag, heading and solved progress here prevents a later
/// consumer from silently rebuilding a different pose sequence from lateral
/// samples alone.
struct ExactPhysicalExecutionTrajectory
{
  double progress_origin_m{std::numeric_limits<double>::quiet_NaN()};
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
};

enum class ExactPhysicalExecutionTrajectoryReason
{
  Accepted,
  TooFewStages,
  InvalidProgressOrigin,
  InvalidProgressRegressionTolerance,
  InvalidMinimumLateralReserve,
  LateralShapeMismatch,
  LagShapeMismatch,
  HeadingShapeMismatch,
  VelocityShapeMismatch,
  ProgressShapeMismatch,
  LowerBoundShapeMismatch,
  UpperBoundShapeMismatch,
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
};

const char * exact_physical_execution_trajectory_reason_name(
  ExactPhysicalExecutionTrajectoryReason reason) noexcept;

ExactPhysicalExecutionTrajectoryValidation
validate_exact_physical_execution_trajectory(
  const ExactPhysicalExecutionTrajectory & trajectory) noexcept;

/// A certificate without every five-state pose/progress field is not exact
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

/// Observe line Recovery with the canonical five-state formulation without
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
/// longitudinal portion of a five-state Follow horizon. All progress values
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

enum class ShadowWarmStartResetReason
{
  None,
  InitialContext,
  InvalidPreviousContext,
  InvalidCurrentContext,
  IntentChanged,
  FormulationChanged,
  HorizonChanged,
  SchemaChanged,
  StageGeometryDiscontinuous,
};

const char * shadow_warm_start_reset_reason_name(
  ShadowWarmStartResetReason reason) noexcept;

struct ShadowWarmStartIdentity
{
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  mpcc_execution_contract::Formulation formulation{
    mpcc_execution_contract::Formulation::Unresolved};
  std::size_t horizon_steps{};
  std::string state_schema_id;
  std::string input_schema_id;
  std::string bounds_schema_id;
  std::string cost_schema_id;
  std::uint64_t stage_geometry_id{};
  int tracking_waypoint{};
  bool circular{false};
  std::vector<mpcc_execution_contract::StageGeometryIdentity> stages;
};

struct ShadowWarmStartResolution
{
  bool valid{false};
  bool apply_warm_start{false};
  bool reset_context{true};
  /// Exact physical stage advance between compatible horizon geometries.
  /// Solver invocation count is deliberately not used as a proxy.
  std::size_t stage_advance{};
  ShadowWarmStartResetReason reason{
    ShadowWarmStartResetReason::InvalidCurrentContext};
};

ShadowWarmStartResolution resolve_shadow_warm_start(
  const std::optional<ShadowWarmStartIdentity> & previous,
  const ShadowWarmStartIdentity & current) noexcept;

}  // namespace multi_purpose_mpc_ros::race_mpcc_foundation

#endif  // MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_
