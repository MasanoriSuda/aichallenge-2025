#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_EXECUTION_ARTIFACT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_EXECUTION_ARTIFACT_HPP_

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact
{

struct Identity
{
  std::uint64_t sequence{};
  /// Exact immutable problem identity consumed by the seven-state solver.
  /// Retained execution keeps this source identity while a separate current
  /// decision certifies the executable suffix against the current world.
  mpcc_execution_contract::MpccProblemContext source_context;
  double snapshot_sec{};
};

/// Return whether the seven-state steering-rate execution artifact owns this
/// canonical normal intent.  All validators and current-world consumers must
/// use this single capability definition; duplicating a Track/Cruise-only
/// subset makes a physically certified Overtake artifact unpublishable.
bool supports_intent(mpcc_execution_contract::ControlIntent intent) noexcept;

/// Resolve whether the current semantic problem can create a seven-state normal
/// request for the selected intent.  Both semantic request assembly and the
/// submission boundary must use this resolver so an intent cannot be admitted
/// by one layer and silently omitted by the next.
bool request_scope_available(
  mpcc_execution_contract::ControlIntent intent,
  bool track_cruise_semantics_available,
  bool follow_semantics_available,
  bool overtake_semantics_available,
  bool rejoin_semantics_available) noexcept;
bool identity_valid(const Identity & identity) noexcept;
bool same_identity(const Identity & lhs, const Identity & rhs) noexcept;

struct PredictedState
{
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
  double velocity_mps{};
  double progress_m{};
  double steering_rad{};
  /// Effective steering state which produces yaw after actuator response.
  /// This is distinct from the serialized command state above.
  double response_steering_rad{};
};

struct ControlStage
{
  double acceleration_mps2{};
  double steering_rate_radps{};
  double virtual_progress_speed_mps{};
  double duration_sec{};
  double virtual_progress_lower_mps{};
  double virtual_progress_upper_mps{};
  /// Original physical acceleration envelope, before the solver-only
  /// certificate inset is applied.
  double acceleration_lower_mps2{};
  double acceleration_upper_mps2{};
  /// Course curvature used by the Frenet dynamics for this stage.
  double path_curvature_radpm{};
};

/// Immutable semantic endpoint which a Return solve must actually reach.
/// This is part of the solved artifact rather than a post-hoc phase hint: a
/// wall-clear trajectory which does not rejoin the requested line is not a
/// certified Return trajectory.
struct TerminalIntentContract
{
  bool active{false};
  double lateral_reference_m{};
  double lateral_tolerance_m{};
  double heading_reference_rad{};
  double heading_tolerance_rad{};
};

/// Proof carried by a short executable prefix that the complete solution from
/// which it was cut reached the semantic endpoint above.  The values are read
/// from the full-horizon primal, not from predicted_states.back(): the latter
/// is only the end of the publisher prefix and is not the Return endpoint.
struct TerminalIntentCertificate
{
  bool active{false};
  std::size_t solved_horizon_steps{};
  double solved_lateral_m{};
  double solved_heading_rad{};
};

/// Immutable executable prefix of one complete, physically row-certified
/// seven-state/three-input solve.  Planning may use a longer horizon than this
/// artifact; only this leading prefix crosses the execution boundary.  This
/// deliberately does not reuse the curvature-input CanonicalExecutionPlan
/// representation.
struct ExecutionArtifact
{
  Identity identity;
  double prediction_origin_sec{};
  /// Time from one command publication to the next.  It certifies that both
  /// physical and desired steering sequences can be sampled over a complete
  /// publisher period; cursor elapsed time is not advanced by this value.
  double publication_interval_sec{};
  double completed_sec{};
  double course_progress_origin_m{};
  /// Physical-equivalent serialized steering command at the
  /// latency-compensated control origin.  The measured/yaw-derived response
  /// state is deliberately not an alternate execution origin.
  double semantic_initial_steering_rad{};
  double semantic_initial_response_steering_rad{};
  double wheelbase_m{};
  double yaw_response_gain{1.0};
  double yaw_response_time_constant_sec{0.13};
  double minimum_frenet_denominator{0.20};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double physical_global_tolerance{};
  double maximum_constraint_violation{};
  double maximum_normalized_constraint_violation{};
  TerminalIntentContract terminal_intent_contract;
  TerminalIntentCertificate terminal_intent_certificate;
  std::vector<PredictedState> predicted_states;
  std::vector<ControlStage> control_stages;
  std::vector<double> nominal_path_distance_m;
  std::vector<double> lateral_lower_m;
  std::vector<double> lateral_upper_m;
};

enum class RejectReason
{
  None,
  InvalidIdentity,
  InvalidTiming,
  InvalidCourseProgressOrigin,
  InvalidLimits,
  InvalidCertificate,
  EmptyHorizon,
  StateCountMismatch,
  PathDistanceCountMismatch,
  InvalidPathDistance,
  CorridorCountMismatch,
  InvalidPredictedState,
  InvalidControlStage,
  InvalidAccelerationControlBounds,
  InvalidProgressControlBounds,
  InvalidLateralCorridor,
  InitialSteeringMismatch,
  SteeringDynamicsMismatch,
  ProgressDynamicsMismatch,
  ProgressRegressionBeyondCertificate,
  SemanticSteeringSequenceRejected,
  InvalidTerminalIntentContract,
  TerminalIntentNotReached,
};

const char * to_string(RejectReason reason) noexcept;

/// Metric tolerance used only for lateral corridor geometry. The global QP
/// tolerance can be dominated by progress or another differently-scaled row
/// and must not become wall clearance. This resolver mirrors the final
/// current-world proof: accept the measured row residual plus a numerical
/// guard, with a small geometry floor.
double physical_lateral_bound_tolerance_m(
  const ExecutionArtifact & artifact) noexcept;

RejectReason validate(const ExecutionArtifact & artifact) noexcept;

enum class CursorReason
{
  Available,
  InvalidArtifact,
  InvalidTime,
  FutureArtifact,
  Exhausted,
};

const char * to_string(CursorReason reason) noexcept;

struct Cursor
{
  bool available{false};
  CursorReason reason{CursorReason::InvalidArtifact};
  std::uint64_t sequence{};
  std::size_t control_stage_index{};
  std::size_t remaining_control_stage_count{};
  double elapsed_sec{};
  double stage_elapsed_sec{};
};

Cursor resolve_cursor(
  const ExecutionArtifact & artifact, double now_sec) noexcept;

enum class ActuationReason
{
  Available,
  InvalidArtifact,
  CursorUnavailable,
  IdentityMismatch,
  InvalidStageIndex,
  SampleRejected,
  NonfiniteActuation,
};

const char * to_string(ActuationReason reason) noexcept;

struct Actuation
{
  std::uint64_t sequence{};
  std::size_t control_stage_index{};
  double predicted_speed_mps{};
  double acceleration_mps2{};
  double steering_rate_radps{};
  double steering_rad{};
  double curvature_radpm{};
  double virtual_progress_speed_mps{};
};

struct ActuationResult
{
  ActuationReason reason{ActuationReason::InvalidArtifact};
  mpcc_rate_resolved::ActuationSampleReason sample_reason{
    mpcc_rate_resolved::ActuationSampleReason::Count};
  std::optional<Actuation> actuation;
};

ActuationResult extract_actuation(
  const ExecutionArtifact & artifact, const Cursor & cursor) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_EXECUTION_ARTIFACT_HPP_
