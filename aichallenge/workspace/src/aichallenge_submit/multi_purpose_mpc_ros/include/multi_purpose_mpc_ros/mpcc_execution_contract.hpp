#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_EXECUTION_CONTRACT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_EXECUTION_CONTRACT_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_execution_contract
{

enum class ControlIntent
{
  Unknown,
  Track,
  Cruise,
  Follow,
  Hold,
  Stop,
  ShiftOut,
  Pass,
  Return,
  Rejoin,
};

enum class Formulation
{
  Unresolved,
  VelocitySteeringProgress6State,
  SolverDerivedBypass,
};

const char * to_string(ControlIntent intent) noexcept;
const char * to_string(Formulation formulation) noexcept;

/// Intents whose normal lateral and longitudinal command may be owned by the
/// canonical six-state MPCC authority. Recovery and emergency overrides are
/// intentionally outside this contract.
bool canonical_normal_intent_supported(ControlIntent intent) noexcept;

enum class AtomicIntentAdmissionReason
{
  ProposedAccepted,
  PreviousRetained,
  NoCurrentWorldAuthority,
  InvalidIntent,
};

const char * to_string(AtomicIntentAdmissionReason reason) noexcept;

struct AtomicIntentAdmissionRequest
{
  ControlIntent proposed_intent{ControlIntent::Unknown};
  ControlIntent previous_published_intent{ControlIntent::Unknown};
  bool proposed_current_world_authority{false};
  bool previous_current_world_authority{false};
};

struct AtomicIntentAdmissionResolution
{
  AtomicIntentAdmissionReason reason{
    AtomicIntentAdmissionReason::InvalidIntent};
  ControlIntent effective_intent{ControlIntent::Unknown};
  bool authority_available{false};
  bool proposal_adopted{false};
  bool previous_retained{false};
};

/// Resolve an intent proposal without creating an authority gap.  The previous
/// intent is eligible only when it is itself a supported six-state intent and
/// has passed current-world proof in this cycle.
AtomicIntentAdmissionResolution resolve_atomic_intent_admission(
  const AtomicIntentAdmissionRequest & request) noexcept;

/// Canonical normal intents whose identity is incomplete without the observed
/// target vehicle and its observation generation.
bool canonical_normal_intent_requires_target(ControlIntent intent) noexcept;

/// Overtake intents whose exact problem identity is incomplete without the
/// selected left/right homotopy.
bool canonical_normal_intent_requires_execution_side(
  ControlIntent intent) noexcept;

/// Classifies whether an asynchronously rebuilt pre-entry trajectory still
/// belongs to the current tactical intent.  A temporarily unavailable
/// tactical selection is deliberately distinct from an explicit opposite-side
/// selection: the former may still undergo observation-only current-world
/// proof, but it is never current tactical authority.
enum class PreentryTacticalIdentityReason
{
  Exact,
  NewerSameSide,
  SelectionUnavailable,
  TargetMismatch,
  MissionGenerationMismatch,
  SideConflict,
  TacticalSequenceRegression,
  InvalidInput,
};

const char * to_string(PreentryTacticalIdentityReason reason) noexcept;

struct PreentryTacticalIdentityRequest
{
  std::string result_target_id;
  int result_side_sign{};
  std::uint64_t result_mission_generation{};
  std::uint64_t result_tactical_sequence{};
  std::string current_target_id;
  bool current_selection_valid{false};
  int current_side_sign{};
  std::uint64_t current_mission_generation{};
  std::uint64_t current_tactical_sequence{};
};

struct PreentryTacticalIdentityResolution
{
  PreentryTacticalIdentityReason reason{
    PreentryTacticalIdentityReason::InvalidInput};
  /// No current target, generation or explicit side contradicts the result.
  /// This is sufficient only to measure current-world proof in shadow.
  bool current_world_observation_permitted{false};
  /// A current same-side tactical selection still endorses this homotopy.
  bool tactical_authority_current{false};
  bool exact{false};
};

PreentryTacticalIdentityResolution resolve_preentry_tactical_identity(
  const PreentryTacticalIdentityRequest & request) noexcept;

/// The sole formulation allowed to own a certified normal publisher command.
/// Exceptional solver-derived bypass paths are intentionally excluded.
bool canonical_normal_formulation_supported(Formulation formulation) noexcept;

struct StageGeometryIdentity
{
  int transition_from_waypoint{};
  int state_waypoint{};
  double transition_distance_m{};
  double cumulative_distance_m{};
};

struct EffectiveStageGeometry
{
  int tracking_waypoint{};
  bool circular{false};
  std::vector<StageGeometryIdentity> stages;
  std::uint64_t fingerprint{};
};

std::uint64_t fingerprint_stage_geometry(
  int tracking_waypoint, bool circular,
  const std::vector<StageGeometryIdentity> & stages) noexcept;

std::optional<EffectiveStageGeometry> resolve_effective_stage_geometry(
  int tracking_waypoint, bool circular,
  const std::vector<StageGeometryIdentity> & raw_stages,
  const std::vector<double> & effective_transition_distances_m) noexcept;

enum class PhysicalWallCertificateReason
{
  NotEvaluated,
  Accepted,
  InvalidInput,
  LateralBoundViolation,
  HeadingUnavailable,
  WallSampleUnavailable,
  HardWallContact,
  CurrentPoseWallSampleUnavailable,
  CurrentPoseHardWallContact,
  CourseFrameUnavailable,
  SweptPathViolation,
};

const char * physical_wall_certificate_reason_name(
  PhysicalWallCertificateReason reason) noexcept;

enum class PhysicalWallPathFailureOrigin
{
  Invalid,
  CurrentPose,
  HorizonStage,
};

struct PhysicalWallPathFailureLocation
{
  PhysicalWallPathFailureOrigin origin{PhysicalWallPathFailureOrigin::Invalid};
  int stage_index{-1};
};

PhysicalWallPathFailureLocation resolve_swept_path_failure_origin(
  std::size_t rejected_path_index, std::size_t horizon_steps) noexcept;

struct PlanarPose
{
  double x_m{};
  double y_m{};
  double yaw_rad{};
};

/// Complete Frenet pose relative to one course-frame pose.  `lag_m` is the
/// signed along-track displacement from the virtual progress frame; it must
/// not be silently replaced by zero when constructing a canonical MPCC x0.
struct FrenetPose
{
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
};

std::optional<FrenetPose> project_planar_pose_to_frenet(
  const PlanarPose & pose, const PlanarPose & course_frame) noexcept;

std::optional<PlanarPose> reconstruct_planar_pose_from_frenet(
  const PlanarPose & course_frame, const FrenetPose & state) noexcept;

struct FirstStageKinematicRequest
{
  PlanarPose initial_pose;
  double initial_speed_mps{};
  double acceleration_mps2{};
  double curvature_radpm{};
  double stage_dt_sec{};
  double elapsed_sec{};
};

struct FirstStageKinematicResult
{
  PlanarPose pose;
  double travel_distance_m{};
  double active_motion_sec{};
};

/// Integrate the executable first MPCC input from the measured world pose.
/// Forward speed is not allowed to become negative during a braking stage.
std::optional<FirstStageKinematicResult> integrate_first_stage_constant_curvature(
  const FirstStageKinematicRequest & request) noexcept;

struct PhysicalWallCertificateDiagnostic
{
  PhysicalWallCertificateReason reason{
    PhysicalWallCertificateReason::NotEvaluated};
  int stage_index{-1};
  int waypoint_id{-1};
  double path_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double lag_m{std::numeric_limits<double>::quiet_NaN()};
  double lower_bound_m{std::numeric_limits<double>::quiet_NaN()};
  double upper_bound_m{std::numeric_limits<double>::quiet_NaN()};
  double bound_reserve_m{std::numeric_limits<double>::quiet_NaN()};
  double heading_offset_rad{std::numeric_limits<double>::quiet_NaN()};
  double reference_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double solved_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double progress_delta_m{std::numeric_limits<double>::quiet_NaN()};
  double pose_x_m{std::numeric_limits<double>::quiet_NaN()};
  double pose_y_m{std::numeric_limits<double>::quiet_NaN()};
  double pose_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  bool out_of_map{false};
  std::size_t contact_cell_count{};
  std::size_t swept_rejected_path_index{std::numeric_limits<std::size_t>::max()};
  std::size_t swept_checked_pose_count{};
  std::size_t swept_rejected_substep{};
  std::size_t swept_rejected_subdivision_count{};
  double swept_rejected_segment_ratio{
    std::numeric_limits<double>::quiet_NaN()};
};

std::string format_physical_wall_certificate_diagnostic(
  const PhysicalWallCertificateDiagnostic & diagnostic);

struct MpccProblemContext
{
  std::uint64_t decision_id{};
  ControlIntent intent{ControlIntent::Unknown};
  std::uint64_t intent_generation{};
  std::uint64_t observation_generation{};
  std::uint64_t stage_geometry_id{};
  std::uint64_t target_obstacle_generation{};
  std::string target_id;
  int execution_side_sign{};
  std::size_t horizon_steps{};
  Formulation formulation{Formulation::Unresolved};
  std::string state_schema_id;
  std::string input_schema_id;
  std::string bounds_schema_id;
  std::string cost_schema_id;
  std::uint64_t fingerprint{};
};

std::uint64_t problem_context_fingerprint(
  const MpccProblemContext & context) noexcept;
MpccProblemContext seal_problem_context(MpccProblemContext context) noexcept;
bool problem_context_complete(const MpccProblemContext & context) noexcept;

struct PhysicalCertificate
{
  bool checked{false};
  bool wall_clear{false};
  bool obstacles_clear{false};
  double minimum_wall_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  double minimum_obstacle_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
};

struct CertifiedMpccSolution
{
  std::uint64_t solution_id{};
  std::uint64_t problem_fingerprint{};
  Formulation formulation{Formulation::Unresolved};
  bool solved{false};
  bool finite{false};
  bool constraints_satisfied{false};
  double maximum_constraint_violation{
    std::numeric_limits<double>::quiet_NaN()};
  PhysicalCertificate physical;
  std::size_t prediction_stage_count{};
  double valid_until_sec{-std::numeric_limits<double>::infinity()};
};

bool solution_certified(const CertifiedMpccSolution & solution) noexcept;

/// Metadata for one canonical normal-authority candidate.  The executable
/// stage count is deliberately separate from CertifiedMpccSolution: a solver
/// certificate without a retained command sequence cannot own control.
/// `execution_certificate_decision_id` is also deliberately independent from
/// the solver problem decision: a retained plan keeps its original solve
/// identity, but its remaining prefix must be revalidated from the current
/// measured pose before it can continue to own control.
struct CanonicalNormalCandidate
{
  std::optional<MpccProblemContext> problem;
  std::optional<CertifiedMpccSolution> solution;
  std::size_t executable_control_stage_count{};
  std::uint64_t execution_plan_id{};
  std::uint64_t execution_certificate_decision_id{};
  std::size_t execution_first_control_stage_index{};
  PhysicalCertificate execution_physical;
};

enum class CanonicalNormalCandidateRejectReason
{
  NotEvaluated,
  None,
  MissingIdentity,
  IncompleteProblem,
  UnsupportedIntent,
  NoncanonicalFormulation,
  IdentityMismatch,
  NotCertified,
  MissingExecutionPlan,
  NoExecutableControl,
  InvalidExecutableHorizon,
  Expired,
  DecisionMismatch,
  ExecutionCertificateDecisionMismatch,
  ExecutionCertificateNotCertified,
  IntentMismatch,
};

const char * to_string(CanonicalNormalCandidateRejectReason reason) noexcept;

enum class CanonicalNormalAuthoritySource
{
  FreshCertified,
  RetainedCertified,
  EmergencyStop,
};

const char * to_string(CanonicalNormalAuthoritySource source) noexcept;

enum class CanonicalNormalAuthorityReason
{
  FreshCertified,
  RetainedCertified,
  NoCanonicalCandidate,
  InvalidRequest,
};

const char * to_string(CanonicalNormalAuthorityReason reason) noexcept;

struct CanonicalNormalAuthorityRequest
{
  std::uint64_t current_decision_id{};
  double now_sec{std::numeric_limits<double>::quiet_NaN()};
  CanonicalNormalCandidate fresh;
  CanonicalNormalCandidate retained;
  /// Current supervisor intent is explicit: a retained problem cannot infer
  /// or carry intent authority into a later control decision.
  ControlIntent current_intent{ControlIntent::Unknown};
};

struct CanonicalNormalAuthorityResolution
{
  CanonicalNormalAuthoritySource source{
    CanonicalNormalAuthoritySource::EmergencyStop};
  CanonicalNormalAuthorityReason reason{
    CanonicalNormalAuthorityReason::NoCanonicalCandidate};
  CanonicalNormalCandidateRejectReason fresh_reject_reason{
    CanonicalNormalCandidateRejectReason::NotEvaluated};
  CanonicalNormalCandidateRejectReason retained_reject_reason{
    CanonicalNormalCandidateRejectReason::NotEvaluated};
  std::optional<MpccProblemContext> problem;
  std::optional<CertifiedMpccSolution> solution;
  std::size_t executable_control_stage_count{};
  std::uint64_t execution_plan_id{};
  std::uint64_t execution_certificate_decision_id{};
  std::size_t execution_first_control_stage_index{};
  bool retained_solution{false};
};

/// Resolve Track/Cruise normal authority without any legacy candidate.
/// Failure is explicit EmergencyStop; Recovery remains a supervisor override
/// outside this normal-authority contract.
CanonicalNormalAuthorityResolution resolve_canonical_normal_authority(
  const CanonicalNormalAuthorityRequest & request) noexcept;

/// The complete first executable actuation from one canonical solution.
/// These fields must remain distinct through the publisher
/// boundary; in particular, predicted speed is not an acceleration command.
struct CanonicalActuation
{
  double predicted_speed_mps{};
  double acceleration_mps2{};
  double curvature_radpm{};
  double steering_tire_angle_rad{};
  double virtual_progress_speed_mps{};
};

struct CanonicalNormalCommand
{
  std::uint64_t decision_id{};
  std::uint64_t execution_plan_id{};
  std::uint64_t execution_certificate_decision_id{};
  std::uint64_t problem_fingerprint{};
  std::uint64_t solution_id{};
  CanonicalNormalAuthoritySource source{
    CanonicalNormalAuthoritySource::EmergencyStop};
  ControlIntent intent{ControlIntent::Unknown};
  Formulation formulation{Formulation::Unresolved};
  bool retained_solution{false};
  double predicted_speed_mps{};
  double acceleration_mps2{};
  double curvature_radpm{};
  double steering_tire_angle_rad{};
  double virtual_progress_speed_mps{};
};

enum class CanonicalNormalCommandReason
{
  Available,
  EmergencyAuthority,
  IncompleteAuthorityIdentity,
  InvalidActuation,
};

const char * to_string(CanonicalNormalCommandReason reason) noexcept;

struct CanonicalNormalCommandResult
{
  CanonicalNormalCommandReason reason{
    CanonicalNormalCommandReason::IncompleteAuthorityIdentity};
  std::optional<CanonicalNormalCommand> command;
};

CanonicalNormalCommandResult build_canonical_normal_command(
  const CanonicalNormalAuthorityResolution & authority,
  const CanonicalActuation & actuation) noexcept;

/// Verify the model command immediately before publication.  A canonical
/// command is already expressed as the physical tire angle used by its model
/// and certificate, so later actuator gain is forbidden.
bool canonical_normal_command_matches_actuation(
  const CanonicalNormalCommand & command, double target_speed_mps,
  double acceleration_mps2, double steering_tire_angle_rad) noexcept;

/// Verify the command after serialization into the ROS control message.  The
/// Ackermann message stores its scalar actuation fields as float32, so exact
/// double comparison here would reject the very command that crossed the
/// publisher boundary.  This remains an exact comparison in the wire type; it
/// is not a numeric tolerance or a second actuation policy.
bool canonical_normal_command_matches_serialized_actuation(
  const CanonicalNormalCommand & command, double target_speed_mps,
  double acceleration_mps2, double steering_tire_angle_rad) noexcept;

/// Resolve the tire angle serialized to the vehicle interface.  Legacy normal
/// paths retain their calibrated raw-command convention during migration;
/// canonical normal authority must publish the certified physical angle
/// exactly.
std::optional<double> resolve_published_steering_tire_angle(
  double model_steering_tire_angle_rad, double legacy_actuator_gain,
  bool canonical_normal_authority) noexcept;

/// Select the physical steering publication convention for every normal
/// command owned by the canonical controller. Recovery is a
/// distinct supervisor and retains its existing actuator convention.
bool canonical_normal_uses_physical_steering(
  bool canonical_normal_authority, bool canonical_emergency_stop,
  bool recovery_override) noexcept;

enum class FinalAuthorityClass
{
  CertifiedNormalSolution,
  EmergencyOverride,
  RecoveryOverride,
  ControlDisabled,
};

const char * to_string(FinalAuthorityClass authority) noexcept;

struct FinalAuthorityClassRequest
{
  bool recovery_override{false};
  bool control_enabled{true};
  bool canonical_normal_source{false};
  bool certified_solution_available{false};
  bool canonical_normal_command_available{false};
};

/// Classify the only four final control authorities. A non-Recovery,
/// control-enabled command is normal only when its source, certified solution
/// and typed canonical command are all present; every other such output is an
/// explicit Emergency override.
FinalAuthorityClass resolve_final_authority_class(
  const FinalAuthorityClassRequest & request) noexcept;

struct FinalControlDecisionRequest
{
  std::uint64_t decision_id{};
  FinalAuthorityClass authority{FinalAuthorityClass::ControlDisabled};
  std::string source;
  std::optional<MpccProblemContext> problem;
  std::optional<CertifiedMpccSolution> solution;
  bool retained_solution{false};
  std::optional<CanonicalNormalCommand> canonical_normal_command;
  ControlIntent supervisor_intent{ControlIntent::Unknown};

  FinalControlDecisionRequest() = default;

  FinalControlDecisionRequest(
    const std::uint64_t decision_id_in,
    const FinalAuthorityClass authority_in,
    const std::string & source_in,
    const std::optional<MpccProblemContext> & problem_in = std::nullopt,
    const std::optional<CertifiedMpccSolution> & solution_in = std::nullopt,
    const bool retained_solution_in = false,
    const std::optional<CanonicalNormalCommand> & canonical_normal_command_in =
    std::nullopt,
    const ControlIntent supervisor_intent_in = ControlIntent::Unknown)
  : decision_id(decision_id_in),
    authority(authority_in),
    source(source_in),
    problem(problem_in),
    solution(solution_in),
    retained_solution(retained_solution_in),
    canonical_normal_command(canonical_normal_command_in),
    supervisor_intent(supervisor_intent_in)
  {
  }
};

struct FinalControlDecision
{
  std::uint64_t decision_id{};
  FinalAuthorityClass authority{FinalAuthorityClass::ControlDisabled};
  std::string source;
  ControlIntent intent{ControlIntent::Unknown};
  Formulation formulation{Formulation::Unresolved};
  std::uint64_t problem_fingerprint{};
  std::uint64_t solution_id{};
  std::uint64_t execution_plan_id{};
  std::uint64_t execution_certificate_decision_id{};
  CanonicalNormalAuthoritySource canonical_source{
    CanonicalNormalAuthoritySource::EmergencyStop};
  bool retained_solution{false};
  bool identity_complete{false};
  bool canonical_contract_satisfied{false};
  std::string reason{"not-resolved"};
};

FinalControlDecision resolve_final_control_decision(
  const FinalControlDecisionRequest & request) noexcept;
std::string format_final_control_decision(
  const FinalControlDecision & decision);

}  // namespace multi_purpose_mpc_ros::mpcc_execution_contract

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_EXECUTION_CONTRACT_HPP_
