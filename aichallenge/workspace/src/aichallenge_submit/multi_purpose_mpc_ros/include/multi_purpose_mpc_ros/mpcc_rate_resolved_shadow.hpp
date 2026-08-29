#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_SHADOW_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_SHADOW_HPP_

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_obstacle.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_wall_refinement.hpp"
#include "multi_purpose_mpc_ros/mpcc_progress.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
using Identity = artifact::Identity;

/// Owned current-world vehicle observation used only to reproduce an
/// architecture comparison.  It carries no candidate, certificate or command
/// authority.  The radius is the exact peer-only physical radius resolved by
/// the live controller for this observation epoch.
struct ReplayDynamicObstacle
{
  std::string id;
  double x_m{};
  double y_m{};
  double velocity_x_mps{};
  double velocity_y_mps{};
  double acceleration_x_mps2{};
  double acceleration_y_mps2{};
  double covariance_x_m2{};
  double covariance_y_m2{};
  double radius_m{};
  std::uint64_t observation_generation{};
};

/// Exact current-world inputs which are outside the convex QP but required to
/// rebuild alternative candidates and rerun final physical proof.  This data
/// is captured from the same callback epoch as Snapshot::request.
struct ReplayWorld
{
  std::uint64_t observation_generation{};
  double observed_sec{};
  bool current{false};
  recovery_footprint::Pose2D current_pose;
  std::vector<recovery_footprint::Pose2D> control_prefix;
  /// Exact age of every measured-to-control pose from observed_sec.  Dynamic
  /// obstacle replay must not infer timing from spatial samples.
  std::vector<double> control_prefix_elapsed_sec;
  /// Raw ego body used by the exact physical proof before hard wall
  /// clearance is added.  Snapshot::wall_footprint is the already expanded
  /// QP refinement footprint and has a different meaning.
  recovery_footprint::FootprintExtents physical_footprint;
  std::uint64_t wall_grid_fingerprint{};
  double hard_wall_clearance_m{};
  double bound_tolerance_m{};
  double swept_step_m{};
  std::vector<ReplayDynamicObstacle> obstacles;
};

struct Snapshot
{
  Identity identity;
  double control_prediction_origin_sec{};
  mpcc_rate_resolved_adapter::Request request;
  /// Leading solved stages which may cross the execution boundary.  The
  /// request still carries the complete planning horizon: shortening the QP
  /// itself makes a distant tactical terminal target numerically abrupt.
  int execution_prefix_steps{};
  double course_progress_origin_m{};
  std::vector<double> nominal_path_distance_m;
  bool progress_aligned_wall_refinement_active{false};
  std::vector<double> wall_reference_progress_m;
  std::vector<double> wall_lower_m;
  std::vector<double> wall_upper_m;
  std::string progress_wall_profile_diagnostic{"not-provided"};
  bool dynamic_obstacle_refinement_active{false};
  int dynamic_obstacle_pass_side_sign{0};
  /// Optional architecture-candidate lattice transition.  Live production
  /// snapshots leave this absent (`-1`) and retain witness-based refinement.
  int dynamic_obstacle_forced_first_pass_side_stage{-1};
  int dynamic_obstacle_forced_first_ahead_stage{-1};
  double dynamic_obstacle_forced_constraint_fraction{1.0};
  int dynamic_obstacle_forced_diagonal_start_stage{-1};
  int dynamic_obstacle_forced_diagonal_full_side_stage{-1};
  /// Candidate-F shadow comparison only.  Geometry is reconstructed from the
  /// immutable ReplayWorld and target identity, never from a retained path.
  bool dynamic_obstacle_forced_physical_diagonal{false};
  std::vector<mpcc_rate_resolved_dynamic_obstacle::StagePrediction>
    dynamic_obstacle_stages;
  bool physical_wall_refinement_active{false};
  std::shared_ptr<const recovery_footprint::OccupancyGrid> wall_grid;
  recovery_footprint::FootprintExtents wall_footprint;
  std::vector<mpc_stage_geometry::CourseFrameKnot> wall_course_frame_knots;
  double wall_lateral_sample_step_m{};
  double wall_heading_bucket_width_rad{0.025};
  double wall_translation_bucket_width_m{};
  double wall_boundary_guard_m{0.001};
  std::optional<ReplayWorld> replay_world;
  double publication_interval_sec{};
};

/// Immutable output of the asynchronous preparation phase.  It is numerical
/// provenance, not an execution certificate: the final refined QP and its
/// solution still belong to the captured state in `snapshot` until a latest-
/// state feedback solve rebuilds and certifies them.
struct LatestStateFeedbackPreparation
{
  Snapshot snapshot;
  mpcc_rate_resolved_problem::AssemblyRequest final_problem;
  Eigen::VectorXd prepared_primal;
};

/// Result of moving an immutable semantic candidate to a later control clock
/// without changing the absolute time of any surviving future stage.  This is
/// observation-only architecture evidence: it does not carry a mailbox, Store
/// or publication API.
enum class TimeAlignedSuffixReason
{
  Accepted,
  InvalidRequest,
  TimeRegression,
  HorizonExhausted,
  ExecutionPrefixExhausted,
  SubminimumFirstStage,
  NominalPathMismatch,
  DynamicObstacleStageMismatch,
  Count,
};

const char * to_string(TimeAlignedSuffixReason reason) noexcept;

struct TimeAlignedSuffixRequest
{
  const Snapshot * source{};
  double control_prediction_origin_sec{};
  artifact::PredictedState initial_state;
  Eigen::Matrix<double, mpcc_rate_resolved::kInputDimension, 1>
  previous_input{
    Eigen::Matrix<double, mpcc_rate_resolved::kInputDimension, 1>::Zero()};
};

struct TimeAlignedSuffixResult
{
  TimeAlignedSuffixReason reason{TimeAlignedSuffixReason::InvalidRequest};
  std::size_t consumed_stage_count{};
  double elapsed_in_first_remaining_stage_sec{};
  double first_remaining_stage_duration_sec{};
  std::optional<Snapshot> snapshot;
  std::string detail{"not-evaluated"};
};

/// Rebuild a complete semantic suffix at the latest control origin.  State,
/// input, nominal path and dynamic-obstacle arrays all advance by one common
/// stage count.  The active first stage is shortened instead of restarting its
/// old duration, so surviving stages keep their original absolute timestamps.
TimeAlignedSuffixResult resolve_time_aligned_suffix(
  const TimeAlignedSuffixRequest & request) noexcept;

/// Observation-only result of transporting the immutable final refined QP to
/// the same absolute-time suffix as `resolve_time_aligned_suffix`.  This is a
/// formulation artifact only: it has no Store, mailbox or publisher API and
/// therefore cannot acquire production authority.
enum class TimeAlignedFeedbackProblemReason
{
  Accepted,
  InvalidRequest,
  SuffixRejected,
  PreparationDimensionMismatch,
  SemanticAdapterRejected,
  RefinementProvenanceMismatch,
  RelinearizationRejected,
  Count,
};

const char * to_string(TimeAlignedFeedbackProblemReason reason) noexcept;

struct TimeAlignedFeedbackProblemRequest
{
  const LatestStateFeedbackPreparation * preparation{};
  double control_prediction_origin_sec{};
  artifact::PredictedState initial_state;
  Eigen::Matrix<double, mpcc_rate_resolved::kInputDimension, 1>
  previous_input{
    Eigen::Matrix<double, mpcc_rate_resolved::kInputDimension, 1>::Zero()};
  persistent_osqp::PhysicalConstraintTolerance physical_constraint_tolerance;
};

struct TimeAlignedFeedbackProblemResult
{
  TimeAlignedFeedbackProblemReason reason{
    TimeAlignedFeedbackProblemReason::InvalidRequest};
  TimeAlignedSuffixResult suffix;
  std::optional<mpcc_rate_resolved_problem::AssemblyRequest> problem;
  /// Sliced former solution with latest x0.  It is used only as the tangent
  /// selector for relinearization; it is not a certified warm start.
  Eigen::VectorXd linearization_primal;
  std::string detail{"not-evaluated"};
};

/// Move a prepared refined QP to the latest semantic suffix without changing
/// the absolute timestamp of any surviving stage. Every retained affine
/// dynamics row is rebuilt around a suffix-owned tangent, and malformed wall
/// or obstacle row provenance is rejected instead of silently discarded.
TimeAlignedFeedbackProblemResult build_time_aligned_feedback_problem(
  const TimeAlignedFeedbackProblemRequest & request) noexcept;

/// Candidate-C outcome.  The immutable B-arm problem remains the source of
/// every cost and hard row; only its SQP tangent is rebuilt by rolling the
/// canonical nonlinear seven-state model from the latest exact x0.
enum class ReachableBridgeReason
{
  Accepted,
  TimeAlignedProblemRejected,
  DimensionMismatch,
  InputPrefixInfeasible,
  NonlinearTransitionRejected,
  RelinearizationRejected,
  AssemblyRejected,
  Exception,
  Count,
};

const char * to_string(ReachableBridgeReason reason) noexcept;

struct ReachableBridgeFeedbackProblemResult
{
  ReachableBridgeReason reason{
    ReachableBridgeReason::TimeAlignedProblemRejected};
  TimeAlignedFeedbackProblemReason time_aligned_problem_reason{
    TimeAlignedFeedbackProblemReason::InvalidRequest};
  TimeAlignedSuffixResult suffix;
  std::optional<mpcc_rate_resolved_problem::AssemblyRequest> problem;
  /// Exact nonlinear rollout used only as tangent and primal bootstrap.
  Eigen::VectorXd linearization_primal;
  std::size_t rollout_stage_count{};
  double maximum_direct_successor_gap{};
  double maximum_state_box_violation{};
  std::string detail{"not-evaluated"};
};

/// Build candidate C without Store/mailbox/publisher authority.  Prepared
/// suffix inputs are projected only into the B problem's existing certified
/// input and cumulative steering-prefix intervals, then propagated through
/// the canonical nonlinear model.  No state or final result is clamped.
ReachableBridgeFeedbackProblemResult
build_reachable_bridge_feedback_problem(
  const TimeAlignedFeedbackProblemRequest & request) noexcept;

enum class NonlinearInteriorWallReason
{
  Accepted,
  InvalidRequest,
  BaseAssemblyRejected,
  DimensionMismatch,
  TransitionLinearizationRejected,
  AugmentedProblemInvalid,
  Exception,
  Count,
};

const char * to_string(NonlinearInteriorWallReason reason) noexcept;

struct NonlinearInteriorWallProblemResult
{
  NonlinearInteriorWallReason reason{
    NonlinearInteriorWallReason::InvalidRequest};
  std::optional<mpcc_rate_resolved_problem::Problem> problem;
  std::size_t original_row_count{};
  std::size_t appended_row_count{};
  double maximum_candidate_violation_m{};
  std::string detail{"not-evaluated"};
};

/// Observation-only augmentation used to test the representation gap between
/// affine endpoint interpolation and exact nonlinear substage wall proof. The
/// returned Problem preserves the original assembled rows as an exact prefix.
NonlinearInteriorWallProblemResult build_nonlinear_interior_wall_problem(
  const Snapshot & snapshot,
  const mpcc_rate_resolved_problem::AssemblyRequest & assembly_request,
  const Eigen::VectorXd & linearization_primal) noexcept;

enum class LatestStateFeedbackReason
{
  InvalidRequest,
  AssemblyRejected,
  BootstrapRejected,
  SolveRejected,
  ArtifactRejected,
  PhysicalAdapterRejected,
  Accepted,
  Exception,
  Count,
};

const char * to_string(LatestStateFeedbackReason reason) noexcept;

struct LatestStateFeedbackRequest
{
  std::shared_ptr<const LatestStateFeedbackPreparation> preparation;
  /// Latest control-origin timestamp.  It becomes the origin of the newly
  /// solved artifact and must not precede the preparation capture.
  double control_prediction_origin_sec{};
  /// Wall-clock completion anchor used by the artifact timing contract.
  double observation_sec{};
  artifact::PredictedState initial_state;
  Eigen::Matrix<double, mpcc_rate_resolved::kInputDimension, 1>
  previous_input{
    Eigen::Matrix<double, mpcc_rate_resolved::kInputDimension, 1>::Zero()};
};

/// Offline architecture-audit ceiling.  More corrections would turn this
/// diagnostic into an unbounded runtime fallback and obscure the exit
/// classification it is intended to produce.
inline constexpr std::size_t kMaximumLatestStateMultiSqpAuditIterations{8U};

struct LatestStateFeedbackResult
{
  Identity identity;
  LatestStateFeedbackReason reason{LatestStateFeedbackReason::InvalidRequest};
  double compute_ms{};
  bool assembled{false};
  bool solve_attempted{false};
  bool solved{false};
  bool finite{false};
  bool constraints_satisfied{false};
  bool time_aligned_suffix_attempted{false};
  bool reachable_bridge_attempted{false};
  bool reachable_bridge_applied{false};
  ReachableBridgeReason reachable_bridge_reason{
    ReachableBridgeReason::TimeAlignedProblemRejected};
  std::size_t reachable_bridge_rollout_stage_count{};
  double reachable_bridge_maximum_direct_successor_gap{};
  double reachable_bridge_maximum_state_box_violation{};
  bool latest_state_multi_sqp_audit_requested{false};
  std::size_t latest_state_multi_sqp_iteration_limit{1U};
  std::size_t latest_state_multi_sqp_attempt_count{};
  std::size_t latest_state_multi_sqp_solve_count{};
  bool nonlinear_interior_wall_audit_requested{false};
  bool nonlinear_interior_wall_audit_applied{false};
  NonlinearInteriorWallReason nonlinear_interior_wall_reason{
    NonlinearInteriorWallReason::InvalidRequest};
  std::size_t nonlinear_interior_wall_row_count{};
  double nonlinear_interior_wall_maximum_candidate_violation_m{};
  TimeAlignedFeedbackProblemReason time_aligned_problem_reason{
    TimeAlignedFeedbackProblemReason::InvalidRequest};
  std::size_t consumed_stage_count{};
  double first_remaining_stage_duration_sec{};
  persistent_osqp::SolveTelemetry solver;
  artifact::RejectReason artifact_reject_reason{
    artifact::RejectReason::None};
  mpcc_rate_resolved_physical_adapter::RejectReason physical_adapter_reason{
    mpcc_rate_resolved_physical_adapter::RejectReason::InvalidArtifact};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason
  physical_exact_reason{
    race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason::TooFewStages};
  int physical_rejected_stage{-1};
  double physical_rejected_lateral_m{
    std::numeric_limits<double>::quiet_NaN()};
  double physical_rejected_lateral_lower_m{
    std::numeric_limits<double>::quiet_NaN()};
  double physical_rejected_lateral_upper_m{
    std::numeric_limits<double>::quiet_NaN()};
  double physical_lateral_bound_tolerance_m{};
  double physical_lateral_violation_m{};
  std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact;
  std::string detail{"not-evaluated"};
};

enum class Outcome
{
  BuildRejected,
  AssemblyRejected,
  SolveRejected,
  NonfiniteResult,
  PhysicalProofRejected,
  ActuationSampleRejected,
  ArtifactRejected,
  Solved,
  Exception,
  Count,
};

const char * to_string(Outcome outcome) noexcept;

enum class RecedingWarmStartReason
{
  Available,
  CurrentProblemBootstrap,
  EmptyCache,
  InvalidPrevious,
  InvalidCurrent,
  SemanticMismatch,
  TimeRegression,
  HorizonExhausted,
  DimensionMismatch,
};

const char * to_string(RecedingWarmStartReason reason) noexcept;

struct Result
{
  Identity identity;
  Outcome outcome{Outcome::BuildRejected};
  double completed_sec{};
  double compute_ms{};
  bool adapter_built{false};
  bool assembled{false};
  bool solve_attempted{false};
  bool solved{false};
  bool finite{false};
  bool constraints_satisfied{false};
  bool actuation_sampled{false};
  mpcc_rate_resolved::ActuationSampleReason actuation_sample_reason{
    mpcc_rate_resolved::ActuationSampleReason::Count};
  double first_acceleration_mps2{};
  double first_steering_rate_radps{};
  double first_virtual_progress_speed_mps{};
  double initial_steering_rad{};
  double solver_initial_steering_rad{};
  double sampled_steering_rad{};
  double calculated_terminal_steering_rad{};
  double first_stage_duration_sec{};
  double publication_interval_sec{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double first_steering_rate_physical_lower_radps{};
  double first_steering_rate_physical_upper_radps{};
  double first_steering_rate_solver_lower_radps{};
  double first_steering_rate_solver_upper_radps{};
  double first_steering_rate_certificate_margin_radps{};
  std::size_t planning_stage_count{};
  std::size_t certified_stage_count{};
  std::size_t sampled_stage_index{};
  double sampled_stage_elapsed_sec{};
  double certified_horizon_duration_sec{};
  double sampled_curvature_radpm{};
  double terminal_velocity_mps{};
  double terminal_progress_m{};
  double terminal_steering_rad{};
  double maximum_constraint_violation{};
  double maximum_normalized_constraint_violation{};
  int maximum_normalized_constraint_row{-1};
  mpcc_rate_resolved_problem::FirstStageInputFeasibility
    first_virtual_progress_feasibility;
  bool progress_wall_refinement_requested{false};
  bool pre_refinement_lateral_support_applied{false};
  mpcc_rate_resolved_wall_refinement::PreRefinementLateralSupportReason
    pre_refinement_lateral_support_reason{
      mpcc_rate_resolved_wall_refinement::
      PreRefinementLateralSupportReason::NotRequested};
  double pre_refinement_lateral_support_lower_m{};
  double pre_refinement_lateral_support_upper_m{};
  bool progress_wall_refinement_applied{false};
  bool progress_wall_refinement_solved{false};
  /// Numerical-owner trace.  A non-zero count proves that this evaluation
  /// submitted a wall-class QP directly to the equilibrated owner; it is not a
  /// retry count.
  std::size_t wall_refinement_solver_solve_count{};
  int wall_refinement_solver_scaling_iterations{};
  mpcc_progress::ProgressAlignedWallBoundsReason
    progress_wall_refinement_reason{
      mpcc_progress::ProgressAlignedWallBoundsReason::NotRequested};
  std::size_t progress_wall_refinement_aligned_stage_count{};
  std::size_t progress_wall_refinement_out_of_range_stage_count{};
  int progress_wall_refinement_first_failure_stage{-1};
  double progress_wall_refinement_maximum_mismatch_m{};
  bool dynamic_obstacle_refinement_requested{false};
  bool dynamic_obstacle_refinement_applied{false};
  bool dynamic_obstacle_refinement_solved{false};
  mpcc_rate_resolved_dynamic_obstacle::Reason
    dynamic_obstacle_refinement_reason{
      mpcc_rate_resolved_dynamic_obstacle::Reason::NotRequested};
  int dynamic_obstacle_resolved_side_sign{};
  int dynamic_obstacle_first_pass_side_stage{-1};
  std::size_t dynamic_obstacle_stay_behind_row_count{};
  std::size_t dynamic_obstacle_pass_side_row_count{};
  std::size_t dynamic_obstacle_ahead_row_count{};
  std::size_t dynamic_obstacle_diagonal_row_count{};
  bool dynamic_obstacle_physical_axis_support_applied{false};
  bool dynamic_obstacle_physical_diagonal_guidance_applied{false};
  double dynamic_obstacle_forced_constraint_fraction{1.0};
  int dynamic_obstacle_first_valid_stage{-1};
  double dynamic_obstacle_first_wall_only_progress_m{};
  double dynamic_obstacle_first_wall_only_effective_progress_m{};
  double dynamic_obstacle_first_wall_only_lateral_m{};
  double dynamic_obstacle_first_target_progress_m{};
  double dynamic_obstacle_first_target_lateral_m{};
  double dynamic_obstacle_first_stay_behind_margin_m{};
  double dynamic_obstacle_first_positive_side_margin_m{};
  double dynamic_obstacle_first_negative_side_margin_m{};
  bool physical_wall_refinement_requested{false};
  bool physical_wall_refinement_applied{false};
  bool physical_wall_refinement_solved{false};
  bool physical_wall_lag_pose_box_applied{false};
  bool physical_wall_heading_pose_box_applied{false};
  mpcc_rate_resolved_wall_refinement::Reason
    physical_wall_refinement_reason{
      mpcc_rate_resolved_wall_refinement::Reason::NotRequested};
  int physical_wall_refinement_first_failure_stage{-1};
  std::size_t physical_wall_refinement_checked_pose_count{};
  std::size_t physical_wall_refinement_cache_hit_count{};
  std::size_t physical_wall_refinement_cache_miss_count{};
  std::size_t physical_wall_refinement_cache_scanned_pose_count{};
  bool wall_feasibility_restoration_requested{false};
  bool wall_feasibility_restoration_attempted{false};
  bool wall_feasibility_restoration_seed_solved{false};
  bool wall_feasibility_restoration_final_refinement_built{false};
  bool wall_feasibility_restoration_final_solved{false};
  std::size_t wall_feasibility_restoration_sqp_count{};
  std::string wall_feasibility_restoration_detail{"not-requested"};
  RecedingWarmStartReason receding_warm_start_reason{
    RecedingWarmStartReason::EmptyCache};
  std::string receding_warm_start_diagnostic{"empty-cache"};
  std::size_t receding_warm_start_stage_advance{};
  bool receding_warm_start_applied{false};
  bool successive_linearization_requested{false};
  bool successive_linearization_applied{false};
  bool successive_linearization_bootstrap_applied{false};
  bool successive_linearization_solved{false};
  mpcc_rate_resolved_adapter::RelinearizationReason
    successive_linearization_reason{
      mpcc_rate_resolved_adapter::RelinearizationReason::InvalidRequest};
  int successive_linearization_failure_stage{-1};
  bool post_refinement_linearization_requested{false};
  bool post_refinement_linearization_applied{false};
  bool post_refinement_linearization_bootstrap_applied{false};
  bool post_refinement_linearization_solved{false};
  bool post_refinement_physical_proof_checked{false};
  bool post_refinement_physical_proof_accepted{false};
  std::size_t post_refinement_linearization_count{};
  mpcc_rate_resolved_adapter::RelinearizationReason
    post_refinement_linearization_reason{
      mpcc_rate_resolved_adapter::RelinearizationReason::InvalidRequest};
  int post_refinement_linearization_failure_stage{-1};
  /// Observation-only outer SQP which rebuilds dynamics, physical obstacle
  /// supports and wall rows around one common latest primal.  It has no
  /// Store/mailbox/publisher path and is never requested by evaluate().
  bool physical_dynamic_sqp_audit_requested{false};
  bool physical_dynamic_sqp_audit_applied{false};
  bool physical_dynamic_sqp_audit_solved{false};
  std::size_t physical_dynamic_sqp_audit_iteration_limit{};
  std::size_t physical_dynamic_sqp_audit_count{};
  std::string physical_dynamic_sqp_audit_detail{"not-requested"};
  persistent_osqp::SolveTelemetry solver;
  artifact::RejectReason execution_artifact_reject_reason{
    artifact::RejectReason::None};
  std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact;
  std::shared_ptr<const LatestStateFeedbackPreparation>
  latest_state_feedback_preparation;
  std::string detail;
};

bool identity_valid(const Identity & identity) noexcept;
bool result_valid(const Result & result) noexcept;

/// Successful full-horizon numerical iterate retained only as an initial
/// guess for the next QP.  It is neither an execution artifact nor authority:
/// every current problem is still solved and certified independently.
struct RecedingWarmStartSeed
{
  Identity identity;
  double control_prediction_origin_sec{};
  double course_progress_origin_m{};
  std::vector<double> stage_durations_sec;
  Eigen::VectorXd primal;
};

struct RecedingWarmStartResolution
{
  RecedingWarmStartReason reason{RecedingWarmStartReason::EmptyCache};
  std::string diagnostic{"empty-cache"};
  std::size_t stage_advance{};
  std::optional<persistent_osqp::WarmStart> warm_start;
};

/// Transport one successful seven-state iterate to the current control and
/// course-progress origins.  Tactical target/generation/side and all schemas
/// must remain identical; observation and stage geometry may advance because
/// the current QP proves them again.  Current x0 is always overwritten and
/// current dual rows are deliberately cold-started.
RecedingWarmStartResolution resolve_receding_warm_start(
  const RecedingWarmStartSeed & previous, const Snapshot & current,
  const mpcc_rate_resolved_problem::AssemblyRequest & current_problem,
  std::size_t current_constraint_count) noexcept;

/// Build a numerical seed solely from the current seven-state problem.  This
/// is used when no semantically compatible solved iterate exists.  It carries
/// no execution authority and does not cross an intent/formulation boundary:
/// inputs are projected into the current boxes and the current affine
/// dynamics are rolled out from the exact initial state.  When a preceding
/// iterate from the same semantic problem is supplied, only its input prefix
/// is transported; all states are rebuilt under the current equality rows and
/// all duals are cold-started.
std::optional<persistent_osqp::WarmStart> build_current_problem_bootstrap(
  const mpcc_rate_resolved_problem::AssemblyRequest & current_problem,
  std::size_t current_constraint_count,
  const Eigen::VectorXd * preceding_same_problem_primal = nullptr) noexcept;

/// Dedicated numerical owner for the rate-resolved shadow. Calls are
/// serialized; a successful full-horizon iterate may seed only a semantically
/// compatible current seven-state problem under the contract above.
class SolverContext
{
public:
  /// Observation-only choice for removing one artificial pose bucket from
  /// physical wall refinement. Physical wall/opponent proof remains
  /// unchanged and these modes have no production authority path.
  enum class WallBucketAuditMode
  {
    OmitHeading,
    OmitLag,
    OmitPose,
    OmitPoseDirect,
  };

  Result evaluate(const Snapshot & snapshot);
  /// Offline architecture comparison only. A relaxed wall-directed QP may
  /// generate a numerical tangent, but it can never become an artifact: the
  /// unchanged full QP and all physical proofs are rebuilt before acceptance.
  /// Production callers must use evaluate().
  Result evaluate_wall_feasibility_restoration_audit(
    const Snapshot & snapshot);
  Result evaluate_wall_bucket_audit(
    const Snapshot & snapshot, WallBucketAuditMode mode);
  /// Observation-only comparison arm.  The existing exact proof chain still
  /// owns acceptance, and this entry point cannot publish an artifact.
  Result evaluate_physical_dynamic_sqp_audit(
    const Snapshot & snapshot, std::size_t iteration_count);
  persistent_osqp::PhysicalConstraintTolerance
  physical_constraint_tolerance() const noexcept;

private:
  Result evaluate_impl(
    const Snapshot & snapshot, bool wall_feasibility_restoration_audit,
    std::optional<WallBucketAuditMode> wall_bucket_audit_mode,
    std::size_t physical_dynamic_sqp_audit_iteration_count);
  std::mutex mutex_;
  std::optional<RecedingWarmStartSeed> warm_start_seed_;
  mpcc_rate_resolved_wall_refinement::Cache wall_refinement_cache_;
  // Wall-bucket and coupled wall/opponent QPs are a distinct KKT class.  The
  // owner is chosen before solve; the normal solver is never tried first and
  // therefore this is not a retry or fallback path.
  persistent_osqp::PersistentOsqpSolver wall_refinement_solver_{
    persistent_osqp::ConstraintPreconditioningPolicy::
    RowToleranceNormalizedWithInternalEquilibration};
  persistent_osqp::PersistentOsqpSolver solver_{
    persistent_osqp::ConstraintPreconditioningPolicy::RowToleranceNormalized};
};

/// Observation-only latest-state feedback owner.  It has deliberately no
/// plan-store, mailbox or authority API.  Calls are serialized so persistent
/// OSQP reuse cannot cross concurrent controller callbacks.
class LatestStateFeedbackSolverContext
{
public:
  /// Historical A arm: replace x0 in the old final QP. Kept only so frozen
  /// mixed-origin regressions remain explicit; production must not call it.
  LatestStateFeedbackResult evaluate(
    const LatestStateFeedbackRequest & request);
  /// Observation-only B arm: advance every stage-indexed object with one
  /// absolute clock, relinearize the surviving prepared suffix and solve it
  /// from the latest serialized predecessor.
  LatestStateFeedbackResult evaluate_time_aligned(
    const LatestStateFeedbackRequest & request);
  /// Observation-only C arm: replace the joined old-state tangent with a
  /// dynamically reachable nonlinear rollout, then solve the unchanged B
  /// problem.  This method cannot acquire production authority.
  LatestStateFeedbackResult evaluate_reachable_bridge_time_aligned(
    const LatestStateFeedbackRequest & request);
  /// Observation-only D arm. Repeatedly relinearize and solve the exact same
  /// C problem, stopping on accepted exact nonlinear proof or the explicit
  /// audit limit. It has no production authority path.
  LatestStateFeedbackResult evaluate_reachable_bridge_multi_sqp_audit(
    const LatestStateFeedbackRequest & request,
    std::size_t iteration_limit);
  /// Observation-only E arm: add true nonlinear substage lateral wall
  /// tangents to C/D while retaining the unchanged exact physical proof.
  LatestStateFeedbackResult
  evaluate_reachable_bridge_nonlinear_interior_wall_audit(
    const LatestStateFeedbackRequest & request,
    std::size_t iteration_limit);
  persistent_osqp::PhysicalConstraintTolerance
  physical_constraint_tolerance() const noexcept;

private:
  LatestStateFeedbackResult evaluate_time_aligned_impl(
    const LatestStateFeedbackRequest & request,
    bool reachable_bridge_candidate,
    std::size_t multi_sqp_iteration_limit,
    bool nonlinear_interior_wall_audit);
  std::mutex mutex_;
  persistent_osqp::PersistentOsqpSolver solver_{
    persistent_osqp::ConstraintPreconditioningPolicy::RowToleranceNormalized};
};

enum class PublishReason
{
  Accepted,
  InvalidResult,
  SequenceRollback,
  SequenceNotSubmitted,
};

const char * to_string(PublishReason reason) noexcept;

struct MailboxState
{
  std::uint64_t latest_submitted_sequence{};
  std::uint64_t latest_published_sequence{};
  std::uint64_t accepted_count{};
  std::uint64_t invalid_result_count{};
  std::uint64_t sequence_rollback_count{};
  std::uint64_t sequence_not_submitted_count{};
  PublishReason last_reason{PublishReason::InvalidResult};
  bool result_available{false};
};

/// Observation-only monotonic transport. It intentionally has no plan-store
/// or authority API, so a shadow result cannot become executable by accident.
class Mailbox
{
public:
  bool register_submission(std::uint64_t sequence);
  PublishReason publish(Result result);
  std::optional<Result> latest_after(std::uint64_t consumed_sequence) const;
  MailboxState state() const;

private:
  mutable std::mutex mutex_;
  std::uint64_t latest_submitted_sequence_{};
  std::uint64_t latest_published_sequence_{};
  std::uint64_t accepted_count_{};
  std::uint64_t invalid_result_count_{};
  std::uint64_t sequence_rollback_count_{};
  std::uint64_t sequence_not_submitted_count_{};
  PublishReason last_reason_{PublishReason::InvalidResult};
  std::optional<Result> latest_result_;
};

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_SHADOW_HPP_
