#ifndef MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_REVALIDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_REVALIDATION_HPP_

#include "multi_purpose_mpc_ros/canonical_execution_plan.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::canonical_retained_revalidation
{

namespace contract = mpcc_execution_contract;
namespace plan = canonical_execution_plan;

struct RetainedStageSample
{
  std::size_t control_stage_index{};
  std::size_t endpoint_state_index{};
  double relative_time_sec{};
  double segment_duration_sec{};
  double segment_start_progress_m{};
  double absolute_progress_m{};
  plan::CanonicalPredictedState endpoint;
};

struct RetainedExecutionWindow
{
  std::uint64_t plan_id{};
  plan::CanonicalExecutionCursor cursor;
  plan::CanonicalPredictedState expected_current_state;
  double expected_current_progress_m{};
  std::vector<RetainedStageSample> samples;
};

enum class RetainedExecutionWindowReason
{
  Accepted,
  InvalidPlan,
  CursorUnavailable,
  PlanIdentityMismatch,
  ExecutionWindowMismatch,
  InvalidPartialStage,
  InvalidProgressEvolution,
};

const char * to_string(RetainedExecutionWindowReason reason) noexcept;

struct RetainedExecutionWindowResult
{
  RetainedExecutionWindowReason reason{
    RetainedExecutionWindowReason::InvalidPlan};
  std::optional<RetainedExecutionWindow> window;
};

RetainedExecutionWindowResult build_retained_execution_window(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor);

struct RetainedCourseFrameProgressRange
{
  double minimum_progress_m{};
  double maximum_progress_m{};
};

/// Return the closed progress interval whose current reference geometry is
/// required to reconstruct a retained execution window.  The interval includes
/// both the newly measured (already branch-lifted) origin and every retained
/// state used by current-world proof; it is therefore intentionally allowed to
/// begin behind the measured origin.
std::optional<RetainedCourseFrameProgressRange>
required_course_frame_progress_range(
  const RetainedExecutionWindow & window,
  double lifted_measured_progress_m) noexcept;

struct CircularProgressLiftRequest
{
  double measured_progress_m{};
  double retained_reference_progress_m{};
  double path_length_m{};
  double continuity_tolerance_m{};
  bool circular{false};
};

enum class CircularProgressLiftReason
{
  Accepted,
  InvalidInput,
  AmbiguousBranch,
  Discontinuous,
};

const char * to_string(CircularProgressLiftReason reason) noexcept;

struct CircularProgressLiftResult
{
  CircularProgressLiftReason reason{CircularProgressLiftReason::InvalidInput};
  double lifted_progress_m{};
  long lap_offset{};
};

CircularProgressLiftResult lift_progress_to_retained_branch(
  const CircularProgressLiftRequest & request) noexcept;

struct CurrentExecutionProvenance
{
  std::uint64_t decision_id{};
  contract::ControlIntent intent{contract::ControlIntent::Unknown};
  std::uint64_t intent_generation{};
  std::uint64_t observation_generation{};
  std::uint64_t stage_geometry_id{};
  std::uint64_t target_obstacle_generation{};
  std::string target_id;
  std::uint64_t control_pose_id{};
  std::uint64_t course_frame_window_id{};
  std::uint64_t obstacle_tube_id{};
  double observation_sec{};
  double path_length_m{};
  bool circular{false};
};

bool current_execution_provenance_complete(
  const CurrentExecutionProvenance & provenance) noexcept;

struct RetainedPathSegmentEvaluation
{
  std::uint64_t observation_generation{};
  std::uint64_t stage_geometry_id{};
  std::uint64_t target_obstacle_generation{};
  std::uint64_t control_pose_id{};
  std::uint64_t course_frame_window_id{};
  std::uint64_t obstacle_tube_id{};
  double start_progress_m{};
  double end_progress_m{};
  bool checked{false};
  bool wall_clear{false};
  bool obstacles_clear{false};
  double minimum_wall_clearance_m{};
  double minimum_obstacle_clearance_m{};
};

struct RetainedStageSafetyEvaluation
{
  std::size_t control_stage_index{};
  double relative_time_sec{};
  double segment_duration_sec{};
  double segment_start_progress_m{};
  double absolute_progress_m{};
  std::uint64_t observation_generation{};
  std::uint64_t stage_geometry_id{};
  std::uint64_t target_obstacle_generation{};
  std::uint64_t course_frame_window_id{};
  std::uint64_t obstacle_tube_id{};
  bool course_frame_available{false};
  bool wall_checked{false};
  bool wall_clear{false};
  bool obstacle_checked{false};
  bool obstacles_clear{false};
  double minimum_wall_clearance_m{};
  double minimum_obstacle_clearance_m{};
};

struct RetainedExecutionProofRequest
{
  CurrentExecutionProvenance current;
  double measured_course_progress_m{};
  double progress_continuity_tolerance_m{};
  RetainedPathSegmentEvaluation measured_to_control_prefix;
  RetainedPathSegmentEvaluation control_to_retained_connector;
  std::vector<RetainedStageSafetyEvaluation> stage_evaluations;
};

struct RetainedExecutionProof
{
  CurrentExecutionProvenance current;
  std::uint64_t plan_id{};
  plan::CanonicalExecutionCursor cursor;
  double lifted_current_progress_m{};
  long lap_offset{};
  RetainedExecutionWindow window;
  RetainedPathSegmentEvaluation measured_to_control_prefix;
  RetainedPathSegmentEvaluation control_to_retained_connector;
  std::vector<RetainedStageSafetyEvaluation> stage_evaluations;
  contract::PhysicalCertificate physical;
  std::uint64_t proof_fingerprint{};
};

enum class RetainedExecutionProofReason
{
  Accepted,
  InvalidPlan,
  CursorUnavailable,
  InvalidCurrentProvenance,
  IntentMismatch,
  IntentGenerationMismatch,
  TargetIdentityMismatch,
  ProgressLiftRejected,
  PrefixIdentityMismatch,
  DelayPrefixRejected,
  ConnectorRejected,
  StageEvaluationCountMismatch,
  StageEvaluationIdentityMismatch,
  CourseFrameUnavailable,
  WallRejected,
  ObstacleRejected,
  InvalidClearance,
  FingerprintMismatch,
};

const char * to_string(RetainedExecutionProofReason reason) noexcept;

struct RetainedExecutionProofResult
{
  RetainedExecutionProofReason reason{
    RetainedExecutionProofReason::InvalidPlan};
  std::optional<RetainedExecutionProof> proof;
};

RetainedExecutionProofResult build_retained_execution_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const RetainedExecutionProofRequest & request);

RetainedExecutionProofReason validate_retained_execution_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const CurrentExecutionProvenance & current,
  const RetainedExecutionProof & proof);

enum class RetainedCandidateBuildReason
{
  Accepted,
  InvalidPlan,
  CursorUnavailable,
  ProofRejected,
};

const char * to_string(RetainedCandidateBuildReason reason) noexcept;

struct RetainedCandidateBuildResult
{
  RetainedCandidateBuildReason reason{
    RetainedCandidateBuildReason::InvalidPlan};
  RetainedExecutionProofReason proof_reason{
    RetainedExecutionProofReason::InvalidPlan};
  std::optional<contract::CanonicalNormalCandidate> candidate;
};

RetainedCandidateBuildResult build_canonical_retained_candidate(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const CurrentExecutionProvenance & current,
  const RetainedExecutionProof & proof);

} // namespace multi_purpose_mpc_ros::canonical_retained_revalidation

#endif // MULTI_PURPOSE_MPC_ROS__CANONICAL_RETAINED_REVALIDATION_HPP_
