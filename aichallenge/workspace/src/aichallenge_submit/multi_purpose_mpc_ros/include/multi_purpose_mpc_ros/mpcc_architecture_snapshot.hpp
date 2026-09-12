#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_SNAPSHOT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_SNAPSHOT_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"
#include "multi_purpose_mpc_ros/mpcc_publication_ledger.hpp"

#include <array>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation
{
struct Request;
}

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan
{
struct CertifiedPlan;
}

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_applied_program
{
class Certificate;
}

namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
{

/// Exact numerical boundary at which the production seven-state pipeline
/// rejected an Overtake problem.  These values identify different convex
/// problems and must not be collapsed into a generic "solver failed" event.
enum class PipelineStage
{
  Initial,
  SuccessiveLinearization,
  WallRefinement,
  DynamicObstacleRefinement,
  PostRefinementLinearization,
  PhysicalProof,
};

const char * to_string(PipelineStage stage) noexcept;

enum class RecordStatus
{
  Written,
  Duplicate,
  UnsupportedIntent,
  InvalidInput,
  IoFailure,
};

const char * to_string(RecordStatus status) noexcept;

struct RecordResult
{
  RecordStatus status{RecordStatus::InvalidInput};
  std::filesystem::path snapshot_file;
  std::string detail;
};

/// Persist the exact QP and immutable semantic/world provenance which were
/// already rejected by production for any canonical seven-state normal
/// intent. This function has no control authority and never changes a problem,
/// warm start or solver setting. At most one artifact per (intent, physical
/// homotopy, pipeline stage, failure outcome) is written by one process.  The
/// homotopy is part of the evidence boundary: collapsing opposite Follow
/// sides can hide the exact candidate that failed in production. Each original
/// bucket also has a fixed post-motion-final-loss bucket per native initializer;
/// optional diagnostic provenance never changes the original recording.
RecordResult record_failure(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const mpcc_rate_resolved_problem::AssemblyRequest & assembly_request,
  const mpcc_rate_resolved_problem::Problem & problem,
  const std::optional<persistent_osqp::WarmStart> & warm_start,
  const persistent_osqp::SolveOutcome & outcome,
  PipelineStage pipeline_stage,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const std::filesystem::path & output_root =
  std::filesystem::path{"mpcc_architecture_snapshots"}) noexcept;

/// Persist a complete immutable interaction at a physical-proof failure
/// boundary which has no rejected QP.  This is the canonical capture path for
/// outer exact-proof and retained terminal-contingency failures.  It has no
/// solver, Store, mailbox or publisher side effect.
RecordResult record_proof_failure(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  PipelineStage pipeline_stage,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const std::filesystem::path & output_root =
  std::filesystem::path{"mpcc_architecture_snapshots"}) noexcept;

/// Observation of the publication ledger at a later authority-loss boundary.
/// Bundle-source evidence is explicitly distinct from an executed trajectory.
/// The failure fingerprint joins this record to its current-world snapshot.
struct PublicationEvidence
{
  std::uint64_t failure_decision_id{};
  std::uint64_t failure_interaction_fingerprint{};
  double failure_observation_sec{};
  double failure_control_origin_sec{};
  std::string source_kind;
  std::uint64_t publication_decision_id{};
  double publication_control_origin_sec{};
  double publication_artifact_elapsed_sec{};
};

struct PublishedExecutionObservation
{
  std::shared_ptr<const mpcc_rate_resolved_shadow::Snapshot> source;
  std::shared_ptr<const mpcc_rate_resolved_execution_artifact::ExecutionArtifact> artifact;
  PublicationEvidence publication;
  /// Full immutable physical evidence, including plans derived without a
  /// solver snapshot. This pointer has no authority or solver ownership.
  std::shared_ptr<const mpcc_rate_resolved_certified_plan::CertifiedPlan> certified_plan{};
};

enum class AuthorityFailureBoundary
{
  TerminalContingency,
  FinalAuthority,
  MovingFinalAuthority,
  StopAlternateOverrun,
};

const char * to_string(AuthorityFailureBoundary boundary) noexcept;

struct AuthorityFailureObservation
{
  mpcc_rate_resolved_shadow::Snapshot current_world;
  AuthorityFailureBoundary boundary{AuthorityFailureBoundary::FinalAuthority};
  std::string detail;
  PublishedExecutionObservation published_execution;
  std::filesystem::path output_root{"mpcc_architecture_snapshots"};
  /// Exact failed or slow evaluation input, not a reconstructed later observation.
  /// The inspected plan is separate from the actual publication ledger above.
  std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request> revalidation_request;
  /// Earlier ordinary evaluation observed Accepted with terminal proof. This
  /// is not a publication claim; retain its own observations and source clock.
  std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request>
  previous_accepted_revalidation_request;
};

enum class ObservationAdmission
{
  Queued,
  Duplicate,
  Invalid,
  Stopped,
};

struct ScheduledFailureCapture;

/// Exact late publisher boundary, separate from an earlier solver-source world.
/// The certificate is immutable evidence only; the final guard already failed.
struct PublicationCallTiming
{
  double guard_finished_ros_sec{};
  double raw_finished_ros_sec{};
  double guard_wall_ms{};
  double raw_publish_wall_ms{};
  double final_publish_wall_ms{};
};

/// Diagnostic witness of movement at an authenticated normal send. It grants
/// no authority and is not a claim of restart or a terminal failure.
struct PublishedNormalMotionObservation
{
  std::uint64_t decision_id{};
  double pose_sec{};
  double forward_velocity_mps{};
  mpcc_vehicle_model::PublicationTransaction publication;
};

struct PostMotionFinalLossObservation
{
  std::uint64_t decision_id{};
  double clock_sec{};
  PublishedNormalMotionObservation prior_normal_motion;
};

struct PublicationFailureObservation
{
  std::shared_ptr<const mpcc_rate_resolved_applied_program::Certificate> certificate;
  std::uint64_t decision_id{};
  double nominal_sec{};
  double decision_clock_sec{};
  double before_clock_sec{};
  double after_clock_sec{};
  double wire_acceleration_mps2{};
  double wire_steering_rad{};
  bool after_publication{false};
  bool moving{false};
  std::filesystem::path output_root{"mpcc_architecture_snapshots"};
  std::shared_ptr<const ScheduledFailureCapture> scheduled_capture{};
  std::optional<PublicationCallTiming> call_timing{};
  std::optional<PublishedNormalMotionObservation> prior_normal_motion{};
};

RecordResult record_publication_failure(const PublicationFailureObservation &) noexcept;

/// Four original fixed buckets: stationary/moving x pre/post failure, plus
/// one paired stationary final loss with an earlier normal-motion witness.
/// Later events never replace admitted evidence. Shutdown drains the queue.
class FirstPublicationFailureRecorder
{
public:
  using Completion = std::function<void(
      const PublicationFailureObservation &, const RecordResult &)>;
  explicit FirstPublicationFailureRecorder(Completion completion = {});
  ~FirstPublicationFailureRecorder();
  FirstPublicationFailureRecorder(const FirstPublicationFailureRecorder &) = delete;
  FirstPublicationFailureRecorder & operator=(const FirstPublicationFailureRecorder &) = delete;
  ObservationAdmission submit(PublicationFailureObservation observation);
  /// Admit both failed selection attempts in one fixed bucket. Call only after
  /// neither due nor active programme supplies normal authority. A separate
  /// recorder owns these events so intermediate rejections cannot consume them.
  ObservationAdmission submit_selection_failure(
    std::array<PublicationFailureObservation, 2> observations);
  void stop() noexcept;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Preserve the first observation in each fixed intent/side/boundary bucket.
/// Unlike a receding planning worker, later observations cannot replace an
/// accepted pending event. All filesystem I/O and completion callbacks run on
/// the private worker, which also binds the failure side of publication
/// evidence to the queued world. Shutdown drains admitted events before joining.
class FirstAuthorityFailureRecorder
{
public:
  using Completion = std::function<void(
      const AuthorityFailureObservation &, const RecordResult &)>;
  explicit FirstAuthorityFailureRecorder(Completion completion = {});
  ~FirstAuthorityFailureRecorder();
  FirstAuthorityFailureRecorder(const FirstAuthorityFailureRecorder &) = delete;
  FirstAuthorityFailureRecorder & operator=(const FirstAuthorityFailureRecorder &) = delete;

  ObservationAdmission submit(AuthorityFailureObservation observation);
  void stop() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Atomically persist one failed current world and its corresponding published
/// source/artifact/clock. Deduplicate only the complete failure record; an
/// unrelated earlier publication must not consume this record's source slot.
/// Missing or invalid publication evidence is explicitly recorded while the
/// valid current world is preserved. Has no control authority or Store effects.
RecordResult record_authority_failure(
  const mpcc_rate_resolved_shadow::Snapshot & current_world,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const PublishedExecutionObservation & published_execution,
  const std::filesystem::path & output_root =
  std::filesystem::path{"mpcc_architecture_snapshots"},
  const std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request> &
  revalidation_request = {},
  const std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request> &
  previous_accepted_revalidation_request = {}) noexcept;

/// Save the original solver input AND the actual immutable artifact, without
/// resolving the input again. Shares the existing bounded failure deduplication
/// and atomic writer. Neither the artifact nor this evidence grants authority.
RecordResult record_published_execution(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const PublicationEvidence & publication,
  const std::string & failure_detail,
  const std::filesystem::path & output_root =
  std::filesystem::path{"mpcc_architecture_snapshots"}) noexcept;

struct RecordedQp
{
  mpcc_rate_resolved_problem::Problem problem;
  std::optional<persistent_osqp::WarmStart> warm_start;
  std::optional<Eigen::VectorXd> rejected_primal;
  std::string intent;
  std::string pipeline_stage;
  std::string failure_outcome;
  std::string failure_detail;
};

/// Complete immutable input shared by architecture candidates A/B/C/D.  It is
/// observation-only data and deliberately exposes no publisher, mailbox,
/// certified-plan store or command conversion API.
struct RecordedInteractionSnapshot
{
  mpcc_rate_resolved_shadow::Snapshot source;
  std::optional<mpcc_rate_resolved_problem::AssemblyRequest> assembly_request;
  std::optional<RecordedQp> recorded_qp;
  std::uint64_t interaction_fingerprint{};
};

/// Verify that every current-world and semantic field required to construct
/// an independent candidate and rerun exact physical proof is owned by the
/// snapshot.
bool interaction_snapshot_complete(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept;

/// Deterministic seal over semantic request, world, wall and obstacle inputs.
/// Returns zero for incomplete input.
std::uint64_t fingerprint_interaction_snapshot(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept;

bool interaction_snapshot_matches_fingerprint(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t expected_fingerprint) noexcept;

/// Load a replay-ready architecture snapshot.  Exact-QP-only legacy artifacts
/// remain available through load_recorded_qp but are rejected here with an
/// explicit incomplete detail.
std::optional<RecordedInteractionSnapshot> load_recorded_interaction_snapshot(
  const std::filesystem::path & snapshot_file,
  std::string * detail = nullptr) noexcept;

/// Load only the exact convex problem required for deterministic solver
/// replay.  Semantic and physical provenance remain in snapshot.yaml for the
/// B/C/D architecture comparison.
std::optional<RecordedQp> load_recorded_qp(
  const std::filesystem::path & snapshot_file,
  std::string * detail = nullptr) noexcept;

struct ReplayResult
{
  bool loaded{false};
  bool warm_start_requested{false};
  bool warm_start_available{false};
  persistent_osqp::SolveOutcome outcome;
  std::string detail;
};

/// Invoke the same solver and immutable row-preconditioning policy used by
/// production, either with the recorded warm start or from a cold state.
ReplayResult replay_recorded_qp(
  const std::filesystem::path & snapshot_file,
  bool use_recorded_warm_start) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_SNAPSHOT_HPP_
