#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_CERTIFIED_PLAN_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_CERTIFIED_PLAN_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow
{
struct Snapshot;
}

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace physical = mpcc_rate_resolved_physical_wall;
namespace shadow = mpcc_rate_resolved_shadow;
namespace contract = mpcc_execution_contract;

/// One immutable seven-state execution artifact joined to the exact physical
/// proof which accepted that artifact.  This is retained evidence only; it has
/// deliberately no command or publisher representation.
struct CertifiedPlan
{
  std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact;
  /// Exact immutable solver input which produced this artifact.  It is
  /// provenance only: certification does not make it an executed or published
  /// source.  Observation consumers may use it only after this plan crosses
  /// the sole command publisher.
  std::shared_ptr<const shadow::Snapshot> solver_source_snapshot;
  /// Exact immutable source consumed by the accepted physical result.  A
  /// retained consumer must match this static-world owner before reusing the
  /// certified suffix.
  std::shared_ptr<const physical::Snapshot> physical_snapshot;
  physical::Identity physical_identity;
  physical::Outcome physical_outcome{physical::Outcome::InvalidInput};
  contract::PhysicalWallCertificateDiagnostic physical_diagnostic;
  double physical_completed_sec{};
};

enum class RejectReason
{
  None,
  MissingArtifact,
  InvalidArtifact,
  InvalidPhysicalResult,
  InvalidPhysicalSnapshot,
  PhysicalProofRejected,
  PhysicalSnapshotMismatch,
  IdentityMismatch,
  Count,
};

const char * to_string(RejectReason reason) noexcept;
RejectReason validate(const CertifiedPlan & plan) noexcept;

struct BuildResult
{
  RejectReason reason{RejectReason::MissingArtifact};
  std::shared_ptr<const CertifiedPlan> plan;
};

BuildResult build(
  std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact,
  const physical::Snapshot & physical_snapshot,
  const physical::Result & physical_result,
  std::shared_ptr<const shadow::Snapshot> solver_source_snapshot = nullptr);

enum class StoreReason
{
  Accepted,
  InvalidPlan,
  StaleSequence,
};

const char * to_string(StoreReason reason) noexcept;

struct StoreState
{
  std::uint64_t latest_certified_sequence{};
  std::uint64_t latest_executed_sequence{};
  std::uint64_t latest_execution_decision_id{};
  std::uint64_t accepted_count{};
  std::uint64_t executed_count{};
  std::uint64_t invalid_plan_count{};
  std::uint64_t stale_sequence_count{};
  std::uint64_t certification_reject_count{};
  RejectReason last_certification_reason{RejectReason::MissingArtifact};
  StoreReason last_reason{StoreReason::InvalidPlan};
  bool candidate_available{false};
  bool executed_plan_available{false};
  bool published_bundle_source_available{false};
  std::uint64_t latest_published_bundle_decision_id{};
  double first_published_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double first_published_artifact_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
};

/// Atomic publication ledger entry.  The certificate records when a plan was
/// solved; this entry records when its first command was actually scheduled at
/// the actuator control origin.  Those clocks are deliberately not
/// interchangeable.
struct ExecutedPlanSnapshot
{
  std::shared_ptr<const CertifiedPlan> plan;
  /// Opposite homotopy certified from the exact same immutable source epoch.
  /// It is evidence only until current-world proof succeeds at consumption.
  std::shared_ptr<const CertifiedPlan> sibling_plan;
  double first_published_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  /// Artifact-local cursor whose command first crossed the publisher.  A
  /// candidate may be adopted from a time-aligned suffix, so execution cannot
  /// be reconstructed from publication time alone.
  double first_published_artifact_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
};

/// Immutable source identity and source-local cursor of the last stateless
/// current-world Bundle which actually crossed the publisher.  This is not an
/// assertion that the unmodified source plan was executed: every later command
/// must rebuild and prove a new current-world Bundle from this clock.
struct PublishedBundleSourceSnapshot
{
  std::shared_ptr<const CertifiedPlan> plan;
  std::shared_ptr<const CertifiedPlan> sibling_plan;
  std::uint64_t publication_decision_id{};
  double publication_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double publication_artifact_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
};

enum class PublishedSourceKind
{
  None,
  ExactExecutedPlan,
  CurrentWorldBundle,
};

const char * to_string(PublishedSourceKind kind) noexcept;

/// Atomic view of the source identity behind the last command which crossed
/// the sole publisher.  A current-world Bundle remains distinct from exact
/// executed evidence even though it takes publication-order precedence.
struct LatestPublishedSourceSnapshot
{
  PublishedSourceKind kind{PublishedSourceKind::None};
  std::shared_ptr<const CertifiedPlan> plan;
  std::shared_ptr<const CertifiedPlan> sibling_plan;
  std::uint64_t publication_decision_id{};
  double publication_control_origin_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double publication_artifact_elapsed_sec{
    std::numeric_limits<double>::quiet_NaN()};
};

/// Atomic candidate lifecycle entry.  A sibling may only accompany a normal
/// Cruise/Follow dynamic-obstacle plan from the same source epoch and opposite
/// homotopy.  Newer unrelated worker output cannot overwrite this association.
struct CandidatePlanSnapshot
{
  std::shared_ptr<const CertifiedPlan> plan;
  std::shared_ptr<const CertifiedPlan> sibling_plan;
};

struct AdmissionResult
{
  RejectReason certification_reason{RejectReason::MissingArtifact};
  StoreReason store_reason{StoreReason::InvalidPlan};

  bool accepted() const noexcept
  {
    return certification_reason == RejectReason::None &&
           store_reason == StoreReason::Accepted;
  }
};

/// Two-phase monotonic store. Certification creates a candidate only. A plan
/// becomes retained evidence exclusively after the exact command selected from
/// it has crossed the publisher boundary and `mark_executed()` is called.
/// Solver completion must never overwrite the last actually executed plan.
class Store
{
public:
  Store() = default;
  Store(const Store &) = delete;
  Store & operator=(const Store &) = delete;

  AdmissionResult certify_and_replace(
    std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact,
    const physical::Snapshot & physical_snapshot,
    const physical::Result & physical_result,
    std::shared_ptr<const shadow::Snapshot> solver_source_snapshot = nullptr);
  StoreReason replace(std::shared_ptr<const CertifiedPlan> plan);
  StoreReason replace_pair(
    std::shared_ptr<const CertifiedPlan> selected_plan,
    std::shared_ptr<const CertifiedPlan> sibling_plan);
  std::shared_ptr<const CertifiedPlan> candidate_snapshot() const;
  CandidatePlanSnapshot candidate_with_sibling_snapshot() const;
  /// Record the exact plan whose command crossed the publisher boundary.
  /// Artifact sequences order one producer's certification stream; they do
  /// not order publication across the normal and pre-entry Gate A producers.
  /// Publication decision identity is therefore the sole execution ledger.
  StoreReason mark_executed(
    std::shared_ptr<const CertifiedPlan> plan,
    std::uint64_t publication_decision_id,
    double publication_control_origin_sec,
    double publication_artifact_elapsed_sec);
  StoreReason mark_executed(
    std::shared_ptr<const CertifiedPlan> plan,
    std::shared_ptr<const CertifiedPlan> sibling_plan,
    std::uint64_t publication_decision_id,
    double publication_control_origin_sec,
    double publication_artifact_elapsed_sec);
  StoreReason record_published_bundle_source(
    std::shared_ptr<const CertifiedPlan> plan,
    std::uint64_t publication_decision_id,
    double publication_control_origin_sec,
    double publication_artifact_elapsed_sec);
  StoreReason record_published_bundle_source(
    std::shared_ptr<const CertifiedPlan> plan,
    std::shared_ptr<const CertifiedPlan> sibling_plan,
    std::uint64_t publication_decision_id,
    double publication_control_origin_sec,
    double publication_artifact_elapsed_sec);
  /// A later exact-plan publication supersedes any Bundle source without
  /// changing exact execution identity or pretending that a new solve ran.
  void supersede_published_bundle_source(
    std::uint64_t publication_decision_id);
  /// Last plan whose command was successfully published.
  std::shared_ptr<const CertifiedPlan> snapshot() const;
  /// Last plan and the control-time origin of its first published command,
  /// read under one lock so a consumer cannot join mismatched lifecycle data.
  ExecutedPlanSnapshot executed_snapshot() const;
  PublishedBundleSourceSnapshot published_bundle_source_snapshot() const;
  LatestPublishedSourceSnapshot latest_published_source_snapshot() const;
  StoreState state() const;
  bool clear();
  bool clear_if_sequence(std::uint64_t expected_sequence);

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const CertifiedPlan> candidate_plan_;
  std::shared_ptr<const CertifiedPlan> candidate_sibling_plan_;
  std::shared_ptr<const CertifiedPlan> executed_plan_;
  std::shared_ptr<const CertifiedPlan> executed_sibling_plan_;
  std::uint64_t latest_executed_sequence_{};
  std::uint64_t latest_execution_decision_id_{};
  std::shared_ptr<const CertifiedPlan> published_bundle_source_plan_;
  std::shared_ptr<const CertifiedPlan> published_bundle_source_sibling_plan_;
  std::uint64_t latest_published_bundle_decision_id_{};
  double published_bundle_control_origin_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  double published_bundle_artifact_elapsed_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t executed_count_{};
  std::uint64_t latest_certified_sequence_{};
  std::uint64_t accepted_count_{};
  std::uint64_t invalid_plan_count_{};
  std::uint64_t stale_sequence_count_{};
  std::uint64_t certification_reject_count_{};
  double first_published_control_origin_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  double first_published_artifact_elapsed_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  RejectReason last_certification_reason_{RejectReason::MissingArtifact};
  StoreReason last_reason_{StoreReason::InvalidPlan};
};

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_CERTIFIED_PLAN_HPP_
