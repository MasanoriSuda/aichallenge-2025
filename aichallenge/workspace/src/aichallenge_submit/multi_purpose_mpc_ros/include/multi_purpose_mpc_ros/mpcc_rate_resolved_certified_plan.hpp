#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_CERTIFIED_PLAN_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_CERTIFIED_PLAN_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"

#include <cstdint>
#include <memory>
#include <mutex>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace physical = mpcc_rate_resolved_physical_wall;
namespace contract = mpcc_execution_contract;

/// One immutable six-state execution artifact joined to the exact physical
/// proof which accepted that artifact.  This is retained evidence only; it has
/// deliberately no command or publisher representation.
struct CertifiedPlan
{
  std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact;
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
  const physical::Result & physical_result);

enum class StoreReason
{
  Accepted,
  InvalidPlan,
  StaleSequence,
};

const char * to_string(StoreReason reason) noexcept;

struct StoreState
{
  std::uint64_t latest_accepted_sequence{};
  std::uint64_t accepted_count{};
  std::uint64_t invalid_plan_count{};
  std::uint64_t stale_sequence_count{};
  std::uint64_t certification_reject_count{};
  RejectReason last_certification_reason{RejectReason::MissingArtifact};
  StoreReason last_reason{StoreReason::InvalidPlan};
  bool plan_available{false};
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

/// All-or-nothing monotonic store.  A rejected replacement cannot destroy the
/// last accepted same-formulation plan needed by a later retained admission.
class Store
{
public:
  Store() = default;
  Store(const Store &) = delete;
  Store & operator=(const Store &) = delete;

  AdmissionResult certify_and_replace(
    std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact,
    const physical::Snapshot & physical_snapshot,
    const physical::Result & physical_result);
  StoreReason replace(std::shared_ptr<const CertifiedPlan> plan);
  std::shared_ptr<const CertifiedPlan> snapshot() const;
  StoreState state() const;
  bool clear();
  bool clear_if_sequence(std::uint64_t expected_sequence);

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const CertifiedPlan> plan_;
  std::uint64_t latest_accepted_sequence_{};
  std::uint64_t accepted_count_{};
  std::uint64_t invalid_plan_count_{};
  std::uint64_t stale_sequence_count_{};
  std::uint64_t certification_reject_count_{};
  RejectReason last_certification_reason_{RejectReason::MissingArtifact};
  StoreReason last_reason_{StoreReason::InvalidPlan};
};

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_CERTIFIED_PLAN_HPP_
