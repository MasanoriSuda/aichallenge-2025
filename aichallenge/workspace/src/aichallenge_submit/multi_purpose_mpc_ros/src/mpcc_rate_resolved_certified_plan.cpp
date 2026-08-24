#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"

#include <cmath>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan
{

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::MissingArtifact: return "missing-artifact";
    case RejectReason::InvalidArtifact: return "invalid-artifact";
    case RejectReason::InvalidPhysicalResult: return "invalid-physical-result";
    case RejectReason::PhysicalProofRejected: return "physical-proof-rejected";
    case RejectReason::IdentityMismatch: return "identity-mismatch";
    case RejectReason::Count: break;
  }
  return "unknown";
}

RejectReason validate(const CertifiedPlan & plan) noexcept
{
  if (plan.execution_artifact == nullptr) {
    return RejectReason::MissingArtifact;
  }
  if (artifact::validate(*plan.execution_artifact) != artifact::RejectReason::None) {
    return RejectReason::InvalidArtifact;
  }
  if (!physical::identity_valid(plan.physical_identity) ||
      !std::isfinite(plan.physical_completed_sec) ||
      plan.physical_completed_sec < plan.physical_identity.captured_sec)
  {
    return RejectReason::InvalidPhysicalResult;
  }
  if (plan.physical_outcome != physical::Outcome::Accepted ||
      plan.physical_diagnostic.reason !=
      contract::PhysicalWallCertificateReason::Accepted)
  {
    return RejectReason::PhysicalProofRejected;
  }
  if (!artifact::same_identity(
      plan.execution_artifact->identity, plan.physical_identity.artifact))
  {
    return RejectReason::IdentityMismatch;
  }
  return RejectReason::None;
}

BuildResult build(
  std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact,
  const physical::Result & physical_result)
{
  BuildResult result;
  if (execution_artifact == nullptr) {
    return result;
  }
  if (artifact::validate(*execution_artifact) != artifact::RejectReason::None) {
    result.reason = RejectReason::InvalidArtifact;
    return result;
  }
  if (!physical::result_valid(physical_result)) {
    result.reason = RejectReason::InvalidPhysicalResult;
    return result;
  }
  if (physical_result.outcome != physical::Outcome::Accepted ||
      physical_result.diagnostic.reason !=
      contract::PhysicalWallCertificateReason::Accepted)
  {
    result.reason = RejectReason::PhysicalProofRejected;
    return result;
  }
  if (!artifact::same_identity(
      execution_artifact->identity, physical_result.identity.artifact))
  {
    result.reason = RejectReason::IdentityMismatch;
    return result;
  }

  auto plan = std::make_shared<CertifiedPlan>();
  plan->execution_artifact = std::move(execution_artifact);
  plan->physical_identity = physical_result.identity;
  plan->physical_outcome = physical_result.outcome;
  plan->physical_diagnostic = physical_result.diagnostic;
  plan->physical_completed_sec = physical_result.completed_sec;
  result.reason = validate(*plan);
  if (result.reason == RejectReason::None) {
    result.plan = std::move(plan);
  }
  return result;
}

const char * to_string(const StoreReason reason) noexcept
{
  switch (reason) {
    case StoreReason::Accepted: return "accepted";
    case StoreReason::InvalidPlan: return "invalid-plan";
    case StoreReason::StaleSequence: return "stale-sequence";
  }
  return "unknown";
}

AdmissionResult Store::certify_and_replace(
  std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact,
  const physical::Result & physical_result)
{
  const auto certified = build(std::move(execution_artifact), physical_result);
  if (certified.plan == nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++certification_reject_count_;
    last_certification_reason_ = certified.reason;
    last_reason_ = StoreReason::InvalidPlan;
    return AdmissionResult{certified.reason, StoreReason::InvalidPlan};
  }
  const auto store_reason = replace(certified.plan);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_certification_reason_ = RejectReason::None;
  }
  return AdmissionResult{certified.reason, store_reason};
}

StoreReason Store::replace(std::shared_ptr<const CertifiedPlan> plan)
{
  if (plan == nullptr || validate(*plan) != RejectReason::None) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = StoreReason::InvalidPlan;
    return StoreReason::InvalidPlan;
  }
  const auto sequence = plan->execution_artifact->identity.sequence;
  std::lock_guard<std::mutex> lock(mutex_);
  if (sequence <= latest_accepted_sequence_) {
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }
  plan_ = std::move(plan);
  latest_accepted_sequence_ = sequence;
  ++accepted_count_;
  last_reason_ = StoreReason::Accepted;
  return StoreReason::Accepted;
}

std::shared_ptr<const CertifiedPlan> Store::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_;
}

StoreState Store::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return StoreState{
    latest_accepted_sequence_, accepted_count_, invalid_plan_count_,
    stale_sequence_count_, certification_reject_count_,
    last_certification_reason_, last_reason_, static_cast<bool>(plan_)};
}

bool Store::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool had_plan = static_cast<bool>(plan_);
  plan_.reset();
  return had_plan;
}

bool Store::clear_if_sequence(const std::uint64_t expected_sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (plan_ == nullptr ||
      plan_->execution_artifact->identity.sequence != expected_sequence)
  {
    return false;
  }
  plan_.reset();
  return true;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan
