#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"

#include <cmath>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan
{
namespace
{

bool same_plan_identity(
  const std::shared_ptr<const CertifiedPlan> & lhs,
  const std::shared_ptr<const CertifiedPlan> & rhs) noexcept
{
  return
    lhs != nullptr && rhs != nullptr &&
    lhs->execution_artifact != nullptr && rhs->execution_artifact != nullptr &&
    artifact::same_identity(
    lhs->execution_artifact->identity, rhs->execution_artifact->identity);
}

bool sibling_pair_valid(
  const std::shared_ptr<const CertifiedPlan> & selected,
  const std::shared_ptr<const CertifiedPlan> & sibling) noexcept
{
  if (
    selected == nullptr || sibling == nullptr ||
    validate(*selected) != RejectReason::None ||
    validate(*sibling) != RejectReason::None ||
    selected->execution_artifact == nullptr ||
    sibling->execution_artifact == nullptr)
  {
    return false;
  }
  const auto & selected_identity = selected->execution_artifact->identity;
  const auto & sibling_identity = sibling->execution_artifact->identity;
  const auto & selected_context = selected_identity.source_context;
  const auto & sibling_context = sibling_identity.source_context;
  const bool normal_avoidance =
    selected_context.intent == contract::ControlIntent::Cruise ||
    selected_context.intent == contract::ControlIntent::Follow;
  const bool active_overtake =
    selected_context.intent == contract::ControlIntent::ShiftOut ||
    selected_context.intent == contract::ControlIntent::Pass;
  if (
    selected_identity.sequence != sibling_identity.sequence ||
    selected_identity.snapshot_sec != sibling_identity.snapshot_sec ||
    (!normal_avoidance && !active_overtake) ||
    sibling_context.intent != selected_context.intent ||
    !selected_context.dynamic_obstacle_constraint_active ||
    !sibling_context.dynamic_obstacle_constraint_active ||
    (selected_context.dynamic_obstacle_side_sign != -1 &&
    selected_context.dynamic_obstacle_side_sign != 1) ||
    sibling_context.dynamic_obstacle_side_sign !=
    -selected_context.dynamic_obstacle_side_sign)
  {
    return false;
  }
  auto expected_sibling = selected_context;
  if (active_overtake) {
    if (
      (selected_context.execution_side_sign != -1 &&
      selected_context.execution_side_sign != 1) ||
      sibling_context.execution_side_sign !=
      -selected_context.execution_side_sign)
    {
      return false;
    }
    expected_sibling.execution_side_sign =
      sibling_context.execution_side_sign;
  }
  expected_sibling.dynamic_obstacle_side_sign =
    sibling_context.dynamic_obstacle_side_sign;
  expected_sibling.fingerprint = 0U;
  expected_sibling = contract::seal_problem_context(
    std::move(expected_sibling));
  return expected_sibling.fingerprint == sibling_context.fingerprint;
}

std::shared_ptr<const CertifiedPlan> associated_sibling(
  const std::shared_ptr<const CertifiedPlan> & published,
  const std::shared_ptr<const CertifiedPlan> & selected,
  const std::shared_ptr<const CertifiedPlan> & sibling) noexcept
{
  if (!sibling_pair_valid(selected, sibling)) {
    return nullptr;
  }
  if (same_plan_identity(published, selected)) {
    return sibling;
  }
  if (same_plan_identity(published, sibling)) {
    return selected;
  }
  return nullptr;
}

}  // namespace

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::MissingArtifact: return "missing-artifact";
    case RejectReason::InvalidArtifact: return "invalid-artifact";
    case RejectReason::InvalidPhysicalResult: return "invalid-physical-result";
    case RejectReason::InvalidPhysicalSnapshot:
      return "invalid-physical-snapshot";
    case RejectReason::PhysicalProofRejected: return "physical-proof-rejected";
    case RejectReason::PhysicalSnapshotMismatch:
      return "physical-snapshot-mismatch";
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
  if (
    plan.physical_snapshot == nullptr ||
    !physical::snapshot_valid(*plan.physical_snapshot))
  {
    return RejectReason::InvalidPhysicalSnapshot;
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
  if (!physical::same_identity(
      plan.physical_snapshot->identity, plan.physical_identity))
  {
    return RejectReason::PhysicalSnapshotMismatch;
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
  const physical::Snapshot & physical_snapshot,
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
  if (!physical::snapshot_valid(physical_snapshot)) {
    result.reason = RejectReason::InvalidPhysicalSnapshot;
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
  if (!physical::same_identity(
      physical_snapshot.identity, physical_result.identity))
  {
    result.reason = RejectReason::PhysicalSnapshotMismatch;
    return result;
  }

  auto plan = std::make_shared<CertifiedPlan>();
  plan->execution_artifact = std::move(execution_artifact);
  plan->physical_snapshot =
    std::make_shared<const physical::Snapshot>(physical_snapshot);
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
  const physical::Snapshot & physical_snapshot,
  const physical::Result & physical_result)
{
  const auto certified = build(
    std::move(execution_artifact), physical_snapshot, physical_result);
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
  return replace_pair(std::move(plan), nullptr);
}

StoreReason Store::replace_pair(
  std::shared_ptr<const CertifiedPlan> selected_plan,
  std::shared_ptr<const CertifiedPlan> sibling_plan)
{
  if (
    selected_plan == nullptr ||
    validate(*selected_plan) != RejectReason::None ||
    (sibling_plan != nullptr &&
    !sibling_pair_valid(selected_plan, sibling_plan)))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = StoreReason::InvalidPlan;
    return StoreReason::InvalidPlan;
  }
  const auto sequence =
    selected_plan->execution_artifact->identity.sequence;
  std::lock_guard<std::mutex> lock(mutex_);
  if (sequence <= latest_certified_sequence_) {
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }
  candidate_plan_ = std::move(selected_plan);
  candidate_sibling_plan_ = std::move(sibling_plan);
  latest_certified_sequence_ = sequence;
  ++accepted_count_;
  last_reason_ = StoreReason::Accepted;
  return StoreReason::Accepted;
}

std::shared_ptr<const CertifiedPlan> Store::candidate_snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return candidate_plan_;
}

CandidatePlanSnapshot Store::candidate_with_sibling_snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return CandidatePlanSnapshot{candidate_plan_, candidate_sibling_plan_};
}

StoreReason Store::mark_executed(
  std::shared_ptr<const CertifiedPlan> plan,
  const std::uint64_t publication_decision_id,
  const double publication_control_origin_sec,
  const double publication_artifact_elapsed_sec)
{
  return mark_executed(
    std::move(plan), nullptr, publication_decision_id,
    publication_control_origin_sec, publication_artifact_elapsed_sec);
}

StoreReason Store::mark_executed(
  std::shared_ptr<const CertifiedPlan> plan,
  std::shared_ptr<const CertifiedPlan> sibling_plan,
  const std::uint64_t publication_decision_id,
  const double publication_control_origin_sec,
  const double publication_artifact_elapsed_sec)
{
  if (
    plan == nullptr || validate(*plan) != RejectReason::None ||
    (sibling_plan != nullptr &&
    !sibling_pair_valid(plan, sibling_plan)) ||
    publication_decision_id == 0U ||
    !std::isfinite(publication_control_origin_sec) ||
    publication_control_origin_sec < 0.0 ||
    !std::isfinite(publication_artifact_elapsed_sec) ||
    publication_artifact_elapsed_sec < 0.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = StoreReason::InvalidPlan;
    return StoreReason::InvalidPlan;
  }
  const auto sequence = plan->execution_artifact->identity.sequence;
  std::lock_guard<std::mutex> lock(mutex_);
  if (sibling_plan == nullptr) {
    sibling_plan = associated_sibling(
      plan, candidate_plan_, candidate_sibling_plan_);
  }
  const bool same_source =
    published_bundle_source_plan_ != nullptr &&
    artifact::same_identity(
    published_bundle_source_plan_->execution_artifact->identity,
    plan->execution_artifact->identity);
  if (sibling_plan == nullptr && same_source) {
    sibling_plan = published_bundle_source_sibling_plan_;
  }
  if (publication_decision_id < latest_published_bundle_decision_id_) {
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }
  const bool same_executed_identity =
    executed_plan_ != nullptr &&
    artifact::same_identity(
      executed_plan_->execution_artifact->identity,
      plan->execution_artifact->identity);
  if (sibling_plan == nullptr && same_executed_identity) {
    sibling_plan = executed_sibling_plan_;
  }
  if (publication_decision_id < latest_execution_decision_id_) {
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }
  if (publication_decision_id == latest_execution_decision_id_) {
    if (
      same_executed_identity &&
      std::abs(
        first_published_control_origin_sec_ -
        publication_control_origin_sec) <= 1e-9 &&
      std::abs(
        first_published_artifact_elapsed_sec_ -
        publication_artifact_elapsed_sec) <= 1e-9)
    {
      last_reason_ = StoreReason::Accepted;
      return StoreReason::Accepted;
    }
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }
  if (
    same_executed_identity &&
    publication_control_origin_sec + 1e-9 <
    first_published_control_origin_sec_)
  {
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }
  executed_plan_ = std::move(plan);
  executed_sibling_plan_ = std::move(sibling_plan);
  latest_executed_sequence_ = sequence;
  latest_execution_decision_id_ = publication_decision_id;
  if (publication_decision_id >= latest_published_bundle_decision_id_) {
    published_bundle_source_plan_.reset();
    published_bundle_source_sibling_plan_.reset();
    latest_published_bundle_decision_id_ = 0U;
    published_bundle_control_origin_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    published_bundle_artifact_elapsed_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  }
  if (!same_executed_identity) {
    first_published_control_origin_sec_ = publication_control_origin_sec;
    first_published_artifact_elapsed_sec_ =
      publication_artifact_elapsed_sec;
  }
  ++executed_count_;
  last_reason_ = StoreReason::Accepted;
  return StoreReason::Accepted;
}

StoreReason Store::record_published_bundle_source(
  std::shared_ptr<const CertifiedPlan> plan,
  const std::uint64_t publication_decision_id,
  const double publication_control_origin_sec,
  const double publication_artifact_elapsed_sec)
{
  return record_published_bundle_source(
    std::move(plan), nullptr, publication_decision_id,
    publication_control_origin_sec, publication_artifact_elapsed_sec);
}

StoreReason Store::record_published_bundle_source(
  std::shared_ptr<const CertifiedPlan> plan,
  std::shared_ptr<const CertifiedPlan> sibling_plan,
  const std::uint64_t publication_decision_id,
  const double publication_control_origin_sec,
  const double publication_artifact_elapsed_sec)
{
  if (
    plan == nullptr || validate(*plan) != RejectReason::None ||
    (sibling_plan != nullptr &&
    !sibling_pair_valid(plan, sibling_plan)) ||
    publication_decision_id == 0U ||
    !std::isfinite(publication_control_origin_sec) ||
    publication_control_origin_sec < 0.0 ||
    !std::isfinite(publication_artifact_elapsed_sec) ||
    publication_artifact_elapsed_sec < 0.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = StoreReason::InvalidPlan;
    return StoreReason::InvalidPlan;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (sibling_plan == nullptr) {
    sibling_plan = associated_sibling(
      plan, candidate_plan_, candidate_sibling_plan_);
  }
  const bool same_source =
    published_bundle_source_plan_ != nullptr &&
    artifact::same_identity(
    published_bundle_source_plan_->execution_artifact->identity,
    plan->execution_artifact->identity);
  if (sibling_plan == nullptr && same_source) {
    sibling_plan = published_bundle_source_sibling_plan_;
  }
  if (
    publication_decision_id < latest_execution_decision_id_ ||
    publication_decision_id < latest_published_bundle_decision_id_)
  {
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }
  if (publication_decision_id == latest_published_bundle_decision_id_) {
    if (
      same_source &&
      std::abs(
        published_bundle_control_origin_sec_ -
        publication_control_origin_sec) <= 1e-9 &&
      std::abs(
        published_bundle_artifact_elapsed_sec_ -
        publication_artifact_elapsed_sec) <= 1e-9)
    {
      last_reason_ = StoreReason::Accepted;
      return StoreReason::Accepted;
    }
    ++stale_sequence_count_;
    last_reason_ = StoreReason::StaleSequence;
    return StoreReason::StaleSequence;
  }

  published_bundle_source_plan_ = std::move(plan);
  published_bundle_source_sibling_plan_ = std::move(sibling_plan);
  latest_published_bundle_decision_id_ = publication_decision_id;
  published_bundle_control_origin_sec_ = publication_control_origin_sec;
  published_bundle_artifact_elapsed_sec_ =
    publication_artifact_elapsed_sec;
  last_reason_ = StoreReason::Accepted;
  return StoreReason::Accepted;
}

void Store::supersede_published_bundle_source(
  const std::uint64_t publication_decision_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (publication_decision_id < latest_published_bundle_decision_id_) {
    return;
  }
  published_bundle_source_plan_.reset();
  published_bundle_source_sibling_plan_.reset();
  latest_published_bundle_decision_id_ = 0U;
  published_bundle_control_origin_sec_ =
    std::numeric_limits<double>::quiet_NaN();
  published_bundle_artifact_elapsed_sec_ =
    std::numeric_limits<double>::quiet_NaN();
}

std::shared_ptr<const CertifiedPlan> Store::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return executed_plan_;
}

ExecutedPlanSnapshot Store::executed_snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ExecutedPlanSnapshot{
    executed_plan_, executed_sibling_plan_, first_published_control_origin_sec_,
    first_published_artifact_elapsed_sec_};
}

PublishedBundleSourceSnapshot Store::published_bundle_source_snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return PublishedBundleSourceSnapshot{
    published_bundle_source_plan_, published_bundle_source_sibling_plan_,
    latest_published_bundle_decision_id_,
    published_bundle_control_origin_sec_,
    published_bundle_artifact_elapsed_sec_};
}

StoreState Store::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  StoreState state;
  state.latest_certified_sequence = latest_certified_sequence_;
  state.latest_executed_sequence = latest_executed_sequence_;
  state.latest_execution_decision_id = latest_execution_decision_id_;
  state.accepted_count = accepted_count_;
  state.executed_count = executed_count_;
  state.invalid_plan_count = invalid_plan_count_;
  state.stale_sequence_count = stale_sequence_count_;
  state.certification_reject_count = certification_reject_count_;
  state.last_certification_reason = last_certification_reason_;
  state.last_reason = last_reason_;
  state.candidate_available = static_cast<bool>(candidate_plan_);
  state.executed_plan_available = static_cast<bool>(executed_plan_);
  state.published_bundle_source_available =
    static_cast<bool>(published_bundle_source_plan_);
  state.latest_published_bundle_decision_id =
    latest_published_bundle_decision_id_;
  state.first_published_control_origin_sec =
    first_published_control_origin_sec_;
  state.first_published_artifact_elapsed_sec =
    first_published_artifact_elapsed_sec_;
  return state;
}

bool Store::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool had_plan = static_cast<bool>(executed_plan_) ||
    static_cast<bool>(published_bundle_source_plan_);
  executed_plan_.reset();
  executed_sibling_plan_.reset();
  published_bundle_source_plan_.reset();
  published_bundle_source_sibling_plan_.reset();
  latest_published_bundle_decision_id_ = 0U;
  published_bundle_control_origin_sec_ =
    std::numeric_limits<double>::quiet_NaN();
  published_bundle_artifact_elapsed_sec_ =
    std::numeric_limits<double>::quiet_NaN();
  first_published_control_origin_sec_ =
    std::numeric_limits<double>::quiet_NaN();
  first_published_artifact_elapsed_sec_ =
    std::numeric_limits<double>::quiet_NaN();
  return had_plan;
}

bool Store::clear_if_sequence(const std::uint64_t expected_sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool cleared = false;
  if (
    executed_plan_ != nullptr &&
    executed_plan_->execution_artifact->identity.sequence == expected_sequence)
  {
    executed_plan_.reset();
    executed_sibling_plan_.reset();
    first_published_control_origin_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    first_published_artifact_elapsed_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    cleared = true;
  }
  if (
    published_bundle_source_plan_ != nullptr &&
    published_bundle_source_plan_->execution_artifact->identity.sequence ==
    expected_sequence)
  {
    published_bundle_source_plan_.reset();
    published_bundle_source_sibling_plan_.reset();
    latest_published_bundle_decision_id_ = 0U;
    published_bundle_control_origin_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    published_bundle_artifact_elapsed_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    cleared = true;
  }
  return cleared;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan
