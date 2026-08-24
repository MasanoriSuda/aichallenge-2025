#ifndef MULTI_PURPOSE_MPC_ROS__CANONICAL_EXECUTION_PLAN_ADAPTER_HPP_
#define MULTI_PURPOSE_MPC_ROS__CANONICAL_EXECUTION_PLAN_ADAPTER_HPP_

#include "multi_purpose_mpc_ros/canonical_execution_plan.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::canonical_execution_plan_adapter
{

namespace contract = mpcc_execution_contract;
namespace plan_contract = canonical_execution_plan;

struct CanonicalPlanExtractionRequest
{
  std::uint64_t plan_id{};
  contract::MpccProblemContext problem;
  contract::CertifiedMpccSolution solution;
  double solved_sec{};
  double progress_origin_m{};
  std::vector<double> stage_duration_sec;
  std::vector<double> lateral_lower_m;
  std::vector<double> lateral_upper_m;
  double required_lateral_tracking_reserve_m{};
  Eigen::VectorXd extended_primal;
};

enum class CanonicalPlanExtractionReason
{
  Accepted,
  InvalidMetadata,
  InvalidProgressOrigin,
  StageDurationCountMismatch,
  CorridorCountMismatch,
  PrimalSizeMismatch,
  NonfinitePrimal,
  PlanContractRejected,
};

const char * to_string(CanonicalPlanExtractionReason reason) noexcept;

struct CanonicalPlanExtractionResult
{
  CanonicalPlanExtractionReason reason{
    CanonicalPlanExtractionReason::InvalidMetadata};
  plan_contract::CanonicalExecutionPlanRejectReason plan_reject_reason{
    plan_contract::CanonicalExecutionPlanRejectReason::None};
  std::optional<plan_contract::CanonicalExecutionPlan> plan;
};

CanonicalPlanExtractionResult extract_canonical_execution_plan(
  const CanonicalPlanExtractionRequest & request);

enum class FreshCanonicalCommandReason
{
  Accepted,
  PlanRejected,
  CursorRejected,
  CandidateRejected,
  AuthorityRejected,
  ActuationRejected,
  ActuationMismatch,
  CommandRejected,
};

const char * to_string(FreshCanonicalCommandReason reason) noexcept;

/// Build a complete fresh-only canonical command without retaining or
/// publishing it.  The direct actuation is extracted independently from the
/// same normalized primal by the caller; equality here proves that plan
/// storage and cursor resolution did not mutate the first executable control.
struct FreshCanonicalCommandRequest
{
  CanonicalPlanExtractionRequest extraction;
  std::uint64_t current_decision_id{};
  double now_sec{};
  contract::ControlIntent current_intent{contract::ControlIntent::Unknown};
  double wheelbase_m{};
  contract::CanonicalActuation direct_actuation;
  double actuation_tolerance{};
};

struct FreshCanonicalCommandResult
{
  bool plan_extracted{false};
  bool cursor_available{false};
  bool candidate_accepted{false};
  bool authority_ready{false};
  bool actuation_extracted{false};
  FreshCanonicalCommandReason reason{FreshCanonicalCommandReason::PlanRejected};
  std::uint64_t plan_id{};
  CanonicalPlanExtractionReason extraction_reason{
    CanonicalPlanExtractionReason::InvalidMetadata};
  plan_contract::CanonicalExecutionPlanRejectReason plan_reject_reason{
    plan_contract::CanonicalExecutionPlanRejectReason::None};
  plan_contract::CanonicalExecutionCursorReason cursor_reason{
    plan_contract::CanonicalExecutionCursorReason::InvalidPlan};
  plan_contract::CanonicalCandidateBuildReason candidate_reason{
    plan_contract::CanonicalCandidateBuildReason::InvalidPlan};
  contract::CanonicalNormalAuthoritySource authority_source{
    contract::CanonicalNormalAuthoritySource::EmergencyStop};
  contract::CanonicalNormalAuthorityReason authority_reason{
    contract::CanonicalNormalAuthorityReason::NoCanonicalCandidate};
  contract::CanonicalNormalCandidateRejectReason fresh_reject_reason{
    contract::CanonicalNormalCandidateRejectReason::NotEvaluated};
  plan_contract::CanonicalActuationReason actuation_reason{
    plan_contract::CanonicalActuationReason::InvalidPlan};
  contract::CanonicalNormalCommandReason command_reason{
    contract::CanonicalNormalCommandReason::IncompleteAuthorityIdentity};
  double maximum_actuation_difference{};
  std::optional<plan_contract::CanonicalExecutionPlan> plan;
  plan_contract::CanonicalExecutionCursor cursor;
  std::optional<contract::CanonicalNormalCommand> command;
};

FreshCanonicalCommandResult build_fresh_canonical_command(
  const FreshCanonicalCommandRequest & request);

}  // namespace multi_purpose_mpc_ros::canonical_execution_plan_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__CANONICAL_EXECUTION_PLAN_ADAPTER_HPP_
