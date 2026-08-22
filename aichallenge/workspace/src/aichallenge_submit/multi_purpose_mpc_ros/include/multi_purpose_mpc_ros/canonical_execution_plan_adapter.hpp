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
  Eigen::VectorXd extended_primal;
};

enum class CanonicalPlanExtractionReason
{
  Accepted,
  InvalidMetadata,
  InvalidProgressOrigin,
  StageDurationCountMismatch,
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

}  // namespace multi_purpose_mpc_ros::canonical_execution_plan_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__CANONICAL_EXECUTION_PLAN_ADAPTER_HPP_
