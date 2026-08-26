#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PRODUCTION_ADAPTER_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PRODUCTION_ADAPTER_HPP_

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_production_adapter
{

namespace contract = mpcc_execution_contract;
namespace retained = mpcc_rate_resolved_retained_revalidation;

enum class Reason
{
  Available,
  RetainedProofUnavailable,
  InvalidPlan,
  InvalidIdentity,
  InvalidPhysicalEvidence,
  InvalidCursor,
  InvalidActuation,
  AuthorityRejected,
  CommandRejected,
  PredictionRejected,
  Count,
};

const char * to_string(Reason reason) noexcept;

/// Complete seven-state normal authority ready for the existing final publisher.
/// This adapter can only transform already accepted evidence.  It may project
/// a lower-bound solver residual to the exact physical boundary when that
/// residual is covered by the sealed certificate; it cannot solve, weaken a
/// certificate or select another formulation.
struct Authority
{
  contract::MpccProblemContext problem;
  contract::CertifiedMpccSolution solution;
  contract::CanonicalNormalCommand command;
  std::size_t first_control_stage_index{};
  std::vector<double> target_speed_horizon_mps;
  std::vector<double> steering_horizon_rad;
  std::pair<std::vector<double>, std::vector<double>> world_prediction;
  double maximum_abs_steering_rad{};
};

struct Result
{
  Reason reason{Reason::RetainedProofUnavailable};
  std::optional<Authority> authority;
};

Result build(const retained::Result & retained_result) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_production_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PRODUCTION_ADAPTER_HPP_
