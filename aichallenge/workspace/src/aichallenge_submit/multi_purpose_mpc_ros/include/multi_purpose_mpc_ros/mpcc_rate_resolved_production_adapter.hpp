#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PRODUCTION_ADAPTER_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PRODUCTION_ADAPTER_HPP_

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
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

/// Observation-only causal Stop successor which certified one bounded normal
/// publisher interval.  Production does not select it in this slice; carrying
/// the complete input/state evidence prevents a boolean certificate from
/// becoming detached from the sequence which was actually proved.
struct CertifiedStopSuccessorEvidence
{
  std::uint64_t source_decision_id{};
  std::uint64_t solution_id{};
  std::uint64_t problem_fingerprint{};
  contract::ControlIntent source_intent{contract::ControlIntent::Unknown};
  double control_origin_sec{std::numeric_limits<double>::quiet_NaN()};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectory exact_trajectory;
  std::vector<mpcc_rate_resolved_physical_adapter::StopContingencyResult::
    ActuationSample> actuation_samples;
  std::size_t publisher_interval_sample_count{};
  std::pair<std::vector<double>, std::vector<double>> world_prediction;
  std::vector<double> world_yaw_rad;
};

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
  std::optional<CertifiedStopSuccessorEvidence> certified_stop_successor;
};

struct Result
{
  Reason reason{Reason::RetainedProofUnavailable};
  std::optional<Authority> authority;
};

Result build(const retained::Result & retained_result) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_production_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PRODUCTION_ADAPTER_HPP_
