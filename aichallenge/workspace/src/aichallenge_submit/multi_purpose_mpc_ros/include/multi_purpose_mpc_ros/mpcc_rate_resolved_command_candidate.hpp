#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_COMMAND_CANDIDATE_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_COMMAND_CANDIDATE_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_command_candidate
{

namespace contract = mpcc_execution_contract;
namespace retained = mpcc_rate_resolved_retained_revalidation;

enum class Formulation
{
  VelocitySteeringProgress6State,
};

const char * to_string(Formulation formulation) noexcept;

enum class Reason
{
  Available,
  RetainedProofUnavailable,
  InvalidCertifiedPlan,
  InvalidIdentity,
  IdentityMismatch,
  InvalidActuation,
  Count,
};

const char * to_string(Reason reason) noexcept;

/// Complete publisher-shaped proposal from one accepted retained six-state
/// proof.  It deliberately remains a distinct type from CanonicalNormalCommand,
/// whose current contract is five-state-specific.
struct Candidate
{
  std::uint64_t decision_id{};
  std::uint64_t artifact_sequence{};
  std::uint64_t source_decision_id{};
  std::uint64_t source_problem_fingerprint{};
  std::uint64_t stage_geometry_id{};
  contract::ControlIntent intent{contract::ControlIntent::Unknown};
  Formulation formulation{Formulation::VelocitySteeringProgress6State};
  std::size_t control_stage_index{};
  double prediction_origin_sec{};
  double predicted_speed_mps{};
  double acceleration_mps2{};
  double steering_rate_radps{};
  double steering_rad{};
  double curvature_radpm{};
  double virtual_progress_speed_mps{};
};

struct Result
{
  Reason reason{Reason::RetainedProofUnavailable};
  std::optional<Candidate> candidate;
};

/// Convert accepted retained evidence without modifying or re-sampling it.
/// This function cannot publish or claim production authority.
Result build(const retained::Result & retained_result) noexcept;

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_command_candidate

#endif // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_COMMAND_CANDIDATE_HPP_
