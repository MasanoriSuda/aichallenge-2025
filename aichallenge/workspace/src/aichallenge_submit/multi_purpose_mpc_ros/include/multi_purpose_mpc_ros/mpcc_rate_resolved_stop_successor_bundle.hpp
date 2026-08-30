#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_SUCCESSOR_BUNDLE_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_SUCCESSOR_BUNDLE_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <cstdint>
#include <memory>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_successor_bundle
{

namespace certified = mpcc_rate_resolved_certified_plan;
namespace retained = mpcc_rate_resolved_retained_revalidation;

enum class Reason
{
  Available,
  StopSuccessorUnavailable,
  InvalidSource,
  UnsupportedTerminalIntent,
  InvalidIdentity,
  InvalidActuationSequence,
  InvalidExecutionArtifact,
  PhysicalWallRejected,
  CertifiedPlanRejected,
  Count,
};

const char * to_string(Reason reason) noexcept;

struct Result
{
  Reason reason{Reason::StopSuccessorUnavailable};
  std::shared_ptr<const certified::CertifiedPlan> plan;
};

/// Seal one accepted current-world Stop rollout as a new immutable seven-state
/// execution plan.  This function has no Store or publisher access: only the
/// existing canonical normal boundary may grant authority to the result.
Result build(
  const retained::Request & request,
  const retained::StopSuccessorResult & stop_successor,
  std::uint64_t artifact_sequence);

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_successor_bundle

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_SUCCESSOR_BUNDLE_HPP_
