#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_ADAPTER_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_ADAPTER_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/race_mpcc_foundation.hpp"

#include <cstdint>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter
{

enum class RejectReason
{
  None,
  InvalidArtifact,
  IntentMismatch,
  StageGeometryMismatch,
  ExactTrajectoryRejected,
  Count,
};

const char * to_string(RejectReason reason) noexcept;

struct Result
{
  RejectReason reason{RejectReason::InvalidArtifact};
  mpcc_rate_resolved_execution_artifact::RejectReason artifact_reason{
    mpcc_rate_resolved_execution_artifact::RejectReason::None};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason exact_reason{
    race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason::Accepted};
  int rejected_stage{-1};
  std::optional<race_mpcc_foundation::ExactPhysicalExecutionTrajectory>
  exact_trajectory;
};

/// Convert one immutable six-state solve into the established exact physical
/// pose contract.  State zero describes the source observation; the physical
/// horizon is states 1..N and is later swept from the current measured pose.
Result build(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  mpcc_execution_contract::ControlIntent current_intent,
  std::uint64_t current_stage_geometry_id) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_ADAPTER_HPP_
