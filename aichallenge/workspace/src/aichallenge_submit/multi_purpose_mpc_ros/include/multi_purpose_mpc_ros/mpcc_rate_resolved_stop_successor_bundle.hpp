#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_SUCCESSOR_BUNDLE_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_SUCCESSOR_BUNDLE_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
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

enum class ActuationRejectDetail
{
  None,
  ExactTrajectoryShape,
  InitialLateralBounds,
  CommandIndexDiscontinuity,
  CommandChangedWithinInterval,
  InvalidDenseSample,
  InvalidCommandDuration,
  PublicationIntervalNotCovered,
  ProgressRegressed,
  Count,
};

const char * to_string(ActuationRejectDetail detail) noexcept;

struct Result
{
  Reason reason{Reason::StopSuccessorUnavailable};
  ActuationRejectDetail actuation_detail{ActuationRejectDetail::None};
  std::size_t rejected_index{std::numeric_limits<std::size_t>::max()};
  double observed_value{std::numeric_limits<double>::quiet_NaN()};
  double required_bound{std::numeric_limits<double>::quiet_NaN()};
  double certificate_tolerance{std::numeric_limits<double>::quiet_NaN()};
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
