#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_EXECUTION_SOURCE_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_EXECUTION_SOURCE_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source
{

namespace certified = mpcc_rate_resolved_certified_plan;
namespace contract = mpcc_execution_contract;

/// Identity expected by the live Overtake Mission when a certified six-state
/// solve is projected into the rolling lateral-prefix supervisor.  The
/// projection is not a command authority; it only keeps the next six-state
/// problem's execution corridor causally connected to the accepted solve.
struct Request
{
  const certified::CertifiedPlan * plan{};
  contract::ControlIntent intent{contract::ControlIntent::Unknown};
  std::string target_id;
  std::uint64_t mission_generation{};
  int side_sign{};
};

enum class RejectReason
{
  None,
  MissingPlan,
  InvalidCertifiedPlan,
  UnsupportedIntent,
  IntentMismatch,
  TargetMismatch,
  MissionGenerationMismatch,
  SideMismatch,
  InvalidSourceTime,
};

const char * to_string(RejectReason reason) noexcept;

/// Lossless lateral/progress projection of the exact physical trajectory
/// already joined to the immutable six-state artifact.  source_snapshot_sec
/// intentionally remains the original observation time: adopting or
/// revalidating this value can never renew the source age.
struct Source
{
  std::uint64_t artifact_sequence{};
  contract::MpccProblemContext source_context;
  double source_snapshot_sec{};
  double source_completed_sec{};
  double course_progress_origin_m{};
  double minimum_lateral_bound_reserve_m{};
  std::vector<double> path_distance_m;
  std::vector<double> lateral_m;
  std::vector<double> progress_m;
};

struct Result
{
  RejectReason reason{RejectReason::MissingPlan};
  Source source;

  bool accepted() const noexcept
  {
    return reason == RejectReason::None;
  }
};

Result build(const Request & request);

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_EXECUTION_SOURCE_HPP_
