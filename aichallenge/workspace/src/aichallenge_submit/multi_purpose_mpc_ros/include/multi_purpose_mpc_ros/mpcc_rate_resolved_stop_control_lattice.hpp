#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_CONTROL_LATTICE_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_CONTROL_LATTICE_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_control_lattice
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace shadow = mpcc_rate_resolved_shadow;

enum class Reason
{
  Accepted,
  InvalidSource,
  PublisherBoundaryUnavailable,
  ReplayWorldUnavailable,
  InvalidBrakingEnvelope,
  InvalidSchedule,
  Count,
};

const char * to_string(Reason reason) noexcept;

struct StopCandidateResult
{
  Reason reason{Reason::InvalidSource};
  shadow::Snapshot candidate;
  std::string detail{"not-evaluated"};

  bool accepted() const noexcept {return reason == Reason::Accepted;}
};

/// Rebase one immutable seven-state world snapshot after the exact publisher
/// interval of `normal_execution` and impose the solver-safe maximum-braking
/// velocity law.  This performs no solve, proof, Store mutation or publish.
StopCandidateResult build_maximum_braking_candidate(
  const shadow::Snapshot & source,
  const artifact::ExecutionArtifact & normal_execution,
  const persistent_osqp::PhysicalConstraintTolerance
  & solver_tolerance) noexcept;

struct Schedule
{
  int initial_rate_sign{};
  int first_switch_stage{};
  int second_switch_stage{};
  std::vector<double> steering_rate_radps;
};

struct ScheduleResult
{
  Reason reason{Reason::InvalidSchedule};
  Schedule schedule;
  std::string detail{"not-evaluated"};

  bool accepted() const noexcept {return reason == Reason::Accepted;}
};

/// Build one positive/negative/hold or negative/positive/hold steering-rate
/// command under the exact actuator and solver-inset bounds of the Stop
/// snapshot.  The returned control is deterministic and remains unproved.
ScheduleResult build_schedule(
  const shadow::Snapshot & maximum_braking_stop,
  int initial_rate_sign, int first_switch_stage,
  int second_switch_stage,
  const persistent_osqp::PhysicalConstraintTolerance
  & solver_tolerance) noexcept;

/// Deterministic horizon-relative schedule grid used by audit and live shadow.
std::vector<ScheduleResult> build_population(
  const shadow::Snapshot & maximum_braking_stop,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance);

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_control_lattice

#endif // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_CONTROL_LATTICE_HPP_
