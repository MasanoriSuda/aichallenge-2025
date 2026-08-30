#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_ADAPTER_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_ADAPTER_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/race_mpcc_foundation.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

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
  double rejected_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double rejected_lateral_lower_m{std::numeric_limits<double>::quiet_NaN()};
  double rejected_lateral_upper_m{std::numeric_limits<double>::quiet_NaN()};
  int minimum_progress_transition_state{-1};
  double minimum_progress_delta_m{
    std::numeric_limits<double>::infinity()};
  double transition_virtual_progress_speed_mps{
    std::numeric_limits<double>::quiet_NaN()};
  double transition_duration_sec{
    std::numeric_limits<double>::quiet_NaN()};
  double progress_dynamics_defect_m{
    std::numeric_limits<double>::quiet_NaN()};
  double certified_progress_regression_tolerance_m{};
  std::optional<race_mpcc_foundation::ExactPhysicalExecutionTrajectory>
  exact_trajectory;
};

/// Convert one immutable seven-state solve into the established exact physical
/// pose contract.  State zero describes the source observation; the physical
/// horizon is states 1..N and is later swept from the current measured pose.
Result build(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  mpcc_execution_contract::ControlIntent current_intent,
  std::uint64_t current_stage_geometry_id) noexcept;

/// Current physical state at the latency-compensated control origin.  The
/// progress coordinate stays on the immutable artifact axis while lag records
/// the current vehicle displacement from that axis.
struct ContinuationInitialState
{
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
  double velocity_mps{};
  double progress_m{};
  double steering_rad{};
  double response_steering_rad{};
};

enum class ContinuationRejectReason
{
  None,
  InvalidArtifact,
  InvalidCursor,
  InvalidInitialState,
  InitialLateralBoundRejected,
  NonlinearModelRejected,
  ActuatorEnvelopeRejected,
  ExactTrajectoryRejected,
  Count,
};

const char * to_string(ContinuationRejectReason reason) noexcept;

enum class ContinuationProofScope
{
  FullSuffix,
  PublisherIntervalPrefix,
};

const char * to_string(ContinuationProofScope scope) noexcept;

struct ContinuationResult
{
  ContinuationRejectReason reason{
    ContinuationRejectReason::InvalidArtifact};
  int rejected_stage{-1};
  ContinuationProofScope scope{ContinuationProofScope::FullSuffix};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason exact_reason{
    race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason::Accepted};
  std::optional<race_mpcc_foundation::ExactPhysicalExecutionTrajectory>
  exact_trajectory;
  /// Nonlinear stage-end states on the shortened suffix. For a publisher
  /// interval prefix these contain exactly the state at the publication
  /// boundary, independent of solver-stage duration. These are kept
  /// beside the dense physical trajectory so a production adapter never
  /// rebuilds a different command/speed horizon from the old affine states.
  std::vector<double> stage_end_velocity_mps;
  std::vector<double> stage_end_steering_rad;
};

/// Replay the unconsumed control suffix from the current physical state.
/// A retained artifact is immutable control evidence, not immutable state
/// evidence: after publication, the only valid physical proof is the result
/// of applying its remaining inputs to the current control-origin state.
ContinuationResult build_continuation(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const mpcc_rate_resolved_execution_artifact::Cursor & cursor,
  const ContinuationInitialState & initial_state) noexcept;

enum class StopContingencyRejectReason
{
  None,
  InvalidArtifact,
  InvalidCursor,
  InvalidInitialState,
  InvalidActuation,
  InvalidBrakingEnvelope,
  InvalidLateralPolicy,
  CourseGeometryUnavailable,
  NonlinearModelRejected,
  ActuatorEnvelopeRejected,
  ExactTrajectoryRejected,
  Count,
};

const char * to_string(StopContingencyRejectReason reason) noexcept;

/// Complete static course support for a synthesized terminal Stop.  This is
/// deliberately distinct from ExecutionArtifact's short executable prefix:
/// braking may continue after that prefix ends, but it may never extrapolate
/// the prefix's final curvature or tactical lateral corridor.
struct StopCourseGeometry
{
  /// Local theta/progress coordinates relative to the execution artifact's
  /// course_progress_origin_m.  Knots are finite and strictly increasing.
  std::vector<double> progress_m;
  /// Piecewise-constant course curvature for every knot interval.
  std::vector<double> curvature_radpm;
  /// Physical lateral support at every knot.  These are not retained Mission
  /// corridor bounds; exact footprint wall proof remains the final authority.
  std::vector<double> lateral_lower_m;
  std::vector<double> lateral_upper_m;
};

inline bool stop_course_geometry_valid(
  const StopCourseGeometry & geometry) noexcept
{
  if (
    geometry.progress_m.size() < 2U ||
    geometry.curvature_radpm.size() + 1U != geometry.progress_m.size() ||
    geometry.lateral_lower_m.size() != geometry.progress_m.size() ||
    geometry.lateral_upper_m.size() != geometry.progress_m.size())
  {
    return false;
  }
  for (std::size_t knot = 0U; knot < geometry.progress_m.size(); ++knot) {
    if (
      !std::isfinite(geometry.progress_m[knot]) ||
      !std::isfinite(geometry.lateral_lower_m[knot]) ||
      !std::isfinite(geometry.lateral_upper_m[knot]) ||
      geometry.lateral_lower_m[knot] > geometry.lateral_upper_m[knot] ||
      (knot > 0U &&
      geometry.progress_m[knot] <= geometry.progress_m[knot - 1U]))
    {
      return false;
    }
  }
  for (const double curvature_radpm : geometry.curvature_radpm) {
    if (!std::isfinite(curvature_radpm)) {
      return false;
    }
  }
  return true;
}

struct StopContingencyResult
{
  StopContingencyRejectReason reason{
    StopContingencyRejectReason::InvalidArtifact};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason exact_reason{
    race_mpcc_foundation::ExactPhysicalExecutionTrajectoryReason::Accepted};
  int rejected_sample{-1};
  /// Steering state after replaying the already selected publisher command
  /// for one publication interval.  This makes the causal hand-off into the
  /// braking suffix observable and testable.
  double publisher_interval_end_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  /// Final steering reached by the path-tracking maximum-braking suffix.
  double braking_suffix_final_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  struct ActuationSample
  {
    double elapsed_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double duration_sec{std::numeric_limits<double>::quiet_NaN()};
    double acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
    double steering_rate_radps{std::numeric_limits<double>::quiet_NaN()};
    double end_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
    double end_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  };
  /// Input/state samples aligned one-to-one with exact_trajectory.  The
  /// certificate can otherwise prove states which no publisher can reproduce.
  std::vector<ActuationSample> actuation_samples;
  /// Number of leading samples which replay the normal command already chosen
  /// for the next publisher interval.  The braking suffix starts here.
  std::size_t publisher_interval_sample_count{};
  std::optional<race_mpcc_foundation::ExactPhysicalExecutionTrajectory>
  exact_trajectory;
};

/// Prove the causal command sequence which the publisher can actually
/// execute when no next canonical solution arrives: replay the current
/// serialized acceleration and steering-rate command for exactly one
/// publication interval, then execute the same rate-limited path-feedback law
/// as the Emergency Stop publisher while applying the configured physical
/// maximum braking until rest. This is rebuilt from the current physical state
/// and is not retained Mission geometry.
StopContingencyResult build_stop_contingency(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const mpcc_rate_resolved_execution_artifact::Cursor & cursor,
  const mpcc_rate_resolved_execution_artifact::Actuation & current_actuation,
  const ContinuationInitialState & initial_state,
  const StopCourseGeometry & course_geometry,
  const race_mpcc_foundation::StopPathTrackingPolicy & lateral_policy,
  double minimum_acceleration_mps2) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PHYSICAL_ADAPTER_HPP_
