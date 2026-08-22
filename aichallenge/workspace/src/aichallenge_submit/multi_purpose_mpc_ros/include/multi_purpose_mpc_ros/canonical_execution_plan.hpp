#ifndef MULTI_PURPOSE_MPC_ROS__CANONICAL_EXECUTION_PLAN_HPP_
#define MULTI_PURPOSE_MPC_ROS__CANONICAL_EXECUTION_PLAN_HPP_

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::canonical_execution_plan
{

namespace contract = mpcc_execution_contract;

struct CanonicalPredictedState
{
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
  double velocity_mps{};
  double progress_m{};
};

struct CanonicalControlStage
{
  double acceleration_mps2{};
  double curvature_radpm{};
  double virtual_progress_speed_mps{};
  double duration_sec{};
};

struct CanonicalExecutionPlan
{
  std::uint64_t plan_id{};
  contract::MpccProblemContext problem;
  contract::CertifiedMpccSolution solution;
  double solved_sec{};
  std::vector<CanonicalPredictedState> predicted_states;
  std::vector<CanonicalControlStage> control_stages;
};

enum class CanonicalExecutionPlanRejectReason
{
  None,
  MissingPlanId,
  IncompleteProblem,
  UnsupportedIntent,
  NoncanonicalFormulation,
  SolutionIdentityMismatch,
  SolutionNotCertified,
  InvalidSolveTime,
  EmptyHorizon,
  StateCountMismatch,
  ControlCountMismatch,
  InvalidPredictedState,
  InvalidControlStage,
};

const char * to_string(CanonicalExecutionPlanRejectReason reason) noexcept;
CanonicalExecutionPlanRejectReason validate_canonical_execution_plan(
  const CanonicalExecutionPlan & plan) noexcept;

enum class CanonicalExecutionCursorReason
{
  Available,
  InvalidPlan,
  InvalidTime,
  FuturePlan,
  CertificateExpired,
  Exhausted,
};

const char * to_string(CanonicalExecutionCursorReason reason) noexcept;

struct CanonicalExecutionCursor
{
  bool available{false};
  CanonicalExecutionCursorReason reason{
    CanonicalExecutionCursorReason::InvalidPlan};
  std::uint64_t plan_id{};
  std::size_t first_control_stage_index{};
  std::size_t remaining_control_stage_count{};
  double stage_elapsed_sec{};
};

CanonicalExecutionCursor resolve_execution_cursor(
  const CanonicalExecutionPlan & plan, double now_sec) noexcept;

struct CanonicalActuation
{
  std::uint64_t plan_id{};
  std::size_t control_stage_index{};
  double predicted_speed_mps{};
  double acceleration_mps2{};
  double curvature_radpm{};
  double steering_tire_angle_rad{};
  double virtual_progress_speed_mps{};
};

enum class CanonicalActuationReason
{
  Available,
  InvalidPlan,
  CursorUnavailable,
  PlanIdentityMismatch,
  InvalidStageIndex,
  InvalidWheelbase,
  NonfiniteActuation,
};

const char * to_string(CanonicalActuationReason reason) noexcept;

struct CanonicalActuationResult
{
  CanonicalActuationReason reason{CanonicalActuationReason::InvalidPlan};
  std::optional<CanonicalActuation> actuation;
};

CanonicalActuationResult extract_canonical_actuation(
  const CanonicalExecutionPlan & plan,
  const CanonicalExecutionCursor & cursor,
  double wheelbase_m) noexcept;

enum class CanonicalExecutionPlanStoreReason
{
  Accepted,
  InvalidPlan,
  StalePlanId,
};

const char * to_string(CanonicalExecutionPlanStoreReason reason) noexcept;

/// Thread-safe all-or-nothing holder for one immutable executable plan.
/// Failed or stale replacement never modifies the previously accepted plan.
class CanonicalExecutionPlanStore
{
public:
  CanonicalExecutionPlanStore() = default;
  CanonicalExecutionPlanStore(const CanonicalExecutionPlanStore &) = delete;
  CanonicalExecutionPlanStore & operator=(
    const CanonicalExecutionPlanStore &) = delete;

  CanonicalExecutionPlanStoreReason replace(CanonicalExecutionPlan plan);
  std::shared_ptr<const CanonicalExecutionPlan> snapshot() const;
  bool clear_if_plan_id(std::uint64_t expected_plan_id);

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const CanonicalExecutionPlan> plan_;
  std::uint64_t latest_accepted_plan_id_{};
};

struct CanonicalExecutionRevalidation
{
  std::uint64_t decision_id{};
  std::uint64_t plan_id{};
  std::size_t first_control_stage_index{};
  std::size_t remaining_control_stage_count{};
  contract::PhysicalCertificate physical;
};

enum class CanonicalCandidateBuildReason
{
  Accepted,
  InvalidPlan,
  CursorUnavailable,
  PlanIdentityMismatch,
  ExecutionWindowMismatch,
  MissingDecisionIdentity,
  PhysicalCertificateRejected,
};

const char * to_string(CanonicalCandidateBuildReason reason) noexcept;

struct CanonicalCandidateBuildResult
{
  CanonicalCandidateBuildReason reason{
    CanonicalCandidateBuildReason::InvalidPlan};
  std::optional<contract::CanonicalNormalCandidate> candidate;
};

CanonicalCandidateBuildResult build_canonical_normal_candidate(
  const CanonicalExecutionPlan & plan,
  const CanonicalExecutionCursor & cursor,
  const CanonicalExecutionRevalidation & revalidation);

}  // namespace multi_purpose_mpc_ros::canonical_execution_plan

#endif  // MULTI_PURPOSE_MPC_ROS__CANONICAL_EXECUTION_PLAN_HPP_
