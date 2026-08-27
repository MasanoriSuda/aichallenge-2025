#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_SNAPSHOT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_SNAPSHOT_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow
{
struct Snapshot;
}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow

namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
{

/// Exact numerical boundary at which the production seven-state pipeline
/// rejected an Overtake problem.  These values identify different convex
/// problems and must not be collapsed into a generic "solver failed" event.
enum class PipelineStage
{
  Initial,
  SuccessiveLinearization,
  WallRefinement,
  DynamicObstacleRefinement,
  PostRefinementLinearization,
  PhysicalProof,
};

const char * to_string(PipelineStage stage) noexcept;

enum class RecordStatus
{
  Written,
  Duplicate,
  NotOvertake,
  InvalidInput,
  IoFailure,
};

const char * to_string(RecordStatus status) noexcept;

struct RecordResult
{
  RecordStatus status{RecordStatus::InvalidInput};
  std::filesystem::path snapshot_file;
  std::string detail;
};

/// Persist the exact QP and immutable semantic/world provenance which were
/// already rejected by production.  This function has no control authority
/// and never changes a problem, warm start or solver setting.  At most one
/// artifact per (intent, pipeline stage, failure outcome) is written by one
/// process.
RecordResult record_failure(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const mpcc_rate_resolved_problem::AssemblyRequest & assembly_request,
  const mpcc_rate_resolved_problem::Problem & problem,
  const std::optional<persistent_osqp::WarmStart> & warm_start,
  const persistent_osqp::SolveOutcome & outcome,
  PipelineStage pipeline_stage,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const std::filesystem::path & output_root =
  std::filesystem::path{"mpcc_architecture_snapshots"}) noexcept;

struct RecordedQp
{
  mpcc_rate_resolved_problem::Problem problem;
  std::optional<persistent_osqp::WarmStart> warm_start;
  std::string intent;
  std::string pipeline_stage;
  std::string failure_outcome;
  std::string failure_detail;
};

/// Load only the exact convex problem required for deterministic solver
/// replay.  Semantic and physical provenance remain in snapshot.yaml for the
/// B/C/D architecture comparison.
std::optional<RecordedQp> load_recorded_qp(
  const std::filesystem::path & snapshot_file,
  std::string * detail = nullptr) noexcept;

struct ReplayResult
{
  bool loaded{false};
  bool warm_start_requested{false};
  bool warm_start_available{false};
  persistent_osqp::SolveOutcome outcome;
  std::string detail;
};

/// Invoke the same solver and immutable row-preconditioning policy used by
/// production, either with the recorded warm start or from a cold state.
ReplayResult replay_recorded_qp(
  const std::filesystem::path & snapshot_file,
  bool use_recorded_warm_start) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_SNAPSHOT_HPP_
