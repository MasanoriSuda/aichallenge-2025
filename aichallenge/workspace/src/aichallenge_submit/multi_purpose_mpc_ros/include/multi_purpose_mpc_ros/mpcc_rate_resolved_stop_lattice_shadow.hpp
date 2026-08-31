#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_LATTICE_SHADOW_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_LATTICE_SHADOW_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_control_lattice.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_lattice_shadow
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace certified = mpcc_rate_resolved_certified_plan;
namespace dynamic = mpcc_rate_resolved_dynamic_proof;
namespace physical = mpcc_rate_resolved_physical_wall;
namespace shadow = mpcc_rate_resolved_shadow;
namespace lattice = mpcc_rate_resolved_stop_control_lattice;

enum class Reason
{
  Accepted,
  InvalidSource,
  CandidateBuildRejected,
  ScheduleRejected,
  SolverRejected,
  ExactTrajectoryRejected,
  StopNotReached,
  WallProofRejected,
  DynamicProofRejected,
  CertifiedPlanRejected,
  Superseded,
  Exception,
  Count,
};

struct EvaluationControl
{
  std::function<bool()> superseded;
};

enum class EvaluationMode
{
  /// Production worker: one bounded seven-state solve from the latest
  /// published normal state. An older control-lattice search must not block
  /// the next current-world observation from reaching the worker.
  DirectSevenStateOnly,
  /// Offline/shadow comparison: retain the broader candidate population so
  /// candidate-generation limits remain observable without owning authority.
  DirectThenControlLattice,
};

const char * to_string(Reason reason) noexcept;

/// Stop artifacts may be re-proved against the current world while the
/// tactical encounter is unchanged. Decision, observation and artifact
/// sequence are deliberately excluded: requiring exact producer identity
/// makes every asynchronous Stop result obsolete before it completes.
bool same_tactical_stop_scope(
  const artifact::Identity & lhs,
  const artifact::Identity & rhs) noexcept;

struct Result
{
  artifact::Identity source_normal_identity;
  Reason reason{Reason::InvalidSource};
  std::size_t attempted_candidate_count{};
  std::size_t population_size{};
  std::size_t selected_legacy_rank{};
  int preferred_initial_rate_sign{1};
  int initial_rate_sign{};
  int first_switch_stage{};
  int second_switch_stage{};
  bool direct_seven_state_attempted{false};
  bool direct_seven_state_accepted{false};
  shadow::Outcome solver_outcome{shadow::Outcome::BuildRejected};
  physical::Outcome wall_outcome{physical::Outcome::InvalidInput};
  bool dynamic_valid{false};
  bool dynamic_clear{false};
  double minimum_dynamic_clearance_m{
    std::numeric_limits<double>::infinity()};
  double minimum_lateral_bound_reserve_m{};
  double selected_solver_ms{};
  double total_compute_ms{};
  std::shared_ptr<const certified::CertifiedPlan> certified_stop_plan;
  std::string detail{"not-evaluated"};

  bool accepted() const noexcept
  {
    return reason == Reason::Accepted && certified_stop_plan != nullptr;
  }
};

/// Evaluate a bounded Stop population from the exact selected normal epoch.
/// The result is observation-only and cannot write a Store or publish.
Result evaluate(
  const shadow::Snapshot & selected_source,
  const artifact::ExecutionArtifact & selected_normal_execution,
  shadow::SolverContext & private_solver_context,
  const EvaluationControl & control = EvaluationControl{},
  EvaluationMode mode = EvaluationMode::DirectThenControlLattice) noexcept;

enum class PublishReason
{
  Accepted,
  InvalidResult,
  DecisionRollback,
};

const char * to_string(PublishReason reason) noexcept;

struct MailboxState
{
  std::uint64_t accepted_count{};
  std::uint64_t invalid_result_count{};
  std::uint64_t decision_rollback_count{};
  /// Global control-cycle chronology. Artifact sequence is producer-local and
  /// cannot order results across ShiftOut, Pass and Gate-A producers.
  std::uint64_t latest_decision_id{};
};

class Mailbox
{
public:
  PublishReason publish(Result result);
  std::optional<Result> latest_after(std::uint64_t decision_id) const;
  MailboxState state() const;

private:
  mutable std::mutex mutex_;
  std::optional<Result> latest_;
  MailboxState state_;
};

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_lattice_shadow

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_STOP_LATTICE_SHADOW_HPP_
