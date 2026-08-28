#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_COMPARISON_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_COMPARISON_HPP_

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include "multi_purpose_mpc_ros/mpcc_stateless_maneuver.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
{

enum class Arm
{
  PersistentA,
  StatelessLeftB,
  StatelessRightB,
  RoughLeftC,
  RoughRightC,
};

const char * to_string(Arm arm) noexcept;

enum class Stage
{
  Accepted,
  SourceRejected,
  CandidateRejected,
  SolverRejected,
  ExactTrajectoryRejected,
  WallProofRejected,
  DynamicProofRejected,
  TerminalSuccessorRejected,
};

const char * to_string(Stage stage) noexcept;

/// Complete, immutable data result of one independently solved/proved arm.
/// It deliberately has no command conversion, store, mailbox or publisher.
struct ManeuverBundle
{
  std::string target_id;
  int pass_side_sign{};
  std::uint64_t source_interaction_fingerprint{};
  std::uint64_t candidate_fingerprint{};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectory exact_trajectory;
  mpcc_rate_resolved_physical_wall::Result wall_certificate;
  mpcc_rate_resolved_dynamic_proof::Result dynamic_certificate;
  mpcc_stateless_maneuver::TerminalSuccessor terminal_successor{
    mpcc_stateless_maneuver::TerminalSuccessor::None};
  mpcc_stateless_maneuver::ContingencyStopIntent stop_suffix;
};

struct ArmResult
{
  Arm arm{Arm::PersistentA};
  Stage stage{Stage::SourceRejected};
  std::uint64_t source_interaction_fingerprint{};
  std::uint64_t candidate_fingerprint{};
  mpcc_rate_resolved_shadow::Outcome solver_outcome{
    mpcc_rate_resolved_shadow::Outcome::BuildRejected};
  double solver_compute_ms{};
  double terminal_progress_m{};
  double terminal_velocity_mps{};
  double minimum_lateral_bound_reserve_m{};
  int lattice_transition_stage{-1};
  int lattice_ahead_stage{-1};
  std::optional<ManeuverBundle> bundle;
  std::string detail{"not-evaluated"};
};

struct Report
{
  bool source_accepted{false};
  std::uint64_t source_interaction_fingerprint{};
  std::vector<ArmResult> arms;
  std::string detail{"not-evaluated"};
};

/// Evaluate A/B from one sealed current-world snapshot.  Each arm owns a
/// fresh SolverContext, preventing solved iterates from crossing arms.
Report compare(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded)
  noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_comparison

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_COMPARISON_HPP_
