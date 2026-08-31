#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_COMPARISON_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_COMPARISON_HPP_

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include "multi_purpose_mpc_ros/mpcc_stateless_maneuver.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
{

enum class Arm
{
  PersistentA,
  PersistentTargetBoundA2,
  StatelessLeftB,
  StatelessRightB,
  StatelessReturnB,
  RoughLeftC,
  RoughRightC,
  RoughReturnC,
  OfflineLeftD,
  OfflineRightD,
  OfflineReturnD,
  DiagonalLeftE,
  DiagonalRightE,
  PhysicalDiagonalLeftF,
  PhysicalDiagonalRightF,
  ProductionLeftG,
  ProductionRightG,
  ProductionReturnG,
  WallRestorationH,
  ExternalPrimalI,
  WallOmitHeadingJ,
  WallOmitLagK,
  WallOmitPoseN,
  WallOmitPoseDirectO,
  WallProductionP,
  KktEquilibratedQ,
  DynamicSqpPersistentL,
  DynamicSqpProductionLeftL,
  DynamicSqpProductionRightL,
  ProofGuidedProductionLeftM,
  ProofGuidedProductionRightM,
  PersistentDeclaredStopR,
  ProductionLeftDeclaredStopR,
  PersistentStopScanS,
  ProductionLeftStopScanS,
  PersistentNormalPathStopT,
  ProductionLeftNormalPathStopT,
  SevenStateStopU,
  SevenStateStopControlLatticeV,
  FollowStayBehindW,
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
  race_mpcc_foundation::ExactPhysicalExecutionTrajectory
  terminal_stop_trajectory;
  mpcc_rate_resolved_physical_wall::Result terminal_stop_wall_certificate;
  mpcc_rate_resolved_dynamic_proof::Result terminal_stop_dynamic_certificate;
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
  double maximum_external_constraint_violation{};
  double maximum_external_normalized_constraint_violation{};
  int maximum_external_normalized_constraint_row{-1};
  int lattice_transition_stage{-1};
  int lattice_ahead_stage{-1};
  bool direct_final_attempted{false};
  Stage direct_final_stage{Stage::SourceRejected};
  bool continuation_attempted{false};
  std::size_t continuation_solved_step_count{};
  double continuation_compute_ms{};
  std::string candidate_source{"none"};
  std::size_t candidate_count{};
  std::size_t dynamic_sqp_depth{};
  double terminal_stop_target_lateral_m{};
  std::size_t terminal_stop_target_attempt_count{};
  /// Audit-only copy of the exact solved Stop inputs.  No production adapter
  /// consumes these values; they expose whether a bounded control lattice can
  /// represent an independently certified seven-state Stop.
  std::vector<double> solved_control_duration_sec;
  std::vector<double> solved_acceleration_mps2;
  std::vector<double> solved_steering_rate_radps;
  double solved_acceleration_min_mps2{};
  double solved_acceleration_max_mps2{};
  double solved_acceleration_time_mean_mps2{};
  double solved_steering_rate_min_radps{};
  double solved_steering_rate_max_radps{};
  double solved_steering_rate_time_mean_radps{};
  std::size_t solved_steering_rate_sign_change_count{};
  std::size_t solved_acceleration_bound_count{};
  std::size_t solved_steering_rate_bound_count{};
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

/// Replay only the audit-only wall feasibility restoration arm. This avoids
/// enumerating the full A--G lattice when iterating on a frozen wall failure.
Report compare_wall_restoration(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded)
  noexcept;

/// Replay only the two observation-only artificial wall-bucket variants.
/// Physical wall, dynamic-obstacle and terminal-successor proof is identical
/// to production and neither arm has a command/publication path.
Report compare_wall_buckets(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded)
  noexcept;

/// Replay the exact recorded QP once with the explicit physical row contract
/// followed by OSQP's standard internal equilibration, then pass a converged
/// primal through the unchanged exact proof chain. Observation-only.
Report compare_kkt_equilibration(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded)
  noexcept;

/// Compare the normal single-SQP persistent/production candidates with an
/// observation-only bounded outer SQP which refreshes dynamics, physical
/// obstacle supports and wall rows together.  No arm has authority APIs.
Report compare_physical_dynamic_sqp(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded)
  noexcept;

/// For every bounded production candidate, certify depth zero first and then
/// depths one through three. The first certified depth wins; later numerical
/// iterates cannot replace it. Observation-only with no authority API.
Report compare_proof_guided_dynamic_sqp(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded)
  noexcept;

/// Compare the unchanged zero-offset terminal Stop with the candidate's
/// declared pass-side Stop reference. Both arms use the same primary solve,
/// nonlinear model and exact proof chain; this function has no authority API.
Report compare_terminal_stop_lateral_contract(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded)
  noexcept;

enum class ExternalPrimalConstraintPolicy
{
  ExactRecorded,
  OmitWallHeadingBucket,
  OmitWallLagBucket,
  PhysicalNonlinearOracle,
};

/// Certify an independently solved primal against the exact recorded QP and
/// the unchanged physical wall, dynamic-obstacle and terminal-successor
/// proofs. This audit has no solver, store, mailbox, command or publisher.
Report verify_external_primal(
  const mpcc_architecture_snapshot::RecordedInteractionSnapshot & recorded,
  const Eigen::VectorXd & primal,
  ExternalPrimalConstraintPolicy policy =
  ExternalPrimalConstraintPolicy::ExactRecorded) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_comparison

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_ARCHITECTURE_COMPARISON_HPP_
