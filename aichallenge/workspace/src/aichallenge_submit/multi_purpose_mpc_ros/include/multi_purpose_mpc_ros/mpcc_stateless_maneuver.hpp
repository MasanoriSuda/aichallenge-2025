#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_STATELESS_MANEUVER_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_STATELESS_MANEUVER_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
{

enum class RejectReason
{
  Accepted,
  IncompleteSnapshot,
  SourceFingerprintMismatch,
  InvalidSide,
  UnsupportedIntent,
  InvalidTransitionStage,
  DynamicTargetUnavailable,
  LateralIntervalUnavailable,
  TerminalSuccessorUnavailable,
  CandidateSealUnavailable,
};

const char * to_string(RejectReason reason) noexcept;

enum class TerminalSuccessor
{
  None,
  Return,
  Replan,
  Stop,
};

const char * to_string(TerminalSuccessor successor) noexcept;

/// Non-authoritative description of the Stop contingency which remains
/// available after Return or another receding bundle.  It is intentionally
/// not a trajectory or certificate; the common seven-state solver must still
/// produce and prove the exact suffix before it can execute.
struct ContingencyStopIntent
{
  bool available{false};
  double hold_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double target_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double maximum_deceleration_mps2{
    std::numeric_limits<double>::quiet_NaN()};
};

struct TerminalResolution
{
  bool accepted{false};
  TerminalSuccessor successor{TerminalSuccessor::None};
  ContingencyStopIntent stop_suffix;
  std::size_t predicted_encounter_stage_count{};
  std::size_t last_encounter_state{};
  std::string detail{"not-evaluated"};
};

/// Current-world target tube rebuilt without persistent Mission products.
/// Stage predictions use the same immutable ReplayWorld observation consumed
/// by the final exact dynamic proof.
struct TargetHorizon
{
  bool accepted{false};
  std::vector<mpcc_rate_resolved_dynamic_obstacle::StagePrediction> stages;
  std::string detail{"not-evaluated"};
};

TargetHorizon rebuild_target_horizon(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept;

/// Resolve the common terminal successor contract without changing candidate
/// geometry.  Persistent and stateless A/B arms must consume this one rule so
/// successor viability cannot explain an architecture comparison result.
TerminalResolution resolve_terminal_successor(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept;

/// Stateless pre-solve input for the existing seven-state SQP.  This type has
/// no authority surface: it owns data only and cannot publish or retain a
/// command.  Exact trajectory and physical certificates are deliberately
/// absent until the unchanged solver/proof pipeline accepts the candidate.
struct Seed
{
  std::string target_id;
  int pass_side_sign{};
  std::uint64_t source_interaction_fingerprint{};
  std::uint64_t candidate_fingerprint{};
  TerminalSuccessor terminal_successor{TerminalSuccessor::None};
  ContingencyStopIntent stop_suffix;
  std::size_t predicted_encounter_stage_count{};
  std::vector<double> path_distance_m;
  std::vector<double> lateral_reference_m;
  mpcc_rate_resolved_shadow::Snapshot solver_snapshot;
};

struct Result
{
  RejectReason reason{RejectReason::IncompleteSnapshot};
  std::optional<Seed> seed;
  std::string detail{"not-evaluated"};
};

enum class CandidateKind
{
  DirectSide,
  EarliestPhysicalDiagonal,
  LatePhysicalDiagonal,
};

const char * to_string(CandidateKind kind) noexcept;

struct Candidate
{
  CandidateKind kind{CandidateKind::DirectSide};
  Seed seed;
};

/// Bounded production-facing candidate population.  It contains no publisher
/// or retained lifecycle state.  All members are derived from one immutable
/// current-world fingerprint and the population size never exceeds three.
struct CandidateSet
{
  RejectReason reason{RejectReason::IncompleteSnapshot};
  std::uint64_t source_interaction_fingerprint{};
  std::vector<Candidate> candidates;
  std::string detail{"not-evaluated"};
};

/// Rebuild one pass-side reference from an immutable current-world snapshot.
/// Persistent Mission geometry, phase state, retained candidates and runtime
/// clocks are not inputs.  The returned snapshot is suitable only for the
/// same shadow SQP and proof pipeline used by the persistent arm.
Result build(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint,
  int pass_side_sign) noexcept;

/// Data-only normal-intent escape candidate. This deliberately accepts only
/// Follow and has no authority surface. Production Overtake callers continue
/// to use build(), whose intent contract remains unchanged. A candidate needs
/// the common SQP, exact proofs, retained revalidation and publication join
/// before it can affect execution.
Result build_follow_escape(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint,
  int pass_side_sign) noexcept;

/// Audit-only A2 candidate.  Rebuild the current-world target binding through
/// the same stateless producer, then restore the captured identity and SQP
/// request byte-for-byte.  This isolates target ownership from reference and
/// Mission-geometry changes and has no production authority surface.
Result bind_current_world_target_preserving_geometry(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint) noexcept;

/// Shadow-only exact disjunction schedule used by candidate D.  Geometry and
/// costs stay stateless-B-identical; only the complete branch timing and an
/// offline continuation fraction are sealed into the candidate.
Result build_disjunction_schedule(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint,
  int pass_side_sign, int first_pass_side_stage,
  int first_ahead_stage, double constraint_fraction) noexcept;

/// Candidate-C rough lattice member.  It explicitly selects the stage-wise
/// complete obstacle disjunct and shapes a smooth current-world reference;
/// the unchanged seven-state SQP and exact proofs retain final authority.
Result build_lattice(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint,
  int pass_side_sign, int first_pass_side_stage,
  int first_ahead_stage) noexcept;

/// Shadow-only candidate-E member. It keeps the stateless-B reference and
/// seals a monotone diagonal supporting-row schedule into the candidate.
Result build_diagonal_schedule(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint,
  int pass_side_sign, int diagonal_start_stage,
  int full_side_stage) noexcept;

/// Candidate-F member.  It uses the same schedule as candidate E but seals a
/// request to derive every separating row from ReplayWorld body geometry.
Result build_physical_diagonal_schedule(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint,
  int pass_side_sign, int diagonal_start_stage,
  int full_side_stage) noexcept;

/// Build the bounded current-world topology population consumed by the
/// asynchronous production worker.  Candidate zero is the direct side
/// homotopy.  When the prediction horizon permits, candidate one is the
/// earliest complete physical behind-to-side diagonal.  Final authority still
/// requires the unchanged SQP and exact physical proofs.
CandidateSet build_bounded_candidates(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  int pass_side_sign) noexcept;

/// Bounded current-world Follow escape population. It contains at most one
/// direct candidate per side and does not solve, certify, retain or publish.
CandidateSet build_follow_escape_candidates(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_STATELESS_MANEUVER_HPP_
