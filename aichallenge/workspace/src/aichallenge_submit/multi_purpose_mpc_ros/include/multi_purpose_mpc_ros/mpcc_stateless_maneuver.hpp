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
  Stop,
};

const char * to_string(TerminalSuccessor successor) noexcept;

/// Non-authoritative description of the Stop problem which must follow the
/// candidate if no Return suffix is visible.  It is intentionally not a
/// trajectory or certificate; the common seven-state solver must still
/// produce and prove the exact suffix before a ManeuverBundle can execute.
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

/// Rebuild one pass-side reference from an immutable current-world snapshot.
/// Persistent Mission geometry, phase state, retained candidates and runtime
/// clocks are not inputs.  The returned snapshot is suitable only for the
/// same shadow SQP and proof pipeline used by the persistent arm.
Result build(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  std::uint64_t source_interaction_fingerprint,
  int pass_side_sign) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_STATELESS_MANEUVER_HPP_
