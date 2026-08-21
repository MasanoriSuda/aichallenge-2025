#ifndef MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace multi_purpose_mpc_ros::race_mpcc_foundation
{

enum class Homotopy
{
  Left,
  Right,
  Hold,
  Return,
  None,
};

const char * homotopy_name(Homotopy homotopy) noexcept;

struct TargetProvenance
{
  bool valid{false};
  std::string target_id;
  double source_stamp_sec{-std::numeric_limits<double>::infinity()};
  double receipt_sec{-std::numeric_limits<double>::infinity()};
  double course_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double course_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t observation_generation{0U};
};

enum class TargetProvenanceRejectReason
{
  None,
  InvalidExpected,
  InvalidCurrent,
  TargetMismatch,
  SourceRegression,
  ReceiptRegression,
  GenerationRegression,
  ProgressDelta,
  LateralDelta,
};

const char * target_provenance_reject_reason_name(
  TargetProvenanceRejectReason reason) noexcept;

struct TargetProvenanceValidationRequest
{
  TargetProvenance expected;
  TargetProvenance current;
  bool circular{false};
  double path_length_m{};
  double maximum_backward_progress_m{};
  double maximum_forward_progress_m{};
  double maximum_lateral_change_m{};
};

struct TargetProvenanceValidation
{
  bool valid{false};
  bool same_observation{false};
  double progress_delta_m{std::numeric_limits<double>::quiet_NaN()};
  double lateral_delta_m{std::numeric_limits<double>::quiet_NaN()};
  TargetProvenanceRejectReason reject_reason{
    TargetProvenanceRejectReason::InvalidExpected};
};

TargetProvenanceValidation validate_target_provenance(
  const TargetProvenanceValidationRequest & request) noexcept;

struct ShadowCandidate
{
  Homotopy homotopy{Homotopy::None};
  bool attempted{false};
  bool feasible{false};
  bool warm_start_applied{false};
  bool solver_context_reset{false};
  std::uint64_t solver_context_solve_count{};
  double objective{std::numeric_limits<double>::infinity()};
  double terminal_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double minimum_wall_reserve_m{std::numeric_limits<double>::quiet_NaN()};
  double solve_ms{};
  int iterations{};
  std::string reason{"not-evaluated"};
};

struct ShadowDecision
{
  std::uint64_t context_epoch{};
  std::string target_id;
  TargetProvenance target_provenance{};
  bool stage_geometry_valid{false};
  std::size_t stage_count{};
  double horizon_distance_m{};
  std::array<ShadowCandidate, 4U> candidates{};
  Homotopy selected{Homotopy::None};
  std::string selection_reason{"not-evaluated"};
};

std::string format_shadow_decision(const ShadowDecision & decision);

}  // namespace multi_purpose_mpc_ros::race_mpcc_foundation

#endif  // MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_
