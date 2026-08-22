#ifndef MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_
#define MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_

#include <multi_purpose_mpc_ros/mpcc_execution_contract.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

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

enum class TargetProvenanceStage
{
  None,
  Observed,
  Locked,
};

const char * target_provenance_stage_name(
  TargetProvenanceStage stage) noexcept;

struct TargetProvenance
{
  bool valid{false};
  std::string target_id;
  double source_stamp_sec{-std::numeric_limits<double>::infinity()};
  double receipt_sec{-std::numeric_limits<double>::infinity()};
  double course_progress_m{std::numeric_limits<double>::quiet_NaN()};
  double course_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t observation_generation{0U};
  TargetProvenanceStage stage{TargetProvenanceStage::None};
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
  StageRegression,
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

enum class TrackCruiseShadowEligibilityReason
{
  Eligible,
  ProgressMpccDisabled,
  MigrationBoundaryInactive,
  ExtendedDynamicsDisabled,
  LiveProgressAlreadyActive,
  TacticalSnapshot,
  IntentNotTrackCruise,
};

const char * track_cruise_shadow_eligibility_reason_name(
  TrackCruiseShadowEligibilityReason reason) noexcept;

struct TrackCruiseShadowEligibilityRequest
{
  bool progress_mpcc_enabled{false};
  bool overtake_only_boundary{true};
  bool extended_dynamics_enabled{false};
  bool live_progress_active{false};
  bool tactical_snapshot{false};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
};

struct TrackCruiseShadowEligibility
{
  bool eligible{false};
  TrackCruiseShadowEligibilityReason reason{
    TrackCruiseShadowEligibilityReason::ProgressMpccDisabled};
};

TrackCruiseShadowEligibility resolve_track_cruise_shadow_eligibility(
  const TrackCruiseShadowEligibilityRequest & request) noexcept;

enum class ShadowWarmStartResetReason
{
  None,
  InitialContext,
  InvalidPreviousContext,
  InvalidCurrentContext,
  IntentChanged,
  FormulationChanged,
  HorizonChanged,
  SchemaChanged,
  StageGeometryDiscontinuous,
};

const char * shadow_warm_start_reset_reason_name(
  ShadowWarmStartResetReason reason) noexcept;

struct ShadowWarmStartIdentity
{
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  mpcc_execution_contract::Formulation formulation{
    mpcc_execution_contract::Formulation::Unresolved};
  std::size_t horizon_steps{};
  std::string state_schema_id;
  std::string input_schema_id;
  std::string bounds_schema_id;
  std::string cost_schema_id;
  std::uint64_t stage_geometry_id{};
  int tracking_waypoint{};
  bool circular{false};
  std::vector<mpcc_execution_contract::StageGeometryIdentity> stages;
};

struct ShadowWarmStartResolution
{
  bool valid{false};
  bool apply_warm_start{false};
  bool reset_context{true};
  ShadowWarmStartResetReason reason{
    ShadowWarmStartResetReason::InvalidCurrentContext};
};

ShadowWarmStartResolution resolve_shadow_warm_start(
  const std::optional<ShadowWarmStartIdentity> & previous,
  const ShadowWarmStartIdentity & current) noexcept;

}  // namespace multi_purpose_mpc_ros::race_mpcc_foundation

#endif  // MULTI_PURPOSE_MPC_ROS__RACE_MPCC_FOUNDATION_HPP_
