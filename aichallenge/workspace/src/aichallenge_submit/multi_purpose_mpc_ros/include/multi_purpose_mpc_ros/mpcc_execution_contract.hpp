#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_EXECUTION_CONTRACT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_EXECUTION_CONTRACT_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_execution_contract
{

enum class ControlIntent
{
  Unknown,
  Track,
  Cruise,
  Follow,
  Hold,
  Stop,
  ShiftOut,
  Pass,
  Return,
  Rejoin,
};

enum class Formulation
{
  Unresolved,
  LegacySpatialMpc3State,
  ProgressContouring3State,
  VelocityProgress5State,
  LowSpeedDirect,
  SolverDerivedBypass,
};

const char * to_string(ControlIntent intent) noexcept;
const char * to_string(Formulation formulation) noexcept;

struct StageGeometryIdentity
{
  int transition_from_waypoint{};
  int state_waypoint{};
  double transition_distance_m{};
  double cumulative_distance_m{};
};

std::uint64_t fingerprint_stage_geometry(
  int tracking_waypoint, bool circular,
  const std::vector<StageGeometryIdentity> & stages) noexcept;

struct MpccProblemContext
{
  std::uint64_t decision_id{};
  ControlIntent intent{ControlIntent::Unknown};
  std::uint64_t intent_generation{};
  std::uint64_t observation_generation{};
  std::uint64_t stage_geometry_id{};
  std::uint64_t target_obstacle_generation{};
  std::string target_id;
  std::size_t horizon_steps{};
  Formulation formulation{Formulation::Unresolved};
  std::string state_schema_id;
  std::string input_schema_id;
  std::string bounds_schema_id;
  std::string cost_schema_id;
  std::uint64_t fingerprint{};
};

std::uint64_t problem_context_fingerprint(
  const MpccProblemContext & context) noexcept;
MpccProblemContext seal_problem_context(MpccProblemContext context) noexcept;
bool problem_context_complete(const MpccProblemContext & context) noexcept;

struct PhysicalCertificate
{
  bool checked{false};
  bool wall_clear{false};
  bool obstacles_clear{false};
  double minimum_wall_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  double minimum_obstacle_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
};

struct CertifiedMpccSolution
{
  std::uint64_t solution_id{};
  std::uint64_t problem_fingerprint{};
  Formulation formulation{Formulation::Unresolved};
  bool solved{false};
  bool finite{false};
  bool constraints_satisfied{false};
  double maximum_constraint_violation{
    std::numeric_limits<double>::quiet_NaN()};
  PhysicalCertificate physical;
  std::size_t prediction_stage_count{};
  double valid_until_sec{-std::numeric_limits<double>::infinity()};
};

bool solution_certified(const CertifiedMpccSolution & solution) noexcept;

enum class FinalAuthorityClass
{
  CertifiedNormalSolution,
  LegacyNormalBypass,
  EmergencyOverride,
  RecoveryOverride,
  ControlDisabled,
};

const char * to_string(FinalAuthorityClass authority) noexcept;

struct FinalControlDecisionRequest
{
  std::uint64_t decision_id{};
  FinalAuthorityClass authority{FinalAuthorityClass::LegacyNormalBypass};
  std::string source;
  std::optional<MpccProblemContext> problem;
  std::optional<CertifiedMpccSolution> solution;
  bool retained_solution{false};
};

struct FinalControlDecision
{
  std::uint64_t decision_id{};
  FinalAuthorityClass authority{FinalAuthorityClass::LegacyNormalBypass};
  std::string source;
  ControlIntent intent{ControlIntent::Unknown};
  Formulation formulation{Formulation::Unresolved};
  std::uint64_t problem_fingerprint{};
  std::uint64_t solution_id{};
  bool retained_solution{false};
  bool identity_complete{false};
  bool canonical_contract_satisfied{false};
  std::string reason{"not-resolved"};
};

FinalControlDecision resolve_final_control_decision(
  const FinalControlDecisionRequest & request) noexcept;
std::string format_final_control_decision(
  const FinalControlDecision & decision);

}  // namespace multi_purpose_mpc_ros::mpcc_execution_contract

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_EXECUTION_CONTRACT_HPP_
