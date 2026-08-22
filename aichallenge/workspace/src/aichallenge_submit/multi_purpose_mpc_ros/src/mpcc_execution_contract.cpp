#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <type_traits>

namespace multi_purpose_mpc_ros::mpcc_execution_contract
{
namespace
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

class FingerprintBuilder
{
public:
  void append_byte(const std::uint8_t value) noexcept
  {
    value_ ^= value;
    value_ *= kFnvPrime;
  }

  void append_u64(const std::uint64_t value) noexcept
  {
    for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
      append_byte(static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
  }

  void append_i64(const std::int64_t value) noexcept
  {
    append_u64(static_cast<std::uint64_t>(value));
  }

  void append_bool(const bool value) noexcept
  {
    append_byte(value ? 1U : 0U);
  }

  void append_double(double value) noexcept
  {
    if (value == 0.0) {
      value = 0.0;
    }
    std::uint64_t bits{};
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    append_u64(bits);
  }

  void append_string(const std::string & value) noexcept
  {
    append_u64(static_cast<std::uint64_t>(value.size()));
    for (const unsigned char character : value) {
      append_byte(character);
    }
  }

  template<typename EnumT>
  void append_enum(const EnumT value) noexcept
  {
    static_assert(std::is_enum_v<EnumT>);
    append_i64(static_cast<std::int64_t>(value));
  }

  std::uint64_t finish() const noexcept
  {
    return value_ == 0U ? 1U : value_;
  }

private:
  std::uint64_t value_{kFnvOffset};
};

bool schemas_complete(const MpccProblemContext & context) noexcept
{
  return
    !context.state_schema_id.empty() && !context.input_schema_id.empty() &&
    !context.bounds_schema_id.empty() && !context.cost_schema_id.empty();
}

std::string fingerprint_text(const std::uint64_t value)
{
  if (value == 0U) {
    return "none";
  }
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setw(16) << std::setfill('0') << value;
  return stream.str();
}

}  // namespace

const char * to_string(const ControlIntent intent) noexcept
{
  switch (intent) {
    case ControlIntent::Unknown: return "unknown";
    case ControlIntent::Track: return "track";
    case ControlIntent::Cruise: return "cruise";
    case ControlIntent::Follow: return "follow";
    case ControlIntent::Hold: return "hold";
    case ControlIntent::Stop: return "stop";
    case ControlIntent::ShiftOut: return "shiftout";
    case ControlIntent::Pass: return "pass";
    case ControlIntent::Return: return "return";
    case ControlIntent::Rejoin: return "rejoin";
  }
  return "unknown";
}

const char * to_string(const Formulation formulation) noexcept
{
  switch (formulation) {
    case Formulation::Unresolved: return "unresolved";
    case Formulation::LegacySpatialMpc3State: return "legacy-spatial-mpc-3state";
    case Formulation::ProgressContouring3State: return "progress-contouring-3state";
    case Formulation::VelocityProgress5State: return "velocity-progress-5state";
    case Formulation::LowSpeedDirect: return "low-speed-direct";
    case Formulation::SolverDerivedBypass: return "solver-derived-bypass";
  }
  return "unknown";
}

std::uint64_t fingerprint_stage_geometry(
  const int tracking_waypoint, const bool circular,
  const std::vector<StageGeometryIdentity> & stages) noexcept
{
  if (stages.empty()) {
    return 0U;
  }
  FingerprintBuilder builder;
  builder.append_string("mpcc-stage-geometry-v1");
  builder.append_i64(tracking_waypoint);
  builder.append_bool(circular);
  builder.append_u64(static_cast<std::uint64_t>(stages.size()));
  for (const auto & stage : stages) {
    if (
      !std::isfinite(stage.transition_distance_m) ||
      !std::isfinite(stage.cumulative_distance_m))
    {
      return 0U;
    }
    builder.append_i64(stage.transition_from_waypoint);
    builder.append_i64(stage.state_waypoint);
    builder.append_double(stage.transition_distance_m);
    builder.append_double(stage.cumulative_distance_m);
  }
  return builder.finish();
}

std::uint64_t problem_context_fingerprint(
  const MpccProblemContext & context) noexcept
{
  FingerprintBuilder builder;
  builder.append_string("mpcc-problem-context-v1");
  builder.append_u64(context.decision_id);
  builder.append_enum(context.intent);
  builder.append_u64(context.intent_generation);
  builder.append_u64(context.observation_generation);
  builder.append_u64(context.stage_geometry_id);
  builder.append_u64(context.target_obstacle_generation);
  builder.append_string(context.target_id);
  builder.append_u64(static_cast<std::uint64_t>(context.horizon_steps));
  builder.append_enum(context.formulation);
  builder.append_string(context.state_schema_id);
  builder.append_string(context.input_schema_id);
  builder.append_string(context.bounds_schema_id);
  builder.append_string(context.cost_schema_id);
  return builder.finish();
}

MpccProblemContext seal_problem_context(MpccProblemContext context) noexcept
{
  context.fingerprint = problem_context_fingerprint(context);
  return context;
}

bool problem_context_complete(const MpccProblemContext & context) noexcept
{
  const bool target_generation_complete =
    context.target_id.empty() ||
    (context.observation_generation > 0U && context.target_obstacle_generation > 0U);
  return
    context.decision_id > 0U && context.intent != ControlIntent::Unknown &&
    context.stage_geometry_id > 0U && context.horizon_steps > 0U &&
    context.formulation != Formulation::Unresolved && schemas_complete(context) &&
    target_generation_complete && context.fingerprint > 0U &&
    context.fingerprint == problem_context_fingerprint(context);
}

bool solution_certified(const CertifiedMpccSolution & solution) noexcept
{
  return
    solution.solution_id > 0U && solution.problem_fingerprint > 0U &&
    solution.formulation != Formulation::Unresolved && solution.solved &&
    solution.finite && solution.constraints_satisfied &&
    std::isfinite(solution.maximum_constraint_violation) &&
    solution.maximum_constraint_violation >= 0.0 &&
    solution.physical.checked && solution.physical.wall_clear &&
    solution.physical.obstacles_clear && solution.prediction_stage_count > 0U &&
    std::isfinite(solution.valid_until_sec);
}

const char * to_string(const FinalAuthorityClass authority) noexcept
{
  switch (authority) {
    case FinalAuthorityClass::CertifiedNormalSolution:
      return "certified-normal-solution";
    case FinalAuthorityClass::LegacyNormalBypass:
      return "legacy-normal-bypass";
    case FinalAuthorityClass::EmergencyOverride:
      return "emergency-override";
    case FinalAuthorityClass::RecoveryOverride:
      return "recovery-override";
    case FinalAuthorityClass::ControlDisabled:
      return "control-disabled";
  }
  return "unknown";
}

FinalControlDecision resolve_final_control_decision(
  const FinalControlDecisionRequest & request) noexcept
{
  FinalControlDecision decision;
  decision.decision_id = request.decision_id;
  decision.authority = request.authority;
  decision.source = request.source;
  decision.retained_solution = request.retained_solution;
  if (request.problem.has_value() && problem_context_complete(request.problem.value())) {
    decision.intent = request.problem->intent;
    decision.formulation = request.problem->formulation;
    decision.problem_fingerprint = request.problem->fingerprint;
  }
  if (request.solution.has_value()) {
    decision.solution_id = request.solution->solution_id;
  }

  if (request.decision_id == 0U) {
    decision.reason = "missing-decision-id";
    return decision;
  }
  if (request.source.empty()) {
    decision.reason = "missing-control-source";
    return decision;
  }
  if (
    request.authority == FinalAuthorityClass::EmergencyOverride ||
    request.authority == FinalAuthorityClass::RecoveryOverride ||
    request.authority == FinalAuthorityClass::ControlDisabled)
  {
    decision.identity_complete = true;
    decision.canonical_contract_satisfied = true;
    decision.reason = "explicit-supervisor-override";
    return decision;
  }
  if (request.authority == FinalAuthorityClass::LegacyNormalBypass) {
    decision.identity_complete = true;
    decision.reason = "legacy-normal-bypass";
    return decision;
  }
  if (!request.problem.has_value()) {
    decision.reason = "missing-problem-context";
    return decision;
  }
  if (!problem_context_complete(request.problem.value())) {
    decision.reason = "incomplete-problem-context";
    return decision;
  }
  if (
    !request.retained_solution &&
    request.problem->decision_id != request.decision_id)
  {
    decision.reason = "decision-problem-id-mismatch";
    return decision;
  }
  if (!request.solution.has_value()) {
    decision.reason = "missing-certified-solution";
    return decision;
  }
  if (!solution_certified(request.solution.value())) {
    decision.reason = "solution-not-certified";
    return decision;
  }
  if (request.solution->problem_fingerprint != request.problem->fingerprint) {
    decision.reason = "problem-solution-fingerprint-mismatch";
    return decision;
  }
  if (request.solution->formulation != request.problem->formulation) {
    decision.reason = "problem-solution-formulation-mismatch";
    return decision;
  }
  decision.identity_complete = true;
  decision.canonical_contract_satisfied =
    request.problem->formulation == Formulation::VelocityProgress5State;
  decision.problem_fingerprint = request.problem->fingerprint;
  decision.solution_id = request.solution->solution_id;
  decision.reason = decision.canonical_contract_satisfied ?
    "matching-certified-solution" :
    "matching-certified-noncanonical-formulation";
  return decision;
}

std::string format_final_control_decision(
  const FinalControlDecision & decision)
{
  std::ostringstream stream;
  stream << "MPCC execution contract: decision=" << decision.decision_id
         << ", authority=" << to_string(decision.authority)
         << ", source=" << (decision.source.empty() ? "<none>" : decision.source)
         << ", intent=" << to_string(decision.intent)
         << ", formulation=" << to_string(decision.formulation)
         << ", problem=" << fingerprint_text(decision.problem_fingerprint)
         << ", solution=" << decision.solution_id
         << ", retained=" << (decision.retained_solution ? 1 : 0)
         << ", identity=" << (decision.identity_complete ? "complete" : "incomplete")
         << ", canonical="
         << (decision.canonical_contract_satisfied ? "satisfied" : "violated")
         << ", reason=" << decision.reason;
  return stream.str();
}

}  // namespace multi_purpose_mpc_ros::mpcc_execution_contract
