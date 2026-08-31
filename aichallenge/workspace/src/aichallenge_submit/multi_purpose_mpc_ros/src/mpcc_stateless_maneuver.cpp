#include "multi_purpose_mpc_ros/mpcc_stateless_maneuver.hpp"

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
{
namespace
{

constexpr double kNumericalTolerance = 1e-9;

bool supported_intent(
  const mpcc_execution_contract::ControlIntent intent) noexcept
{
  using Intent = mpcc_execution_contract::ControlIntent;
  return intent == Intent::ShiftOut || intent == Intent::Pass ||
         intent == Intent::Return;
}

bool contains_zero(const double lower, const double upper) noexcept
{
  return !std::isnan(lower) && !std::isnan(upper) &&
         lower != std::numeric_limits<double>::infinity() &&
         upper != -std::numeric_limits<double>::infinity() &&
         lower <= kNumericalTolerance && upper >= -kNumericalTolerance;
}

bool valid_interval(const double lower, const double upper) noexcept
{
  return !std::isnan(lower) && !std::isnan(upper) &&
         lower != std::numeric_limits<double>::infinity() &&
         upper != -std::numeric_limits<double>::infinity() && lower <= upper;
}

Result reject(const RejectReason reason, std::string detail)
{
  Result result;
  result.reason = reason;
  result.detail = std::move(detail);
  return result;
}

std::optional<int> steering_reachable_full_side_stage(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const int pass_side_sign, const int first_valid_stage) noexcept
{
  const auto & request = source.request;
  const int horizon = request.horizon_steps;
  if (
    (pass_side_sign != -1 && pass_side_sign != 1) ||
    first_valid_stage < 0 || horizon <= 0 ||
    request.inputs.size() != static_cast<std::size_t>(horizon) ||
    !std::isfinite(request.current_steering_rad) ||
    !std::isfinite(request.maximum_abs_steering_rad) ||
    request.maximum_abs_steering_rad < 0.0 ||
    !std::isfinite(request.maximum_abs_steering_rate_radps) ||
    request.maximum_abs_steering_rate_radps <= kNumericalTolerance ||
    !std::isfinite(request.yaw_response_time_constant_sec) ||
    request.yaw_response_time_constant_sec < 0.0)
  {
    return std::nullopt;
  }

  const double side_steering_limit_rad =
    static_cast<double>(pass_side_sign) *
    request.maximum_abs_steering_rad;
  const double reachability_duration_sec =
    std::abs(side_steering_limit_rad - request.current_steering_rad) /
    request.maximum_abs_steering_rate_radps +
    request.yaw_response_time_constant_sec;
  double cumulative_duration_sec = 0.0;
  for (int full_side_stage = 1; full_side_stage < horizon; ++full_side_stage) {
    const double stage_dt_sec = request.inputs[
      static_cast<std::size_t>(full_side_stage - 1)].stage_dt_sec;
    if (!std::isfinite(stage_dt_sec) || stage_dt_sec <= 0.0) {
      return std::nullopt;
    }
    cumulative_duration_sec += stage_dt_sec;
    if (
      full_side_stage >= first_valid_stage + 2 &&
      cumulative_duration_sec + kNumericalTolerance >=
      reachability_duration_sec)
    {
      return full_side_stage;
    }
  }
  return std::nullopt;
}

}  // namespace

TargetHorizon resolve_canonical_target_horizon(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept
{
  TargetHorizon result;
  const int horizon = source.request.horizon_steps;
  const auto & context = source.identity.source_context;
  if (
    horizon <= 0 || !source.replay_world.has_value() ||
    !source.dynamic_obstacle_refinement_active ||
    !context.dynamic_obstacle_constraint_active ||
    source.dynamic_obstacle_stages.size() !=
    static_cast<std::size_t>(horizon))
  {
    result.detail = "canonical current-epoch target tube unavailable";
    return result;
  }
  const auto & world = source.replay_world.value();
  const std::string & selected_obstacle_id = context.dynamic_obstacle_id;
  const std::uint64_t selected_obstacle_generation =
    context.dynamic_obstacle_generation;
  const auto target = std::find_if(
    world.obstacles.begin(), world.obstacles.end(),
    [&](const auto & obstacle) {
      return obstacle.id == selected_obstacle_id;
    });
  if (
    !world.current || selected_obstacle_id.empty() ||
    selected_obstacle_generation == 0U ||
    world.observation_generation != selected_obstacle_generation ||
    target == world.obstacles.end() || target->observation_generation == 0U ||
    target->observation_generation != selected_obstacle_generation ||
    !std::isfinite(target->x_m) || !std::isfinite(target->y_m) ||
    !std::isfinite(target->velocity_x_mps) ||
    !std::isfinite(target->velocity_y_mps) ||
    !std::isfinite(target->radius_m) || target->radius_m < 0.0)
  {
    result.detail = "canonical target identity does not match ReplayWorld";
    return result;
  }
  bool any_valid = false;
  for (std::size_t stage_index = 0U;
    stage_index < source.dynamic_obstacle_stages.size(); ++stage_index)
  {
    const auto & stage = source.dynamic_obstacle_stages[stage_index];
    if (
      !std::isfinite(stage.target_progress_m) ||
      !std::isfinite(stage.target_lateral_m) ||
      !std::isfinite(stage.longitudinal_overlap_m) ||
      stage.longitudinal_overlap_m <= 0.0 ||
      !std::isfinite(stage.lateral_center_separation_m) ||
      stage.lateral_center_separation_m <= 0.0)
    {
      std::ostringstream detail;
      detail << "invalid canonical target stage " << stage_index;
      result.detail = detail.str();
      return result;
    }
    any_valid = any_valid || stage.valid;
  }
  if (!any_valid) {
    result.detail = "canonical target tube has no valid prediction stage";
    return result;
  }
  result.stages = source.dynamic_obstacle_stages;
  result.accepted = true;
  result.detail = "accepted canonical current-epoch target tube";
  return result;
}

TerminalResolution resolve_terminal_successor(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept
{
  namespace model = mpcc_rate_resolved;
  TerminalResolution resolution;
  const int horizon = source.request.horizon_steps;
  const auto target_horizon = resolve_canonical_target_horizon(source);
  if (
    horizon <= 0 || !target_horizon.accepted ||
    target_horizon.stages.size() != static_cast<std::size_t>(horizon) ||
    source.nominal_path_distance_m.size() !=
    static_cast<std::size_t>(horizon + 1) ||
    source.request.states.size() != static_cast<std::size_t>(horizon + 1) ||
    source.request.inputs.size() != static_cast<std::size_t>(horizon))
  {
    resolution.detail = target_horizon.accepted ?
      "terminal horizon unavailable" : target_horizon.detail;
    return resolution;
  }
  bool target_stage_available = false;
  for (int stage = 1; stage <= horizon; ++stage) {
    const auto & target = target_horizon.stages[
      static_cast<std::size_t>(stage - 1)];
    if (!target.valid) {
      continue;
    }
    target_stage_available = true;
    const double ego_progress_m = source.nominal_path_distance_m[
      static_cast<std::size_t>(stage)];
    const bool encounter =
      std::abs(target.target_progress_m - ego_progress_m) <=
      target.longitudinal_overlap_m + kNumericalTolerance;
    if (encounter) {
      ++resolution.predicted_encounter_stage_count;
      resolution.last_encounter_state = static_cast<std::size_t>(stage);
    }
  }
  if (!target_stage_available) {
    resolution.detail = "no valid target prediction stage";
    return resolution;
  }
  const auto & terminal = source.request.states.back();
  const auto & terminal_target = target_horizon.stages.back();
  const double terminal_ego_progress_m = source.nominal_path_distance_m.back();
  const bool target_rear_clear =
    terminal_ego_progress_m >= terminal_target.target_progress_m +
    terminal_target.longitudinal_overlap_m - kNumericalTolerance;
  const bool return_visible = target_rear_clear &&
    contains_zero(
      terminal.lower[model::kLateralIndex],
      terminal.upper[model::kLateralIndex]);
  double maximum_deceleration_mps2 = 0.0;
  for (const auto & input : source.request.inputs) {
    maximum_deceleration_mps2 = std::min(
      maximum_deceleration_mps2,
      input.lower[model::kAccelerationIndex]);
  }
  const bool stop_available =
    maximum_deceleration_mps2 < -kNumericalTolerance;
  if (!stop_available) {
    resolution.detail = "physical braking authority unavailable";
    return resolution;
  }
  resolution.accepted = true;
  resolution.successor = return_visible ?
    TerminalSuccessor::Return : TerminalSuccessor::Replan;
  resolution.stop_suffix.available = true;
  resolution.stop_suffix.hold_lateral_m =
    terminal.reference[model::kLateralIndex];
  resolution.stop_suffix.target_velocity_mps = 0.0;
  resolution.stop_suffix.maximum_deceleration_mps2 =
    maximum_deceleration_mps2;
  resolution.detail = return_visible ?
    "return-visible-with-stop-contingency" :
    "replan-visible-with-stop-contingency";
  return resolution;
}

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::Accepted: return "accepted";
    case RejectReason::IncompleteSnapshot: return "incomplete-snapshot";
    case RejectReason::SourceFingerprintMismatch:
      return "source-fingerprint-mismatch";
    case RejectReason::InvalidSide: return "invalid-side";
    case RejectReason::UnsupportedIntent: return "unsupported-intent";
    case RejectReason::InvalidTransitionStage:
      return "invalid-transition-stage";
    case RejectReason::DynamicTargetUnavailable:
      return "dynamic-target-unavailable";
    case RejectReason::LateralIntervalUnavailable:
      return "lateral-interval-unavailable";
    case RejectReason::TerminalSuccessorUnavailable:
      return "terminal-successor-unavailable";
    case RejectReason::CandidateSealUnavailable:
      return "candidate-seal-unavailable";
  }
  return "unknown";
}

const char * to_string(const TerminalSuccessor successor) noexcept
{
  switch (successor) {
    case TerminalSuccessor::None: return "none";
    case TerminalSuccessor::Return: return "return";
    case TerminalSuccessor::Replan: return "replan";
    case TerminalSuccessor::Stop: return "stop";
  }
  return "unknown";
}

const char * to_string(const CandidateKind kind) noexcept
{
  switch (kind) {
    case CandidateKind::DirectSide: return "direct-side";
    case CandidateKind::SteeringReachablePhysicalDiagonal:
      return "steering-reachable-physical-diagonal";
    case CandidateKind::MidPhysicalDiagonal:
      return "mid-physical-diagonal";
    case CandidateKind::EncounterBoundaryPhysicalDiagonal:
      return "encounter-boundary-physical-diagonal";
    case CandidateKind::LateExactDisjunction:
      return "late-exact-disjunction";
    case CandidateKind::ReturnRejoin:
      return "return-rejoin";
  }
  return "unknown";
}

static Result build_with_intent_policy(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign, const bool allow_normal_avoidance) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  namespace contract = mpcc_execution_contract;
  namespace model = mpcc_rate_resolved;
  try {
    if (!architecture::interaction_snapshot_complete(source)) {
      return reject(
        RejectReason::IncompleteSnapshot,
        "source is not interaction-replay-ready");
    }
    if (!architecture::interaction_snapshot_matches_fingerprint(
        source, source_interaction_fingerprint))
    {
      return reject(
        RejectReason::SourceFingerprintMismatch,
        "source interaction fingerprint mismatch");
    }
    if (pass_side_sign != -1 && pass_side_sign != 1) {
      return reject(RejectReason::InvalidSide, "pass side must be -1 or +1");
    }
    const bool normal_avoidance =
      allow_normal_avoidance &&
      (source.identity.source_context.intent ==
      mpcc_execution_contract::ControlIntent::Follow ||
      source.identity.source_context.intent ==
      mpcc_execution_contract::ControlIntent::Cruise);
    if (
      !supported_intent(source.identity.source_context.intent) &&
      !normal_avoidance)
    {
      return reject(
        RejectReason::UnsupportedIntent,
        "stateless producer is restricted to Overtake intents");
    }
    const int horizon = source.request.horizon_steps;
    const auto & source_context = source.identity.source_context;
    const std::string & selected_obstacle_id =
      source_context.dynamic_obstacle_constraint_active ?
      source_context.dynamic_obstacle_id : source_context.target_id;
    const std::uint64_t selected_obstacle_generation =
      source_context.dynamic_obstacle_constraint_active ?
      source_context.dynamic_obstacle_generation :
      source_context.target_obstacle_generation;
    const auto target_horizon = resolve_canonical_target_horizon(source);
    if (
      !target_horizon.accepted || target_horizon.stages.size() !=
      static_cast<std::size_t>(horizon) || source.nominal_path_distance_m.size() !=
      static_cast<std::size_t>(horizon + 1))
    {
      return reject(
        RejectReason::DynamicTargetUnavailable,
        target_horizon.detail);
    }

    if (
      source.identity.source_context.intent ==
      mpcc_execution_contract::ControlIntent::Return)
    {
      const auto & contract_endpoint = source.terminal_intent_contract;
      if (!contract_endpoint.active) {
        return reject(
          RejectReason::TerminalSuccessorUnavailable,
          "Return endpoint contract unavailable");
      }
      const auto & first_target = target_horizon.stages.front();
      const double initial_effective_progress_m =
        source.request.initial_state[model::kProgressIndex] +
        source.request.initial_state[model::kLagIndex];
      const double initial_lateral_m =
        source.request.initial_state[model::kLateralIndex];
      int dynamic_side_sign = 0;
      auto longitudinal_topology =
        mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::Automatic;
      if (
        initial_effective_progress_m <=
        first_target.target_progress_m - first_target.longitudinal_overlap_m +
        kNumericalTolerance)
      {
        longitudinal_topology = mpcc_rate_resolved_dynamic_obstacle::
          LongitudinalTopology::StayBehind;
      } else if (
        initial_effective_progress_m >=
        first_target.target_progress_m +
        first_target.longitudinal_overlap_m -
        kNumericalTolerance)
      {
        longitudinal_topology = mpcc_rate_resolved_dynamic_obstacle::
          LongitudinalTopology::StayAhead;
      } else if (
        initial_lateral_m >=
        first_target.target_lateral_m +
        first_target.lateral_center_separation_m -
        kNumericalTolerance)
      {
        dynamic_side_sign = 1;
      } else if (
        initial_lateral_m <=
        first_target.target_lateral_m -
        first_target.lateral_center_separation_m +
        kNumericalTolerance)
      {
        dynamic_side_sign = -1;
      } else {
        return reject(
          RejectReason::DynamicTargetUnavailable,
          "Return relation is neither longitudinally nor laterally "
          "separated");
      }

      Seed seed;
      seed.target_id = source.identity.source_context.target_id;
      seed.pass_side_sign = dynamic_side_sign;
      seed.source_interaction_fingerprint = source_interaction_fingerprint;
      seed.path_distance_m = source.nominal_path_distance_m;
      seed.solver_snapshot = source;
      auto & candidate = seed.solver_snapshot;
      candidate.identity.source_context.execution_side_sign = pass_side_sign;
      candidate.identity.source_context.dynamic_obstacle_constraint_active =
        true;
      candidate.identity.source_context.dynamic_obstacle_id =
        selected_obstacle_id;
      candidate.identity.source_context.dynamic_obstacle_generation =
        selected_obstacle_generation;
      candidate.identity.source_context.dynamic_obstacle_side_sign =
        dynamic_side_sign;
      candidate.identity.source_context.fingerprint = 0U;
      candidate.identity.source_context =
        contract::seal_problem_context(candidate.identity.source_context);
      candidate.dynamic_obstacle_refinement_active = true;
      candidate.dynamic_obstacle_pass_side_sign = dynamic_side_sign;
      candidate.dynamic_obstacle_longitudinal_topology = longitudinal_topology;
      candidate.dynamic_obstacle_forced_first_pass_side_stage = -1;
      candidate.dynamic_obstacle_forced_first_ahead_stage = -1;
      candidate.dynamic_obstacle_forced_constraint_fraction = 1.0;
      candidate.dynamic_obstacle_forced_diagonal_start_stage = -1;
      candidate.dynamic_obstacle_forced_diagonal_full_side_stage = -1;
      candidate.dynamic_obstacle_forced_physical_diagonal = false;
      candidate.dynamic_obstacle_stages = target_horizon.stages;

      seed.lateral_reference_m.reserve(candidate.request.states.size());
      for (const auto & state : candidate.request.states) {
        seed.lateral_reference_m.push_back(
          state.reference[model::kLateralIndex]);
      }
      auto & terminal_state = candidate.request.states.back();
      terminal_state.reference[model::kLateralIndex] =
        contract_endpoint.lateral_reference_m;
      terminal_state.reference[model::kHeadingIndex] =
        contract_endpoint.heading_reference_rad;
      terminal_state.lower[model::kLateralIndex] =
        std::max(
        terminal_state.lower[model::kLateralIndex],
        contract_endpoint.lateral_reference_m -
        contract_endpoint.lateral_tolerance_m);
      terminal_state.upper[model::kLateralIndex] =
        std::min(
        terminal_state.upper[model::kLateralIndex],
        contract_endpoint.lateral_reference_m +
        contract_endpoint.lateral_tolerance_m);
      terminal_state.lower[model::kHeadingIndex] =
        std::max(
        terminal_state.lower[model::kHeadingIndex],
        contract_endpoint.heading_reference_rad -
        contract_endpoint.heading_tolerance_rad);
      terminal_state.upper[model::kHeadingIndex] =
        std::min(
        terminal_state.upper[model::kHeadingIndex],
        contract_endpoint.heading_reference_rad +
        contract_endpoint.heading_tolerance_rad);
      if (
        terminal_state.lower[model::kLateralIndex] >
        terminal_state.upper[model::kLateralIndex] ||
        terminal_state.lower[model::kHeadingIndex] >
        terminal_state.upper[model::kHeadingIndex])
      {
        return reject(
          RejectReason::TerminalSuccessorUnavailable,
          "Return endpoint is outside the physical state corridor");
      }
      seed.lateral_reference_m.back() = contract_endpoint.lateral_reference_m;

      const auto terminal = resolve_terminal_successor(source);
      if (!terminal.accepted) {
        return reject(
          RejectReason::TerminalSuccessorUnavailable,
          terminal.detail);
      }
      seed.predicted_encounter_stage_count =
        terminal.predicted_encounter_stage_count;
      seed.terminal_successor = terminal.successor;
      seed.stop_suffix = terminal.stop_suffix;
      if (!architecture::interaction_snapshot_complete(candidate)) {
        return reject(
          RejectReason::CandidateSealUnavailable,
          "rebuilt Return candidate is incomplete");
      }
      seed.candidate_fingerprint =
        architecture::fingerprint_interaction_snapshot(candidate);
      if (seed.candidate_fingerprint == 0U) {
        return reject(
          RejectReason::CandidateSealUnavailable,
          "rebuilt Return candidate fingerprint unavailable");
      }
      Result result;
      result.reason = RejectReason::Accepted;
      const char * const topology_name =
        longitudinal_topology == mpcc_rate_resolved_dynamic_obstacle::
        LongitudinalTopology::StayBehind ?
        "stay-behind" :
        longitudinal_topology == mpcc_rate_resolved_dynamic_obstacle::
        LongitudinalTopology::StayAhead ?
        "stay-ahead" :
        "side";
      result.detail = std::string{"accepted/Return/"} + topology_name +
        "/side=" + std::to_string(dynamic_side_sign);
      result.seed = std::move(seed);
      return result;
    }

    Seed seed;
    seed.target_id = source.identity.source_context.target_id;
    seed.pass_side_sign = pass_side_sign;
    seed.source_interaction_fingerprint = source_interaction_fingerprint;
    seed.path_distance_m = source.nominal_path_distance_m;
    seed.lateral_reference_m.reserve(static_cast<std::size_t>(horizon + 1));
    seed.solver_snapshot = source;
    auto & candidate = seed.solver_snapshot;
    if (!normal_avoidance) {
      candidate.identity.source_context.execution_side_sign = pass_side_sign;
    }
    candidate.identity.source_context.dynamic_obstacle_constraint_active = true;
    candidate.identity.source_context.dynamic_obstacle_id = selected_obstacle_id;
    candidate.identity.source_context.dynamic_obstacle_generation =
      selected_obstacle_generation;
    candidate.identity.source_context.dynamic_obstacle_side_sign =
      pass_side_sign;
    candidate.identity.source_context.fingerprint = 0U;
    candidate.identity.source_context = contract::seal_problem_context(
      candidate.identity.source_context);
    candidate.dynamic_obstacle_pass_side_sign = pass_side_sign;
    candidate.dynamic_obstacle_refinement_active = true;
    candidate.dynamic_obstacle_stages = target_horizon.stages;
    // The source may itself be a failed Direct/Diagonal/Disjunction
    // candidate captured by the architecture auditor.  A stateless seed must
    // not inherit that candidate's temporal topology; otherwise DirectSide is
    // mislabeled and every derived member combines old and new forced rows.
    // Candidate-specific builders below are the sole owners of these fields.
    candidate.dynamic_obstacle_longitudinal_topology =
      mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::Automatic;
    candidate.dynamic_obstacle_forced_first_pass_side_stage = -1;
    candidate.dynamic_obstacle_forced_first_ahead_stage = -1;
    candidate.dynamic_obstacle_forced_constraint_fraction = 1.0;
    candidate.dynamic_obstacle_forced_diagonal_start_stage = -1;
    candidate.dynamic_obstacle_forced_diagonal_full_side_stage = -1;
    candidate.dynamic_obstacle_forced_physical_diagonal = false;

    auto & initial = candidate.request.states.front();
    initial.reference[model::kLateralIndex] =
      candidate.request.initial_state[model::kLateralIndex];
    initial.reference[model::kLagIndex] =
      candidate.request.initial_state[model::kLagIndex];
    initial.reference[model::kHeadingIndex] =
      candidate.request.initial_state[model::kHeadingIndex];
    seed.lateral_reference_m.push_back(
      initial.reference[model::kLateralIndex]);

    const auto terminal = resolve_terminal_successor(source);
    if (!terminal.accepted &&
      terminal.detail == "no valid target prediction stage")
    {
      return reject(
        RejectReason::DynamicTargetUnavailable,
        terminal.detail);
    }
    seed.predicted_encounter_stage_count =
      terminal.predicted_encounter_stage_count;

    for (int stage = 1; stage <= horizon; ++stage) {
      auto & state = candidate.request.states[static_cast<std::size_t>(stage)];
      const auto & target = target_horizon.stages[
        static_cast<std::size_t>(stage - 1)];
      const double lower = state.lower[model::kLateralIndex];
      const double upper = state.upper[model::kLateralIndex];
      if (!valid_interval(lower, upper)) {
        std::ostringstream detail;
        detail << "invalid lateral interval at state " << stage;
        return reject(
          RejectReason::LateralIntervalUnavailable, detail.str());
      }
      double desired_lateral_m = 0.0;
      const bool side_reference_active = target.valid &&
        (seed.predicted_encounter_stage_count == 0U ||
        static_cast<std::size_t>(stage) <= terminal.last_encounter_state);
      if (side_reference_active) {
        desired_lateral_m = target.target_lateral_m +
          static_cast<double>(pass_side_sign) *
          target.lateral_center_separation_m;
      }
      desired_lateral_m = std::clamp(desired_lateral_m, lower, upper);
      state.reference[model::kLateralIndex] = desired_lateral_m;
      state.reference[model::kLagIndex] = 0.0;
      state.reference[model::kHeadingIndex] = 0.0;
      seed.lateral_reference_m.push_back(desired_lateral_m);
    }
    if (!terminal.accepted) {
      return reject(
        RejectReason::TerminalSuccessorUnavailable, terminal.detail);
    }
    seed.terminal_successor = terminal.successor;
    seed.stop_suffix = terminal.stop_suffix;

    if (!architecture::interaction_snapshot_complete(candidate)) {
      return reject(
        RejectReason::CandidateSealUnavailable,
        "rebuilt candidate is incomplete");
    }
    seed.candidate_fingerprint =
      architecture::fingerprint_interaction_snapshot(candidate);
    if (seed.candidate_fingerprint == 0U) {
      return reject(
        RejectReason::CandidateSealUnavailable,
        "rebuilt candidate fingerprint unavailable");
    }
    Result result;
    result.reason = RejectReason::Accepted;
    result.detail = "accepted";
    result.seed = std::move(seed);
    return result;
  } catch (const std::exception & exception) {
    return reject(RejectReason::CandidateSealUnavailable, exception.what());
  } catch (...) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "unknown stateless producer exception");
  }
}

Result build(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign) noexcept
{
  return build_with_intent_policy(
    source, source_interaction_fingerprint, pass_side_sign, false);
}

Result build_normal_avoidance(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign) noexcept
{
  const auto intent = source.identity.source_context.intent;
  if (
    intent != mpcc_execution_contract::ControlIntent::Follow &&
    intent != mpcc_execution_contract::ControlIntent::Cruise)
  {
    return reject(
      RejectReason::UnsupportedIntent,
      "normal avoidance accepts only Cruise or Follow intent");
  }
  return build_with_intent_policy(
    source, source_interaction_fingerprint, pass_side_sign, true);
}

Result bind_current_world_target_preserving_geometry(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  namespace model = mpcc_rate_resolved;
  const int source_side = source.identity.source_context.execution_side_sign;
  auto result = build(source, source_interaction_fingerprint, source_side);
  if (!result.seed.has_value()) {
    return result;
  }

  auto & seed = result.seed.value();
  auto & candidate = seed.solver_snapshot;
  // build() sealed the newly introduced target rows into the candidate
  // problem identity.  Preserve that identity while restoring only the
  // captured path/reference geometry; assigning source.identity here would
  // make the QP rows and their immutable provenance disagree.
  candidate.request = source.request;
  seed.lateral_reference_m.clear();
  seed.lateral_reference_m.reserve(source.request.states.size());
  for (const auto & state : source.request.states) {
    seed.lateral_reference_m.push_back(
      state.reference[model::kLateralIndex]);
  }
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "target-bound persistent candidate is incomplete");
  }
  seed.candidate_fingerprint =
    architecture::fingerprint_interaction_snapshot(candidate);
  if (seed.candidate_fingerprint == 0U) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "target-bound persistent candidate fingerprint unavailable");
  }
  result.reason = RejectReason::Accepted;
  result.detail = "accepted";
  return result;
}

Result build_disjunction_schedule(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign, const int first_pass_side_stage,
  const int first_ahead_stage, const double constraint_fraction) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  if (
    source.identity.source_context.intent ==
    mpcc_execution_contract::ControlIntent::Return)
  {
    return reject(
      RejectReason::UnsupportedIntent,
      "Return does not admit pass-side disjunction schedules");
  }
  auto result = build(source, source_interaction_fingerprint, pass_side_sign);
  if (!result.seed.has_value()) {
    return result;
  }
  const int horizon = source.request.horizon_steps;
  if (
    first_pass_side_stage < 0 || first_pass_side_stage >= horizon ||
    first_ahead_stage <= first_pass_side_stage ||
    first_ahead_stage > horizon || !std::isfinite(constraint_fraction) ||
    constraint_fraction < 0.0 || constraint_fraction > 1.0)
  {
    return reject(
      RejectReason::InvalidTransitionStage,
      "lattice transition stage is outside the planning horizon");
  }
  auto & seed = result.seed.value();
  auto & candidate = seed.solver_snapshot;
  candidate.dynamic_obstacle_forced_first_pass_side_stage =
    first_pass_side_stage;
  candidate.dynamic_obstacle_forced_first_ahead_stage = first_ahead_stage;
  candidate.dynamic_obstacle_forced_constraint_fraction =
    constraint_fraction;
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "exact disjunction candidate is incomplete");
  }
  seed.candidate_fingerprint =
    architecture::fingerprint_interaction_snapshot(candidate);
  if (seed.candidate_fingerprint == 0U) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "exact disjunction candidate fingerprint unavailable");
  }
  result.reason = RejectReason::Accepted;
  result.detail = "accepted";
  return result;
}

Result build_lattice(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign, const int first_pass_side_stage,
  const int first_ahead_stage) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  namespace model = mpcc_rate_resolved;
  auto result = build_disjunction_schedule(
    source, source_interaction_fingerprint, pass_side_sign,
    first_pass_side_stage, first_ahead_stage, 1.0);
  if (!result.seed.has_value()) {
    return result;
  }
  const int horizon = source.request.horizon_steps;
  const auto target_horizon = resolve_canonical_target_horizon(source);
  if (
    !target_horizon.accepted ||
    target_horizon.stages.size() != static_cast<std::size_t>(horizon))
  {
    return reject(
      RejectReason::DynamicTargetUnavailable, target_horizon.detail);
  }

  auto & seed = result.seed.value();
  auto & candidate = seed.solver_snapshot;
  seed.lateral_reference_m.clear();
  seed.lateral_reference_m.reserve(static_cast<std::size_t>(horizon + 1));
  const double initial_lateral_m =
    candidate.request.initial_state[model::kLateralIndex];
  seed.lateral_reference_m.push_back(initial_lateral_m);
  candidate.request.states.front().reference[model::kLateralIndex] =
    initial_lateral_m;
  for (int stage = 1; stage <= horizon; ++stage) {
    auto & state = candidate.request.states[static_cast<std::size_t>(stage)];
    const auto & target = target_horizon.stages[
      static_cast<std::size_t>(stage - 1)];
    const double full_side_lateral_m = target.target_lateral_m +
      static_cast<double>(pass_side_sign) *
      target.lateral_center_separation_m;
    const double raw_fraction = std::min(
      1.0, static_cast<double>(stage) /
      static_cast<double>(first_pass_side_stage + 1));
    const double smooth_fraction =
      raw_fraction * raw_fraction * (3.0 - 2.0 * raw_fraction);
    double desired_lateral_m = initial_lateral_m +
      smooth_fraction * (full_side_lateral_m - initial_lateral_m);
    if (stage - 1 >= first_ahead_stage) {
      const double return_fraction = std::clamp(
        static_cast<double>(stage - first_ahead_stage) /
        static_cast<double>(horizon - first_ahead_stage), 0.0, 1.0);
      const double return_smooth =
        return_fraction * return_fraction * (3.0 - 2.0 * return_fraction);
      desired_lateral_m *= 1.0 - return_smooth;
    }
    desired_lateral_m = std::clamp(
      desired_lateral_m, state.lower[model::kLateralIndex],
      state.upper[model::kLateralIndex]);
    state.reference[model::kLateralIndex] = desired_lateral_m;
    state.reference[model::kLagIndex] = 0.0;
    state.reference[model::kHeadingIndex] = 0.0;
    seed.lateral_reference_m.push_back(desired_lateral_m);
  }
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "rough lattice candidate is incomplete");
  }
  seed.candidate_fingerprint =
    architecture::fingerprint_interaction_snapshot(candidate);
  if (seed.candidate_fingerprint == 0U) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "rough lattice candidate fingerprint unavailable");
  }
  result.reason = RejectReason::Accepted;
  result.detail = "accepted";
  return result;
}

Result build_normal_avoidance_lattice(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign, const int first_pass_side_stage,
  const int first_ahead_stage) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  namespace model = mpcc_rate_resolved;
  auto result = build_normal_avoidance(
    source, source_interaction_fingerprint, pass_side_sign);
  if (!result.seed.has_value()) {
    return result;
  }
  const int horizon = source.request.horizon_steps;
  if (
    first_pass_side_stage < 0 || first_pass_side_stage >= horizon ||
    first_ahead_stage <= first_pass_side_stage ||
    first_ahead_stage > horizon)
  {
    return reject(
      RejectReason::InvalidTransitionStage,
      "normal-avoidance lattice is outside the planning horizon");
  }
  const auto target_horizon = resolve_canonical_target_horizon(source);
  if (
    !target_horizon.accepted ||
    target_horizon.stages.size() != static_cast<std::size_t>(horizon))
  {
    return reject(
      RejectReason::DynamicTargetUnavailable, target_horizon.detail);
  }

  auto & seed = result.seed.value();
  auto & candidate = seed.solver_snapshot;
  candidate.dynamic_obstacle_forced_first_pass_side_stage =
    first_pass_side_stage;
  candidate.dynamic_obstacle_forced_first_ahead_stage = first_ahead_stage;
  candidate.dynamic_obstacle_forced_constraint_fraction = 1.0;

  const double initial_lateral_m =
    candidate.request.initial_state[model::kLateralIndex];
  seed.lateral_reference_m.clear();
  seed.lateral_reference_m.reserve(static_cast<std::size_t>(horizon + 1));
  seed.lateral_reference_m.push_back(initial_lateral_m);
  candidate.request.states.front().reference[model::kLateralIndex] =
    initial_lateral_m;
  for (int stage = 1; stage <= horizon; ++stage) {
    auto & state = candidate.request.states[static_cast<std::size_t>(stage)];
    const auto & target = target_horizon.stages[
      static_cast<std::size_t>(stage - 1)];
    const double full_side_lateral_m = target.target_lateral_m +
      static_cast<double>(pass_side_sign) *
      target.lateral_center_separation_m;
    const double side_fraction_raw = std::clamp(
      static_cast<double>(stage) /
      static_cast<double>(first_pass_side_stage + 1), 0.0, 1.0);
    const double side_fraction = side_fraction_raw * side_fraction_raw *
      (3.0 - 2.0 * side_fraction_raw);
    double desired_lateral_m = initial_lateral_m +
      side_fraction * (full_side_lateral_m - initial_lateral_m);
    if (stage - 1 >= first_ahead_stage) {
      const double return_fraction_raw = std::clamp(
        static_cast<double>(stage - first_ahead_stage) /
        static_cast<double>(horizon - first_ahead_stage), 0.0, 1.0);
      const double return_fraction =
        return_fraction_raw * return_fraction_raw *
        (3.0 - 2.0 * return_fraction_raw);
      desired_lateral_m = desired_lateral_m * (1.0 - return_fraction) +
        source.request.states[static_cast<std::size_t>(stage)].reference[
        model::kLateralIndex] * return_fraction;
    }
    desired_lateral_m = std::clamp(
      desired_lateral_m, state.lower[model::kLateralIndex],
      state.upper[model::kLateralIndex]);
    state.reference[model::kLateralIndex] = desired_lateral_m;
    state.reference[model::kLagIndex] = 0.0;
    state.reference[model::kHeadingIndex] = 0.0;
    seed.lateral_reference_m.push_back(desired_lateral_m);
  }
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "normal-avoidance lattice candidate is incomplete");
  }
  seed.candidate_fingerprint =
    architecture::fingerprint_interaction_snapshot(candidate);
  if (seed.candidate_fingerprint == 0U) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "normal-avoidance lattice fingerprint unavailable");
  }
  result.reason = RejectReason::Accepted;
  result.detail = "accepted/audit-only-normal-avoidance-lattice";
  return result;
}

Result build_return_rejoin_schedule(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int rejoin_start_stage, const int rejoin_complete_stage) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  namespace contract = mpcc_execution_contract;
  namespace model = mpcc_rate_resolved;
  if (source.identity.source_context.intent != contract::ControlIntent::Return) {
    return reject(
      RejectReason::UnsupportedIntent,
      "Return rejoin schedule accepts only Return intent");
  }
  const int horizon = source.request.horizon_steps;
  if (
    rejoin_start_stage < 0 || rejoin_start_stage >= horizon ||
    rejoin_complete_stage <= rejoin_start_stage ||
    rejoin_complete_stage > horizon)
  {
    return reject(
      RejectReason::InvalidTransitionStage,
      "Return rejoin schedule is outside the planning horizon");
  }
  const int source_side =
    source.identity.source_context.execution_side_sign;
  if (source_side != -1 && source_side != 1) {
    return reject(
      RejectReason::InvalidSide,
      "Return rejoin schedule requires the committed homotopy");
  }
  auto result = build(source, source_interaction_fingerprint, source_side);
  if (!result.seed.has_value()) {
    return result;
  }

  auto & seed = result.seed.value();
  auto & candidate = seed.solver_snapshot;
  const double initial_lateral_m =
    candidate.request.initial_state[model::kLateralIndex];
  const double terminal_lateral_m =
    source.terminal_intent_contract.lateral_reference_m;
  std::vector<double> reference_lateral_m(
    static_cast<std::size_t>(horizon + 1), initial_lateral_m);
  for (int stage = 1; stage <= horizon; ++stage) {
    const double raw_fraction = std::clamp(
      static_cast<double>(stage - rejoin_start_stage) /
      static_cast<double>(rejoin_complete_stage - rejoin_start_stage),
      0.0, 1.0);
    const double smooth_fraction =
      raw_fraction * raw_fraction * (3.0 - 2.0 * raw_fraction);
    auto & state = candidate.request.states[static_cast<std::size_t>(stage)];
    const double desired_lateral_m = std::clamp(
      initial_lateral_m +
      smooth_fraction * (terminal_lateral_m - initial_lateral_m),
      state.lower[model::kLateralIndex],
      state.upper[model::kLateralIndex]);
    reference_lateral_m[static_cast<std::size_t>(stage)] =
      desired_lateral_m;
    state.reference[model::kLateralIndex] = desired_lateral_m;
    state.reference[model::kLagIndex] = 0.0;
  }

  // Supply the same SQP with a geometric heading seed derived from the new
  // polynomial reference.  This is a reference only; steering, yaw-response,
  // wall and terminal bounds remain unchanged and retain final authority.
  for (int stage = 1; stage < horizon; ++stage) {
    const auto before = static_cast<std::size_t>(stage - 1);
    const auto after = static_cast<std::size_t>(stage + 1);
    const double distance_m =
      seed.path_distance_m[after] - seed.path_distance_m[before];
    const double slope = distance_m > kNumericalTolerance ?
      (reference_lateral_m[after] - reference_lateral_m[before]) /
      distance_m : 0.0;
    auto & state = candidate.request.states[static_cast<std::size_t>(stage)];
    state.reference[model::kHeadingIndex] = std::clamp(
      std::atan(slope), state.lower[model::kHeadingIndex],
      state.upper[model::kHeadingIndex]);
  }
  candidate.request.states.back().reference[model::kHeadingIndex] =
    source.terminal_intent_contract.heading_reference_rad;
  seed.lateral_reference_m = std::move(reference_lateral_m);

  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "Return rejoin schedule candidate is incomplete");
  }
  seed.candidate_fingerprint =
    architecture::fingerprint_interaction_snapshot(candidate);
  if (seed.candidate_fingerprint == 0U) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "Return rejoin schedule fingerprint unavailable");
  }
  result.reason = RejectReason::Accepted;
  result.detail = "accepted/audit-only-Return-rejoin-schedule";
  return result;
}

Result build_diagonal_schedule(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign, const int diagonal_start_stage,
  const int full_side_stage) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  auto result = build(source, source_interaction_fingerprint, pass_side_sign);
  if (!result.seed.has_value()) {
    return result;
  }
  const int horizon = source.request.horizon_steps;
  if (
    diagonal_start_stage < 0 || full_side_stage >= horizon ||
    full_side_stage < diagonal_start_stage + 2)
  {
    return reject(
      RejectReason::InvalidTransitionStage,
      "diagonal guidance schedule is outside the planning horizon");
  }
  auto & seed = result.seed.value();
  auto & candidate = seed.solver_snapshot;
  candidate.dynamic_obstacle_forced_diagonal_start_stage =
    diagonal_start_stage;
  candidate.dynamic_obstacle_forced_diagonal_full_side_stage =
    full_side_stage;
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "diagonal guidance candidate is incomplete");
  }
  seed.candidate_fingerprint =
    architecture::fingerprint_interaction_snapshot(candidate);
  if (seed.candidate_fingerprint == 0U) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "diagonal guidance candidate fingerprint unavailable");
  }
  result.reason = RejectReason::Accepted;
  result.detail = "accepted";
  return result;
}

Result build_physical_diagonal_schedule(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign, const int diagonal_start_stage,
  const int full_side_stage) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  auto result = build_diagonal_schedule(
    source, source_interaction_fingerprint, pass_side_sign,
    diagonal_start_stage, full_side_stage);
  if (!result.seed.has_value()) {
    return result;
  }
  auto & seed = result.seed.value();
  auto & candidate = seed.solver_snapshot;
  candidate.dynamic_obstacle_forced_physical_diagonal = true;
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "physical diagonal guidance candidate is incomplete");
  }
  seed.candidate_fingerprint =
    architecture::fingerprint_interaction_snapshot(candidate);
  if (seed.candidate_fingerprint == 0U) {
    return reject(
      RejectReason::CandidateSealUnavailable,
      "physical diagonal guidance candidate fingerprint unavailable");
  }
  result.reason = RejectReason::Accepted;
  result.detail = "accepted";
  return result;
}

CandidateSet build_bounded_candidates(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const int pass_side_sign) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  namespace model = mpcc_rate_resolved;
  CandidateSet result;
  const auto source_fingerprint =
    architecture::fingerprint_interaction_snapshot(source);
  result.source_interaction_fingerprint = source_fingerprint;
  if (source_fingerprint == 0U) {
    result.reason = RejectReason::IncompleteSnapshot;
    result.detail = "current-world interaction fingerprint unavailable";
    return result;
  }

  if (
    source.identity.source_context.intent ==
    mpcc_execution_contract::ControlIntent::Return)
  {
    auto rejoin = build(source, source_fingerprint, pass_side_sign);
    if (!rejoin.seed.has_value()) {
      result.reason = rejoin.reason;
      result.detail = rejoin.detail;
      return result;
    }
    result.candidates.push_back(
      Candidate{CandidateKind::ReturnRejoin, std::move(rejoin.seed.value())});
    result.reason = RejectReason::Accepted;
    result.detail = "current-world Return rejoin candidate";
    return result;
  }

  auto direct = build(source, source_fingerprint, pass_side_sign);
  if (!direct.seed.has_value()) {
    result.reason = direct.reason;
    result.detail = direct.detail;
    return result;
  }
  // Keep candidate assembly outside result.candidates.  Holding a reference
  // to candidates.front() while appending the second member used to read a
  // vector-invalidated Snapshot when capacity grew, making the third topology
  // non-deterministically disappear.  Move the completed bounded population
  // only after every schedule has been derived from the stable direct seed.
  const auto & direct_snapshot = direct.seed->solver_snapshot;
  std::optional<Candidate> steering_reachable_candidate;
  std::optional<Candidate> mid_candidate;
  std::optional<Candidate> third_candidate;
  int first_valid_stage = -1;
  int last_contiguous_valid_stage = -1;
  for (int stage = 0; stage < direct_snapshot.request.horizon_steps; ++stage) {
    const bool valid =
      static_cast<std::size_t>(stage) <
      direct_snapshot.dynamic_obstacle_stages.size() &&
      direct_snapshot.dynamic_obstacle_stages[
        static_cast<std::size_t>(stage)].valid;
    if (valid && first_valid_stage < 0) {
      first_valid_stage = stage;
      last_contiguous_valid_stage = stage;
    } else if (valid && last_contiguous_valid_stage == stage - 1) {
      last_contiguous_valid_stage = stage;
    } else if (first_valid_stage >= 0) {
      break;
    }
  }
  const int terminal_side_stage = direct_snapshot.request.horizon_steps - 1;
  const auto steering_reachable_stage =
    steering_reachable_full_side_stage(
    direct_snapshot, pass_side_sign, first_valid_stage);
  if (steering_reachable_stage.has_value()) {
    auto diagonal = build_physical_diagonal_schedule(
      source, source_fingerprint, pass_side_sign,
      first_valid_stage, steering_reachable_stage.value());
    if (diagonal.seed.has_value()) {
      steering_reachable_candidate.emplace(
        Candidate{
          CandidateKind::SteeringReachablePhysicalDiagonal,
          std::move(diagonal.seed.value())});
    }
  }
  // DirectSide already represents immediate avoidance.  The former
  // first_valid+2 sample duplicated that temporal extreme and omitted the
  // ordinary gradual transition: on a 20-stage horizon production sampled
  // full-side at stages 2 and 19, while the unchanged proof pipeline certified
  // the same frozen problem at stage 9.  Sample the integer midpoint of the
  // current valid encounter interval and keep the population bounded.
  const int mid_full_side_stage = std::max(
    first_valid_stage + 2,
    first_valid_stage + (terminal_side_stage - first_valid_stage) / 2);
  if (
    first_valid_stage >= 0 &&
    mid_full_side_stage < direct_snapshot.request.horizon_steps &&
    (!steering_reachable_stage.has_value() ||
    steering_reachable_stage.value() != mid_full_side_stage))
  {
    auto diagonal = build_physical_diagonal_schedule(
      source, source_fingerprint, pass_side_sign,
      first_valid_stage, mid_full_side_stage);
    if (diagonal.seed.has_value()) {
      mid_candidate.emplace(
        Candidate{
          CandidateKind::MidPhysicalDiagonal,
          std::move(diagonal.seed.value())});
    }
  }

  const int horizon = direct_snapshot.request.horizon_steps;
  const bool finite_encounter_boundary =
    first_valid_stage >= 0 && last_contiguous_valid_stage >= first_valid_stage &&
    last_contiguous_valid_stage + 1 < horizon;
  int encounter_boundary_stage = -1;
  int last_nominal_stay_behind_stage = -1;
  if (finite_encounter_boundary) {
    // A finite canonical target tube ends before the unrelated control
    // horizon. Represent that measured temporal topology explicitly instead
    // of silently dropping the third candidate when later target stages are
    // invalid. The unchanged SQP and exact ReplayWorld proofs retain final
    // authority.
    encounter_boundary_stage = last_contiguous_valid_stage + 1;
    last_nominal_stay_behind_stage = first_valid_stage - 1;
    for (
      int stage = first_valid_stage;
      stage <= last_contiguous_valid_stage; ++stage)
    {
      const auto index = static_cast<std::size_t>(stage);
      if (index >= direct_snapshot.request.states.size()) {
        break;
      }
      const auto & target = direct_snapshot.dynamic_obstacle_stages[index];
      const double nominal_ego_progress_m =
        direct_snapshot.request.states[index].reference[model::kProgressIndex];
      const double stay_behind_upper_m =
        target.target_progress_m - target.longitudinal_overlap_m;
      if (
        !std::isfinite(nominal_ego_progress_m) ||
        !std::isfinite(stay_behind_upper_m) ||
        nominal_ego_progress_m > stay_behind_upper_m)
      {
        break;
      }
      last_nominal_stay_behind_stage = stage;
    }
    if (
      last_nominal_stay_behind_stage >= first_valid_stage &&
      encounter_boundary_stage >= last_nominal_stay_behind_stage + 2)
    {
      auto boundary_diagonal = build_physical_diagonal_schedule(
        source, source_fingerprint, pass_side_sign,
        last_nominal_stay_behind_stage, encounter_boundary_stage);
      if (boundary_diagonal.seed.has_value()) {
        third_candidate.emplace(
          Candidate{
            CandidateKind::EncounterBoundaryPhysicalDiagonal,
            std::move(boundary_diagonal.seed.value())});
      }
    }
  } else {
    // When the target occupies the complete prediction horizon there is no
    // observed encounter boundary. Preserve the independently certified
    // complete-disjunction topology rather than inventing one beyond the
    // sealed current-world evidence.
    constexpr int kLateSideSuffixStageCount = 3;
    const int late_side_stage = horizon - kLateSideSuffixStageCount;
    if (first_valid_stage >= 0 && late_side_stage > first_valid_stage) {
      auto late_exact = build_disjunction_schedule(
        source, source_fingerprint, pass_side_sign,
        late_side_stage, horizon, 1.0);
      if (late_exact.seed.has_value()) {
        third_candidate.emplace(
          Candidate{
            CandidateKind::LateExactDisjunction,
            std::move(late_exact.seed.value())});
      }
    }
  }

  double first_nominal_progress_m = std::numeric_limits<double>::quiet_NaN();
  double first_target_progress_m = std::numeric_limits<double>::quiet_NaN();
  double first_longitudinal_overlap_m =
    std::numeric_limits<double>::quiet_NaN();
  if (
    first_valid_stage >= 0 &&
    static_cast<std::size_t>(first_valid_stage) <
    direct_snapshot.dynamic_obstacle_stages.size() &&
    static_cast<std::size_t>(first_valid_stage) <
    direct_snapshot.request.states.size())
  {
    const auto index = static_cast<std::size_t>(first_valid_stage);
    first_nominal_progress_m = direct_snapshot.request.states[index].reference[
      model::kProgressIndex];
    first_target_progress_m =
      direct_snapshot.dynamic_obstacle_stages[index].target_progress_m;
    first_longitudinal_overlap_m =
      direct_snapshot.dynamic_obstacle_stages[index].longitudinal_overlap_m;
  }

  result.candidates.reserve(4U);
  result.candidates.push_back(
    Candidate{CandidateKind::DirectSide, std::move(direct.seed.value())});
  if (steering_reachable_candidate.has_value()) {
    result.candidates.push_back(
      std::move(steering_reachable_candidate.value()));
  }
  if (mid_candidate.has_value()) {
    result.candidates.push_back(std::move(mid_candidate.value()));
  }
  if (third_candidate.has_value()) {
    result.candidates.push_back(std::move(third_candidate.value()));
  }

  result.reason = RejectReason::Accepted;
  std::ostringstream detail;
  detail << "bounded current-world candidate population/count="
         << result.candidates.size() << "/valid=" << first_valid_stage
         << ':' << last_contiguous_valid_stage << "/stay_behind="
         << last_nominal_stay_behind_stage << "/boundary="
         << encounter_boundary_stage << "/steering_reachable="
         << steering_reachable_stage.value_or(-1) << "/mid="
         << mid_full_side_stage;
  if (std::isfinite(first_nominal_progress_m)) {
    detail << "/first_nominal=" << first_nominal_progress_m
           << "/first_target=" << first_target_progress_m
           << "/first_overlap=" << first_longitudinal_overlap_m;
  }
  result.detail = detail.str();
  return result;
}

CandidateSet build_normal_avoidance_candidates(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  CandidateSet population;
  if (
    source.identity.source_context.intent !=
    mpcc_execution_contract::ControlIntent::Follow &&
    source.identity.source_context.intent !=
    mpcc_execution_contract::ControlIntent::Cruise)
  {
    population.reason = RejectReason::UnsupportedIntent;
    population.detail =
      "normal avoidance population accepts only Cruise or Follow intent";
    return population;
  }
  population.source_interaction_fingerprint =
    architecture::fingerprint_interaction_snapshot(source);
  if (population.source_interaction_fingerprint == 0U) {
    population.reason = RejectReason::IncompleteSnapshot;
    population.detail =
      "normal avoidance source fingerprint unavailable";
    return population;
  }
  std::ostringstream rejected;
  for (const int side : {1, -1}) {
    auto built = build_normal_avoidance(
      source, population.source_interaction_fingerprint, side);
    if (!built.seed.has_value()) {
      rejected << (side > 0 ? "positive" : "negative") << '='
               << to_string(built.reason) << '/' << built.detail << ';';
      continue;
    }
    population.candidates.push_back(
      Candidate{CandidateKind::DirectSide, std::move(built.seed.value())});
  }
  if (population.candidates.empty()) {
    population.reason = RejectReason::CandidateSealUnavailable;
    population.detail =
      std::string{"no normal avoidance candidate/"} + rejected.str();
    return population;
  }
  population.reason = RejectReason::Accepted;
  population.detail =
    std::string{"accepted/count="} +
    std::to_string(population.candidates.size());
  return population;
}

}  // namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
