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
  return std::isfinite(lower) && std::isfinite(upper) &&
         lower <= kNumericalTolerance && upper >= -kNumericalTolerance;
}

Result reject(const RejectReason reason, std::string detail)
{
  Result result;
  result.reason = reason;
  result.detail = std::move(detail);
  return result;
}

}  // namespace

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::Accepted: return "accepted";
    case RejectReason::IncompleteSnapshot: return "incomplete-snapshot";
    case RejectReason::SourceFingerprintMismatch:
      return "source-fingerprint-mismatch";
    case RejectReason::InvalidSide: return "invalid-side";
    case RejectReason::UnsupportedIntent: return "unsupported-intent";
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
    case TerminalSuccessor::Stop: return "stop";
  }
  return "unknown";
}

Result build(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign) noexcept
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
    if (!supported_intent(source.identity.source_context.intent)) {
      return reject(
        RejectReason::UnsupportedIntent,
        "stateless producer is restricted to Overtake intents");
    }
    const int horizon = source.request.horizon_steps;
    if (
      !source.dynamic_obstacle_refinement_active ||
      source.dynamic_obstacle_stages.size() !=
      static_cast<std::size_t>(horizon) ||
      source.nominal_path_distance_m.size() !=
      static_cast<std::size_t>(horizon + 1))
    {
      return reject(
        RejectReason::DynamicTargetUnavailable,
        "complete target stage horizon unavailable");
    }

    Seed seed;
    seed.target_id = source.identity.source_context.target_id;
    seed.pass_side_sign = pass_side_sign;
    seed.source_interaction_fingerprint = source_interaction_fingerprint;
    seed.path_distance_m = source.nominal_path_distance_m;
    seed.lateral_reference_m.reserve(static_cast<std::size_t>(horizon + 1));
    seed.solver_snapshot = source;
    auto & candidate = seed.solver_snapshot;
    candidate.identity.source_context.execution_side_sign = pass_side_sign;
    candidate.identity.source_context.fingerprint = 0U;
    candidate.identity.source_context = contract::seal_problem_context(
      candidate.identity.source_context);
    candidate.dynamic_obstacle_pass_side_sign = pass_side_sign;

    auto & initial = candidate.request.states.front();
    initial.reference[model::kLateralIndex] =
      candidate.request.initial_state[model::kLateralIndex];
    initial.reference[model::kLagIndex] =
      candidate.request.initial_state[model::kLagIndex];
    initial.reference[model::kHeadingIndex] =
      candidate.request.initial_state[model::kHeadingIndex];
    seed.lateral_reference_m.push_back(
      initial.reference[model::kLateralIndex]);

    bool target_stage_available = false;
    std::size_t last_encounter_state = 0U;
    for (int stage = 1; stage <= horizon; ++stage) {
      const auto & target = source.dynamic_obstacle_stages[
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
        ++seed.predicted_encounter_stage_count;
        last_encounter_state = static_cast<std::size_t>(stage);
      }
    }
    if (!target_stage_available) {
      return reject(
        RejectReason::DynamicTargetUnavailable,
        "no valid target prediction stage");
    }

    for (int stage = 1; stage <= horizon; ++stage) {
      auto & state = candidate.request.states[static_cast<std::size_t>(stage)];
      const auto & target = source.dynamic_obstacle_stages[
        static_cast<std::size_t>(stage - 1)];
      const double lower = state.lower[model::kLateralIndex];
      const double upper = state.upper[model::kLateralIndex];
      if (
        !std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
      {
        std::ostringstream detail;
        detail << "invalid lateral interval at state " << stage;
        return reject(
          RejectReason::LateralIntervalUnavailable, detail.str());
      }
      double desired_lateral_m = 0.0;
      const bool side_reference_active = target.valid &&
        (seed.predicted_encounter_stage_count == 0U ||
        static_cast<std::size_t>(stage) <= last_encounter_state);
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
    const auto & terminal = candidate.request.states.back();
    const double terminal_lower = terminal.lower[model::kLateralIndex];
    const double terminal_upper = terminal.upper[model::kLateralIndex];
    const bool return_visible =
      seed.predicted_encounter_stage_count > 0U &&
      last_encounter_state < static_cast<std::size_t>(horizon) &&
      contains_zero(terminal_lower, terminal_upper);
    if (return_visible) {
      seed.terminal_successor = TerminalSuccessor::Return;
    } else {
      double maximum_deceleration_mps2 = 0.0;
      for (const auto & input : candidate.request.inputs) {
        maximum_deceleration_mps2 = std::min(
          maximum_deceleration_mps2,
          input.lower[model::kAccelerationIndex]);
      }
      const bool stop_available =
        maximum_deceleration_mps2 < -kNumericalTolerance &&
        contains_zero(
          terminal.lower[model::kVelocityIndex],
          terminal.upper[model::kVelocityIndex]);
      if (!stop_available) {
        return reject(
          RejectReason::TerminalSuccessorUnavailable,
          "neither Return nor semantic Stop suffix is available");
      }
      seed.terminal_successor = TerminalSuccessor::Stop;
      seed.stop_suffix.available = true;
      seed.stop_suffix.hold_lateral_m =
        terminal.reference[model::kLateralIndex];
      seed.stop_suffix.target_velocity_mps = 0.0;
      seed.stop_suffix.maximum_deceleration_mps2 =
        maximum_deceleration_mps2;
    }

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

}  // namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
