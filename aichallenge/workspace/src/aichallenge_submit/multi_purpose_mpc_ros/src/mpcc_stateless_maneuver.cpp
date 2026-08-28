#include "multi_purpose_mpc_ros/mpcc_stateless_maneuver.hpp"

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
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

struct CourseProjection
{
  bool valid{false};
  double progress_m{};
  double lateral_m{};
  double squared_distance_m2{};
};

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

CourseProjection project_to_recorded_course(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots,
  const double x_m, const double y_m, const double minimum_progress_m) noexcept
{
  CourseProjection best;
  best.squared_distance_m2 = std::numeric_limits<double>::infinity();
  if (
    knots.size() < 2U || !std::isfinite(x_m) || !std::isfinite(y_m) ||
    !std::isfinite(minimum_progress_m))
  {
    return best;
  }
  for (std::size_t index = 1U; index < knots.size(); ++index) {
    const auto & from = knots[index - 1U];
    const auto & to = knots[index];
    const double dx_m = to.x_m - from.x_m;
    const double dy_m = to.y_m - from.y_m;
    const double length_squared_m2 = dx_m * dx_m + dy_m * dy_m;
    if (
      !std::isfinite(from.progress_m) || !std::isfinite(to.progress_m) ||
      !std::isfinite(from.x_m) || !std::isfinite(from.y_m) ||
      !std::isfinite(to.x_m) || !std::isfinite(to.y_m) ||
      to.progress_m <= from.progress_m + kNumericalTolerance ||
      !std::isfinite(length_squared_m2) ||
      length_squared_m2 <= kNumericalTolerance)
    {
      return CourseProjection{};
    }
    const double ratio = std::clamp(
      ((x_m - from.x_m) * dx_m + (y_m - from.y_m) * dy_m) /
      length_squared_m2, 0.0, 1.0);
    const double progress_m =
      from.progress_m + ratio * (to.progress_m - from.progress_m);
    if (progress_m + kNumericalTolerance < minimum_progress_m) {
      continue;
    }
    const double projected_x_m = from.x_m + ratio * dx_m;
    const double projected_y_m = from.y_m + ratio * dy_m;
    const double residual_x_m = x_m - projected_x_m;
    const double residual_y_m = y_m - projected_y_m;
    const double squared_distance_m2 =
      residual_x_m * residual_x_m + residual_y_m * residual_y_m;
    const auto frame = mpc_stage_geometry::sample_course_frame(
      knots, progress_m, kNumericalTolerance);
    if (!frame.has_value()) {
      return CourseProjection{};
    }
    const double lateral_m =
      std::cos(frame->heading_rad) * (y_m - frame->y_m) -
      std::sin(frame->heading_rad) * (x_m - frame->x_m);
    if (
      !std::isfinite(squared_distance_m2) || !std::isfinite(lateral_m))
    {
      return CourseProjection{};
    }
    if (
      !best.valid || squared_distance_m2 + kNumericalTolerance <
      best.squared_distance_m2 ||
      (std::abs(squared_distance_m2 - best.squared_distance_m2) <=
      kNumericalTolerance && progress_m < best.progress_m))
    {
      best.valid = true;
      best.progress_m = progress_m;
      best.lateral_m = lateral_m;
      best.squared_distance_m2 = squared_distance_m2;
    }
  }
  return best;
}

Result reject(const RejectReason reason, std::string detail)
{
  Result result;
  result.reason = reason;
  result.detail = std::move(detail);
  return result;
}

}  // namespace

TargetHorizon rebuild_target_horizon(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept
{
  TargetHorizon result;
  const int horizon = source.request.horizon_steps;
  if (
    horizon <= 0 || !source.replay_world.has_value() ||
    source.request.inputs.size() != static_cast<std::size_t>(horizon) ||
    source.wall_course_frame_knots.size() < 2U ||
    !std::isfinite(source.course_progress_origin_m))
  {
    result.detail = "current-world horizon inputs unavailable";
    return result;
  }
  const auto & world = source.replay_world.value();
  const auto target = std::find_if(
    world.obstacles.begin(), world.obstacles.end(),
    [&](const auto & obstacle) {
      return obstacle.id == source.identity.source_context.target_id;
    });
  if (
    target == world.obstacles.end() || target->observation_generation == 0U ||
    target->observation_generation != world.observation_generation ||
    !std::isfinite(target->x_m) || !std::isfinite(target->y_m) ||
    !std::isfinite(target->velocity_x_mps) ||
    !std::isfinite(target->velocity_y_mps) ||
    !std::isfinite(target->radius_m) || target->radius_m < 0.0 ||
    !world.physical_footprint.valid())
  {
    result.detail = "selected current-world target unavailable";
    return result;
  }
  const double observation_age_sec =
    source.control_prediction_origin_sec - world.observed_sec;
  if (!std::isfinite(observation_age_sec) || observation_age_sec < 0.0) {
    result.detail = "control prediction predates target observation";
    return result;
  }
  const auto & footprint = world.physical_footprint;
  const double longitudinal_overlap_m = std::max(
    footprint.front_extent_m, footprint.rear_extent_m) +
    footprint.margin_m + target->radius_m;
  const double lateral_center_separation_m = std::max(
    footprint.left_extent_m, footprint.right_extent_m) +
    footprint.margin_m + target->radius_m;
  if (
    !std::isfinite(longitudinal_overlap_m) || longitudinal_overlap_m <= 0.0 ||
    !std::isfinite(lateral_center_separation_m) ||
    lateral_center_separation_m <= 0.0)
  {
    result.detail = "physical target separation unavailable";
    return result;
  }

  result.stages.reserve(static_cast<std::size_t>(horizon));
  double elapsed_sec = observation_age_sec;
  double minimum_progress_m = source.wall_course_frame_knots.front().progress_m;
  for (int stage = 0; stage < horizon; ++stage) {
    const double dt_sec = source.request.inputs[
      static_cast<std::size_t>(stage)].stage_dt_sec;
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      result.stages.clear();
      result.detail = "invalid target prediction stage time";
      return result;
    }
    elapsed_sec += dt_sec;
    const double x_m = target->x_m + target->velocity_x_mps * elapsed_sec;
    const double y_m = target->y_m + target->velocity_y_mps * elapsed_sec;
    const auto projection = project_to_recorded_course(
      source.wall_course_frame_knots, x_m, y_m, minimum_progress_m);
    if (!projection.valid) {
      result.stages.clear();
      std::ostringstream detail;
      detail << "target course projection unavailable at stage " << stage;
      result.detail = detail.str();
      return result;
    }
    minimum_progress_m = projection.progress_m;
    result.stages.push_back(
      mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
        true,
        projection.progress_m - source.course_progress_origin_m,
        projection.lateral_m,
        longitudinal_overlap_m,
        lateral_center_separation_m});
  }
  result.accepted = true;
  result.detail = "accepted";
  return result;
}

TerminalResolution resolve_terminal_successor(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept
{
  namespace model = mpcc_rate_resolved;
  TerminalResolution resolution;
  const int horizon = source.request.horizon_steps;
  const auto target_horizon = rebuild_target_horizon(source);
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
    const auto target_horizon = rebuild_target_horizon(source);
    if (
      !target_horizon.accepted || target_horizon.stages.size() !=
      static_cast<std::size_t>(horizon) || source.nominal_path_distance_m.size() !=
      static_cast<std::size_t>(horizon + 1))
    {
      return reject(
        RejectReason::DynamicTargetUnavailable,
        target_horizon.detail);
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
    candidate.dynamic_obstacle_refinement_active = true;
    candidate.dynamic_obstacle_stages = target_horizon.stages;

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

}  // namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
