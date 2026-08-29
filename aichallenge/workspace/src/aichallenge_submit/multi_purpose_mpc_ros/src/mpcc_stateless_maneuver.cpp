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
  const auto & problem_context = source.identity.source_context;
  const std::string & selected_obstacle_id =
    problem_context.dynamic_obstacle_constraint_active ?
    problem_context.dynamic_obstacle_id : problem_context.target_id;
  const std::uint64_t selected_obstacle_generation =
    problem_context.dynamic_obstacle_constraint_active ?
    problem_context.dynamic_obstacle_generation :
    problem_context.target_obstacle_generation;
  const auto target = std::find_if(
    world.obstacles.begin(), world.obstacles.end(),
    [&](const auto & obstacle) {
      return obstacle.id == selected_obstacle_id;
    });
  if (
    selected_obstacle_id.empty() || selected_obstacle_generation == 0U ||
    target == world.obstacles.end() || target->observation_generation == 0U ||
    target->observation_generation != selected_obstacle_generation ||
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
    case CandidateKind::MidPhysicalDiagonal:
      return "mid-physical-diagonal";
    case CandidateKind::LatePhysicalDiagonal:
      return "late-physical-diagonal";
  }
  return "unknown";
}

static Result build_with_intent_policy(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign, const bool allow_follow_audit) noexcept
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
    const bool follow_audit =
      allow_follow_audit &&
      source.identity.source_context.intent ==
      mpcc_execution_contract::ControlIntent::Follow;
    if (!supported_intent(source.identity.source_context.intent) && !follow_audit) {
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
    if (!follow_audit) {
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

Result build_follow_escape(
  const mpcc_rate_resolved_shadow::Snapshot & source,
  const std::uint64_t source_interaction_fingerprint,
  const int pass_side_sign) noexcept
{
  if (
    source.identity.source_context.intent !=
    mpcc_execution_contract::ControlIntent::Follow)
  {
    return reject(
      RejectReason::UnsupportedIntent,
      "Follow escape audit accepts only Follow intent");
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
  const auto target_horizon = rebuild_target_horizon(source);
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
  CandidateSet result;
  const auto source_fingerprint =
    architecture::fingerprint_interaction_snapshot(source);
  result.source_interaction_fingerprint = source_fingerprint;
  if (source_fingerprint == 0U) {
    result.reason = RejectReason::IncompleteSnapshot;
    result.detail = "current-world interaction fingerprint unavailable";
    return result;
  }

  auto direct = build(source, source_fingerprint, pass_side_sign);
  if (!direct.seed.has_value()) {
    result.reason = direct.reason;
    result.detail = direct.detail;
    return result;
  }
  result.candidates.push_back(
    Candidate{CandidateKind::DirectSide, std::move(direct.seed.value())});

  const auto & direct_snapshot = result.candidates.front().seed.solver_snapshot;
  int first_valid_stage = -1;
  for (int stage = 0; stage < direct_snapshot.request.horizon_steps; ++stage) {
    if (
      static_cast<std::size_t>(stage) <
      direct_snapshot.dynamic_obstacle_stages.size() &&
      direct_snapshot.dynamic_obstacle_stages[
        static_cast<std::size_t>(stage)].valid)
    {
      first_valid_stage = stage;
      break;
    }
  }
  const int terminal_side_stage = direct_snapshot.request.horizon_steps - 1;
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
    mid_full_side_stage < direct_snapshot.request.horizon_steps)
  {
    auto diagonal = build_physical_diagonal_schedule(
      source, source_fingerprint, pass_side_sign,
      first_valid_stage, mid_full_side_stage);
    if (diagonal.seed.has_value()) {
      result.candidates.push_back(
        Candidate{
          CandidateKind::MidPhysicalDiagonal,
          std::move(diagonal.seed.value())});
    }
  }

  // The direct and mid-horizon candidates do not represent a physically valid
  // wait-then-shift maneuver. Sample one additional temporal homotopy at a
  // normalized late knot. This remains bounded and current-world-only; the
  // unchanged solver and nonlinear proofs decide whether it can be published.
  if (first_valid_stage >= 0 && terminal_side_stage > first_valid_stage + 1) {
    const int stage_span = terminal_side_stage - first_valid_stage;
    const int late_start_stage =
      first_valid_stage + (2 * stage_span + 2) / 3;
    if (late_start_stage + 1 < terminal_side_stage) {
      auto late_diagonal = build_physical_diagonal_schedule(
        source, source_fingerprint, pass_side_sign,
        late_start_stage, terminal_side_stage);
      if (late_diagonal.seed.has_value()) {
        result.candidates.push_back(
          Candidate{
            CandidateKind::LatePhysicalDiagonal,
            std::move(late_diagonal.seed.value())});
      }
    }
  }

  result.reason = RejectReason::Accepted;
  result.detail = "bounded current-world candidate population";
  return result;
}

CandidateSet build_follow_escape_candidates(
  const mpcc_rate_resolved_shadow::Snapshot & source) noexcept
{
  namespace architecture = mpcc_architecture_snapshot;
  CandidateSet population;
  if (
    source.identity.source_context.intent !=
    mpcc_execution_contract::ControlIntent::Follow)
  {
    population.reason = RejectReason::UnsupportedIntent;
    population.detail = "Follow escape population accepts only Follow intent";
    return population;
  }
  population.source_interaction_fingerprint =
    architecture::fingerprint_interaction_snapshot(source);
  if (population.source_interaction_fingerprint == 0U) {
    population.reason = RejectReason::IncompleteSnapshot;
    population.detail = "Follow escape source fingerprint unavailable";
    return population;
  }
  std::ostringstream rejected;
  for (const int side : {1, -1}) {
    auto built = build_follow_escape(
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
      std::string{"no Follow escape candidate/"} + rejected.str();
    return population;
  }
  population.reason = RejectReason::Accepted;
  population.detail =
    std::string{"accepted/count="} +
    std::to_string(population.candidates.size());
  return population;
}

}  // namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
