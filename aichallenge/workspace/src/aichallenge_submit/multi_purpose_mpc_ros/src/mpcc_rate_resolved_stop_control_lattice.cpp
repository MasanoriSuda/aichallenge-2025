#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_control_lattice.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_control_lattice
{
namespace
{

namespace contract = mpcc_execution_contract;
namespace architecture = mpcc_architecture_snapshot;
namespace model = mpcc_rate_resolved;
namespace physical = mpcc_rate_resolved_physical_adapter;
namespace recovery = recovery_footprint;

StopCandidateResult reject_stop(
  const Reason reason,
  std::string detail) noexcept
{
  StopCandidateResult result;
  result.reason = reason;
  result.detail = std::move(detail);
  return result;
}

ScheduleResult reject_schedule(
  const Reason reason,
  std::string detail) noexcept
{
  ScheduleResult result;
  result.reason = reason;
  result.detail = std::move(detail);
  return result;
}

int preferred_initial_rate_sign(
  const shadow::Snapshot & maximum_braking_stop) noexcept
{
  const double previous_rate =
    maximum_braking_stop.request.previous_input[model::kSteeringRateIndex];
  if (std::isfinite(previous_rate) && std::abs(previous_rate) > 1e-9) {
    return previous_rate > 0.0 ? 1 : -1;
  }
  const double steering = maximum_braking_stop.request.current_steering_rad;
  if (std::isfinite(steering) && std::abs(steering) > 1e-9) {
    return steering > 0.0 ? -1 : 1;
  }
  return 1;
}

struct ScheduleGeometry
{
  int first_switch_stage{};
  int second_switch_stage{};
  std::size_t first_legacy_index{};
  std::optional<std::size_t> positive_legacy_index;
  std::optional<std::size_t> negative_legacy_index;
};

double normalized_geometry_distance_squared(
  const ScheduleGeometry & lhs, const ScheduleGeometry & rhs,
  const int horizon_steps) noexcept
{
  const double denominator = static_cast<double>(std::max(1, horizon_steps));
  const double first_delta = static_cast<double>(
    lhs.first_switch_stage - rhs.first_switch_stage) / denominator;
  const double second_delta = static_cast<double>(
    lhs.second_switch_stage - rhs.second_switch_stage) / denominator;
  return first_delta * first_delta + second_delta * second_delta;
}

double nominal_geometry_distance_squared(
  const ScheduleGeometry & geometry, const int horizon_steps) noexcept
{
  const double denominator = static_cast<double>(std::max(1, horizon_steps));
  const double first =
    static_cast<double>(geometry.first_switch_stage) / denominator;
  const double second =
    static_cast<double>(geometry.second_switch_stage) / denominator;
  const double first_delta = first - 0.15;
  const double second_delta = second - 0.30;
  return first_delta * first_delta + second_delta * second_delta;
}

StopCandidateResult impose_maximum_braking_law(
  shadow::Snapshot candidate,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance,
  const char * source_description) noexcept
{
  if (candidate.request.states.empty() || candidate.request.inputs.empty() ||
    candidate.request.states.size() != candidate.request.inputs.size() + 1U)
  {
    return reject_stop(
      Reason::InvalidSource,
      std::string{source_description} + " state/input shape invalid");
  }
  if (!candidate.replay_world.has_value()) {
    return reject_stop(
      Reason::ReplayWorldUnavailable,
      std::string{source_description} + " replay world unavailable");
  }

  double minimum_acceleration_mps2 = 0.0;
  for (const auto & input : candidate.request.inputs) {
    const auto solver_bounds =
      mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
      input.lower[model::kAccelerationIndex],
      input.upper[model::kAccelerationIndex], solver_tolerance);
    if (!solver_bounds.has_value()) {
      return reject_stop(
        Reason::InvalidBrakingEnvelope,
        "maximum-braking solver inset unavailable");
    }
    minimum_acceleration_mps2 =
      std::min(minimum_acceleration_mps2, solver_bounds->lower);
  }

  const double initial_velocity_mps =
    candidate.request.initial_state[model::kVelocityIndex];
  if (!std::isfinite(initial_velocity_mps) || initial_velocity_mps < 0.0 ||
    !std::isfinite(minimum_acceleration_mps2) ||
    minimum_acceleration_mps2 >= 0.0)
  {
    return reject_stop(
      Reason::InvalidBrakingEnvelope,
      std::string{"invalid "} + source_description + " braking envelope");
  }

  // Exact Stop proof owns the complete suffix through rest, rather than the
  // shorter normal publication prefix.
  candidate.execution_prefix_steps = candidate.request.horizon_steps;
  candidate.request.maximum_braking_feasibility = true;
  mpcc_vehicle_model::State body{0, 0, 0, initial_velocity_mps,
    candidate.request.current_lateral_velocity_mps, candidate.request.current_yaw_rate_radps,
    candidate.request.current_steering_rad, candidate.request.current_response_steering_rad};
  candidate.request.states.front().reference[model::kVelocityIndex] =
    initial_velocity_mps;
  for (std::size_t stage = 0U; stage < candidate.request.inputs.size();
    ++stage)
  {
    auto & input = candidate.request.inputs[stage];
    if (!std::isfinite(input.stage_dt_sec) || input.stage_dt_sec <= 0.0) {
      return reject_stop(
        Reason::InvalidBrakingEnvelope,
        "invalid Stop stage duration");
    }
    const auto bounds = mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
      input.lower[model::kAccelerationIndex], input.upper[model::kAccelerationIndex],
      solver_tolerance);
    if (!bounds || bounds->lower >= 0.0) {
      return reject_stop(Reason::InvalidBrakingEnvelope, "Stop wire braking unavailable");
    }
    input.reference[model::kAccelerationIndex] = bounds->lower;
    input.reference[model::kVirtualProgressSpeedIndex] = std::max(0.0, body.forward_velocity_mps);
    const auto advanced = mpcc_vehicle_model::advance(
      body, {bounds->lower, 0.0}, candidate.request.vehicle_model, input.stage_dt_sec);
    if (!advanced) {
      return reject_stop(Reason::InvalidBrakingEnvelope, "Stop body transition rejected");
    }
    body = advanced->state;
    // The speed seed follows the same wire/body law. Future speed remains a
    // dynamic state: steering changes wheel forces and cannot coexist with a
    // separately imposed v0 + a*t equality. Only terminal rest is hard.
    candidate.request.states[stage + 1U].reference[model::kVelocityIndex] =
      body.forward_velocity_mps;
  }
  if (candidate.request.states.back().reference[model::kVelocityIndex] > 0.0) {
    // A normal horizon can end before braking reaches rest. Overwriting only
    // its terminal velocity would contradict the final acceleration/dynamics.
    // Retiming also changes every peer prediction, so this producer must not
    // silently extend the immutable source clock to manufacture a Stop.
    return reject_stop(
      Reason::InvalidBrakingEnvelope,
      "maximum-braking Stop horizon ends before terminal rest");
  }
  candidate.request.states.back().lower[model::kVelocityIndex] = 0.0;
  candidate.request.states.back().upper[model::kVelocityIndex] = 0.0;
  // The maximum-braking law is already hard. Stop's remaining task is to
  // find a physically clear trajectory through rest; continuing to optimize
  // the inherited racing-line/progress objective gives it a different mission.
  // References remain numerical seeds and every source hard bound is retained.
  for (auto & state : candidate.request.states) {
    state.weight.setZero();
    state.linear_cost.setZero();
  }
  for (auto & input : candidate.request.inputs) {
    input.weight.setZero();
    input.linear_cost.setZero();
  }
  candidate.request.input_delta_weight.setZero();
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject_stop(
      Reason::InvalidSource,
      std::string{source_description} + " interaction snapshot incomplete");
  }

  StopCandidateResult result;
  result.reason = Reason::Accepted;
  result.candidate = std::move(candidate);
  result.detail = "accepted";
  return result;
}

} // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Accepted:
      return "accepted";
    case Reason::InvalidSource:
      return "invalid-source";
    case Reason::PublisherBoundaryUnavailable:
      return "publisher-boundary-unavailable";
    case Reason::ReplayWorldUnavailable:
      return "replay-world-unavailable";
    case Reason::InvalidBrakingEnvelope:
      return "invalid-braking-envelope";
    case Reason::InvalidSchedule:
      return "invalid-schedule";
    case Reason::Count:
      break;
  }
  return "unknown";
}

StopCandidateResult build_maximum_braking_candidate(
  const shadow::Snapshot & source,
  const artifact::ExecutionArtifact & normal_execution,
  const persistent_osqp::PhysicalConstraintTolerance
  & solver_tolerance) noexcept
{
  if (source.request.states.empty() || source.request.inputs.empty() ||
    source.request.states.size() != source.request.inputs.size() + 1U ||
    artifact::validate(normal_execution) != artifact::RejectReason::None)
  {
    return reject_stop(
      Reason::InvalidSource,
      "invalid source state/input shape");
  }

  const auto cursor = artifact::resolve_cursor(
    normal_execution, normal_execution.prediction_origin_sec);
  const auto continuation = physical::build_continuation(
    normal_execution, cursor,
    physical::ContinuationInitialState{
      source.request.initial_state[model::kLateralIndex],
      source.request.initial_state[model::kLagIndex],
      source.request.initial_state[model::kHeadingIndex],
      source.request.initial_state[model::kVelocityIndex],
      source.request.initial_state[model::kProgressIndex],
      source.request.current_steering_rad,
      source.request.current_response_steering_rad,
      source.request.current_lateral_velocity_mps, source.request.current_yaw_rate_radps});
  if (!continuation.exact_trajectory.has_value() ||
    !std::isfinite(continuation.publisher_interval_end_steering_rad) ||
    !std::isfinite(
      continuation.publisher_interval_end_response_steering_rad))
  {
    std::ostringstream detail;
    detail << "publisher-boundary continuation unavailable/reason="
           << physical::to_string(continuation.reason)
           << "/scope=" << physical::to_string(continuation.scope)
           << "/exact=" << (continuation.exact_trajectory.has_value() ? 1 : 0)
           << "/steering=" << continuation.publisher_interval_end_steering_rad
           << "/response="
           << continuation.publisher_interval_end_response_steering_rad;
    return reject_stop(Reason::PublisherBoundaryUnavailable, detail.str());
  }

  const auto prefix_sample =
    std::lower_bound(
    continuation.exact_trajectory->elapsed_time_sec.begin(),
    continuation.exact_trajectory->elapsed_time_sec.end(),
    source.publication_interval_sec - 1e-12);
  if (prefix_sample == continuation.exact_trajectory->elapsed_time_sec.end()) {
    return reject_stop(
      Reason::PublisherBoundaryUnavailable,
      "publisher-boundary sample unavailable");
  }
  const auto prefix_index = static_cast<std::size_t>(std::distance(
      continuation.exact_trajectory->elapsed_time_sec.begin(), prefix_sample));
  if (std::abs(*prefix_sample - source.publication_interval_sec) > 1e-9) {
    return reject_stop(
      Reason::PublisherBoundaryUnavailable,
      "publisher-boundary sample timestamp mismatch");
  }
  const auto & prefix = continuation.exact_trajectory.value();
  const double prefix_progress_m = prefix.progress_m[prefix_index];
  if (!std::isfinite(prefix_progress_m)) {
    return reject_stop(
      Reason::PublisherBoundaryUnavailable,
      "publisher-boundary progress non-finite");
  }

  auto candidate = source;
  candidate.control_prediction_origin_sec += source.publication_interval_sec;
  if (candidate.terminal_stop_course_geometry) {
    for (auto & progress : candidate.terminal_stop_course_geometry->progress_m) {
      progress += source.course_progress_origin_m - prefix_progress_m;
    }
  }
  candidate.course_progress_origin_m = prefix_progress_m;
  candidate.request.course_frame.progress_origin_m = prefix_progress_m;
  candidate.request.initial_state[model::kLateralIndex] =
    prefix.lateral_m[prefix_index];
  candidate.request.initial_state[model::kLagIndex] =
    prefix.lag_m[prefix_index];
  candidate.request.initial_state[model::kHeadingIndex] =
    prefix.heading_offset_rad[prefix_index];
  candidate.request.initial_state[model::kVelocityIndex] =
    prefix.velocity_mps[prefix_index];
  candidate.request.initial_state[model::kProgressIndex] = 0.0;
  // The common semantic adapter owns the state-zero equality and rebases its
  // box from this publisher-boundary initial state.  Candidate producers must
  // not duplicate that ownership with a second set of stage-zero bounds.
  candidate.request.current_steering_rad =
    continuation.publisher_interval_end_steering_rad;
  candidate.request.current_response_steering_rad =
    continuation.publisher_interval_end_response_steering_rad;
  candidate.request.current_lateral_velocity_mps =
    continuation.publisher_interval_end_lateral_velocity_mps;
  candidate.request.current_yaw_rate_radps = continuation.publisher_interval_end_yaw_rate_radps;
  candidate.request.previous_input[model::kAccelerationIndex] =
    normal_execution.control_stages.front().acceleration_mps2;
  candidate.request.previous_input[model::kSteeringRateIndex] =
    normal_execution.control_stages.front().steering_rate_radps;
  candidate.request.previous_input[model::kVirtualProgressSpeedIndex] =
    normal_execution.control_stages.front().virtual_progress_speed_mps;
  const double progress_shift_m =
    prefix_progress_m - source.course_progress_origin_m;
  for (auto & obstacle : candidate.dynamic_obstacle_stages) {
    if (obstacle.valid) {
      obstacle.target_progress_m -= progress_shift_m;
    }
  }
  if (!candidate.replay_world.has_value()) {
    return reject_stop(
      Reason::ReplayWorldUnavailable,
      "replay world unavailable at publisher boundary");
  }
  const double old_control_origin_age_sec =
    source.control_prediction_origin_sec -
    candidate.replay_world->observed_sec;
  for (std::size_t sample = 0U; sample <= prefix_index; ++sample) {
    const auto frame = mpc_stage_geometry::sample_course_frame(
      candidate.wall_course_frame_knots, prefix.progress_m[sample],
      std::max(1e-9, candidate.replay_world->bound_tolerance_m));
    if (!frame.has_value()) {
      return reject_stop(
        Reason::PublisherBoundaryUnavailable,
        "publisher-boundary course frame unavailable");
    }
    const auto pose = contract::reconstruct_planar_pose_from_frenet(
      contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
      contract::FrenetPose{prefix.lateral_m[sample], prefix.lag_m[sample],
        prefix.heading_offset_rad[sample]});
    if (!pose.has_value()) {
      return reject_stop(
        Reason::PublisherBoundaryUnavailable,
        "publisher-boundary world pose unavailable");
    }
    candidate.replay_world->control_prefix.push_back(
      recovery::Pose2D{pose->x_m, pose->y_m, pose->yaw_rad});
    candidate.replay_world->control_prefix_elapsed_sec.push_back(
      old_control_origin_age_sec + prefix.elapsed_time_sec[sample]);
  }

  return impose_maximum_braking_law(
    std::move(candidate), solver_tolerance, "publisher-boundary Stop");
}

StopCandidateResult build_current_world_maximum_braking_candidate(
  const shadow::Snapshot & source,
  const persistent_osqp::PhysicalConstraintTolerance
  & solver_tolerance) noexcept
{
  if (!architecture::interaction_snapshot_complete(source)) {
    return reject_stop(
      Reason::InvalidSource,
      "current-world interaction snapshot incomplete");
  }
  // No cursor/rebase is allowed here.  The source was built after the exact
  // serialized predecessor and its replay prefix already ends at this
  // control prediction origin.
  return impose_maximum_braking_law(
    source, solver_tolerance, "current-world Stop");
}

static StopCandidateResult build_complete_rest_candidate(
  const shadow::Snapshot & source,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance,
  const double dt) noexcept
{
  if (!architecture::interaction_snapshot_complete(source)) {
    return reject_stop(Reason::InvalidSource, "current-world interaction snapshot incomplete");
  }
  auto candidate = source;
  auto & request = candidate.request;
  if (!std::isfinite(dt) || dt < request.minimum_stage_dt_sec ||
    !std::isfinite(request.initial_state[model::kVelocityIndex]) ||
    request.initial_state[model::kVelocityIndex] < 0.0 ||
    request.states.back().lower[model::kVelocityIndex] > 0.0 ||
    request.states.back().upper[model::kVelocityIndex] < 0.0)
  {
    return reject_stop(Reason::InvalidBrakingEnvelope, "source bounds exclude complete rest");
  }
  if (dt > request.maximum_stage_dt_sec) {
    return reject_stop(Reason::InvalidBrakingEnvelope, "Stop clock exceeds source support");
  }
  request.maximum_stage_dt_sec = dt;
  // Stop asks whether a complete resting trajectory exists. Racing rewards
  // must not drive an otherwise free feasibility trajectory during this task.
  for (auto & state : request.states) {
    state.weight.setZero();
    state.linear_cost.setZero();
  }
  for (auto & input : request.inputs) {
    input.weight.setZero();
    input.linear_cost.setZero();
  }
  request.input_delta_weight.setZero();
  mpcc_vehicle_model::State braking_body{0, 0, 0,
    request.initial_state[model::kVelocityIndex], request.current_lateral_velocity_mps,
    request.current_yaw_rate_radps, request.current_steering_rad,
    request.current_response_steering_rad};
  for (auto & input : request.inputs) {
    const auto bounds = mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
      input.lower[model::kAccelerationIndex], input.upper[model::kAccelerationIndex],
      solver_tolerance);
    if (!bounds || bounds->lower >= 0.0) {
      return reject_stop(Reason::InvalidBrakingEnvelope, "complete-rest braking bound unavailable");
    }
    input.stage_dt_sec = dt;
    const auto next = mpcc_vehicle_model::advance(
      braking_body, {bounds->lower, 0.0}, request.vehicle_model, dt);
    if (!next) {
      return reject_stop(Reason::InvalidBrakingEnvelope, "complete-rest body transition rejected");
    }
    braking_body = next->state;
  }
  if (braking_body.forward_velocity_mps != 0.0 || braking_body.lateral_velocity_mps != 0.0 ||
    braking_body.yaw_rate_radps != 0.0) {
    return reject_stop(Reason::InvalidBrakingEnvelope, "source maximum stage clock ends before rest");
  }
  candidate.execution_prefix_steps = request.horizon_steps;
  // A freely optimized progress state must stay in the supplied wall/course
  // domain. A source can declare wider boxes than its wall profile because
  // its old prescribed braking trajectory never reached that boundary.
  // Intersect hard boxes before solving; do not clamp an accepted trajectory
  // or extend geometry beyond the immutable observation.
  const double progress_lower = std::max(candidate.wall_reference_progress_m.front(),
    candidate.wall_course_frame_knots.front().progress_m - candidate.course_progress_origin_m);
  const double progress_upper = std::min(candidate.wall_reference_progress_m.back(),
    candidate.wall_course_frame_knots.back().progress_m - candidate.course_progress_origin_m);
  for (std::size_t stage = 1U; stage < request.states.size(); ++stage) {
    auto & state = request.states[stage];
    state.lower[model::kProgressIndex] = std::max(state.lower[model::kProgressIndex], progress_lower);
    state.upper[model::kProgressIndex] = std::min(state.upper[model::kProgressIndex], progress_upper);
    if (state.lower[model::kProgressIndex] > state.upper[model::kProgressIndex]) {
      return reject_stop(Reason::InvalidSource, "complete-rest progress domain is empty");
    }
  }
  // Nominal settled-contact rest is invariant only with nonpositive wire
  // input. A tiny positive QP input can satisfy a numerical zero-speed row
  // while immediately leaving rest in the native body model. Declare the
  // terminal mode's input condition before the common solver inset; never
  // round or clamp an accepted serialized command after solving.
  request.inputs.back().upper[model::kAccelerationIndex] = std::min(
    request.inputs.back().upper[model::kAccelerationIndex], 0.0);
  request.states.back().reference[model::kVelocityIndex] = 0.0;
  request.states.back().lower[model::kVelocityIndex] = 0.0;
  request.states.back().upper[model::kVelocityIndex] = 0.0;
  request.course_frame = {
    std::make_shared<const std::vector<mpc_stage_geometry::CourseFrameKnot>>(
      candidate.wall_course_frame_knots), candidate.course_progress_origin_m};

  const auto & world = *candidate.replay_world;
  auto & context = candidate.identity.source_context;
  const shadow::ReplayDynamicObstacle * primary = nullptr;
  std::vector<std::string> observed_ids;
  for (const auto & peer : world.obstacles) {
    if (peer.id.empty() || peer.observation_generation != world.observation_generation ||
      !std::isfinite(peer.radius_m) || peer.radius_m < 0.0 ||
      !std::isfinite(peer.x_m) || !std::isfinite(peer.y_m) ||
      !std::isfinite(peer.velocity_x_mps) || !std::isfinite(peer.velocity_y_mps) ||
      std::find(observed_ids.begin(), observed_ids.end(), peer.id) != observed_ids.end())
    {
      return reject_stop(Reason::InvalidSource, "complete-rest peer provenance invalid");
    }
    observed_ids.push_back(peer.id);
    if (context.dynamic_obstacle_constraint_active ? peer.id == context.dynamic_obstacle_id :
      (primary == nullptr || peer.id < primary->id))
    {
      primary = &peer;
    }
  }
  if (context.dynamic_obstacle_constraint_active && primary == nullptr) {
    return reject_stop(Reason::InvalidSource, "complete-rest primary peer absent");
  }
  if (primary != nullptr) {
    if (!world.current || world.observation_generation == 0U) {
      return reject_stop(Reason::InvalidSource, "complete-rest peer world unavailable");
    }
    if (!context.dynamic_obstacle_constraint_active) {
      candidate.dynamic_obstacle_pass_side_sign = 0;
      context.dynamic_obstacle_side_sign = 0;
    }
    candidate.dynamic_obstacle_refinement_active = true;
    context.dynamic_obstacle_constraint_active = true;
    context.dynamic_obstacle_id = primary->id;
    context.dynamic_obstacle_generation = world.observation_generation;
    candidate.dynamic_obstacle_stages.clear();
    double elapsed = candidate.control_prediction_origin_sec - world.observed_sec;
    for (int stage = 0; stage < request.horizon_steps; ++stage) {
      elapsed += request.inputs[stage].stage_dt_sec;
      const auto frame = mpc_stage_geometry::sample_course_frame(
        candidate.wall_course_frame_knots,
        candidate.course_progress_origin_m + candidate.wall_reference_progress_m[stage + 1]);
      if (!frame) {
        return reject_stop(Reason::InvalidSource, "complete-rest peer stage frame unavailable");
      }
      const auto relative = contract::project_planar_pose_to_frenet(
        {primary->x_m + primary->velocity_x_mps * elapsed,
          primary->y_m + primary->velocity_y_mps * elapsed, frame->heading_rad},
        {frame->x_m, frame->y_m, frame->heading_rad});
      if (!relative) {
        return reject_stop(Reason::InvalidSource, "complete-rest peer projection unavailable");
      }
      candidate.dynamic_obstacle_stages.push_back({true,
        candidate.wall_reference_progress_m[stage + 1] + relative->lag_m, relative->lateral_m,
        world.physical_footprint.front_extent_m + world.physical_footprint.margin_m + primary->radius_m,
        world.physical_footprint.left_extent_m + world.physical_footprint.margin_m + primary->radius_m});
    }
  }
  // Both terminal meaning and the changed clock belong to this candidate;
  // neither may alias the rolling normal source in warm starts or artifacts.
  // The numerical clock is also an artifact/warm-start identity, not just
  // part of the larger interaction snapshot fingerprint.
  std::uint64_t stage_dt_bits{};
  static_assert(sizeof(stage_dt_bits) == sizeof(dt));
  std::memcpy(&stage_dt_bits, &dt, sizeof(dt));
  context.bounds_schema_id += "/terminal-dt-bits-" + std::to_string(stage_dt_bits);
  context.bounds_schema_id += shadow::kCompleteRestBoundsSchemaSuffix;
  context.cost_schema_id += "/zero-cost-rest-feasibility-v1";
  context = contract::seal_problem_context(context);
  if (!architecture::interaction_snapshot_complete(candidate)) {
    return reject_stop(Reason::InvalidSource, "complete-rest interaction snapshot incomplete");
  }
  StopCandidateResult result;
  result.reason = Reason::Accepted;
  result.candidate = std::move(candidate);
  result.detail = "accepted/free-controls-through-rest";
  return result;
}

StopCandidateResult build_current_world_complete_rest_candidate(
  const shadow::Snapshot & source,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance) noexcept
{
  return build_complete_rest_candidate(source, solver_tolerance,
    source.request.maximum_stage_dt_sec);
}

std::vector<StopCandidateResult> build_current_world_complete_rest_population(
  const shadow::Snapshot & source,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance)
{
  auto maximum = build_current_world_complete_rest_candidate(source, solver_tolerance);
  if (!maximum.accepted()) {
    return {std::move(maximum)};
  }
  const auto & request = source.request;
  // Use the weakest available braking bound so the candidate clock does not
  // assume an input that a later stage excludes. This is a nominal seed only;
  // the solved controls and complete physical rest still require certification.
  double braking = -std::numeric_limits<double>::infinity();
  for (const auto & input : request.inputs) {
    const auto bounds = mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
      input.lower[model::kAccelerationIndex], input.upper[model::kAccelerationIndex],
      solver_tolerance);
    if (!bounds) {
      return {std::move(maximum)};
    }
    braking = std::max(braking, bounds->lower);
  }
  mpcc_vehicle_model::State body{0, 0, 0,
    request.initial_state[model::kVelocityIndex], request.current_lateral_velocity_mps,
    request.current_yaw_rate_radps, request.current_steering_rad,
    request.current_response_steering_rad};
  const double step = model::kMaximumPhysicalIntegrationStepSec;
  const double maximum_duration = request.horizon_steps * request.maximum_stage_dt_sec;
  double elapsed = 0.0;
  const auto resting = [](const auto & state) {
      return state.forward_velocity_mps == 0.0 && state.lateral_velocity_mps == 0.0 &&
             state.yaw_rate_radps == 0.0;
    };
  while (!resting(body) && elapsed + step <= maximum_duration + 1e-9) {
    const auto next = mpcc_vehicle_model::advance(body, {braking, 0.0},
      request.vehicle_model, step);
    if (!next) {
      return {std::move(maximum)};
    }
    body = next->state;
    elapsed += step;
  }
  if (!resting(body)) {
    return {std::move(maximum)};
  }
  const double nominal_dt = std::max({request.minimum_stage_dt_sec,
    source.publication_interval_sec,
    std::ceil(elapsed / request.horizon_steps / step) * step});
  if (nominal_dt >= request.maximum_stage_dt_sec) {
    return {std::move(maximum)};
  }
  auto nominal = build_complete_rest_candidate(source, solver_tolerance, nominal_dt);
  if (!nominal.accepted()) {
    return {std::move(maximum)};
  }
  nominal.detail = "native-braking-clock/" + nominal.detail;
  maximum.detail = "maximum-support-clock/" + maximum.detail;
  return {std::move(nominal), std::move(maximum)};
}

ScheduleResult build_schedule(
  const shadow::Snapshot & maximum_braking_stop,
  const int initial_rate_sign,
  const int first_switch_stage,
  const int second_switch_stage,
  const persistent_osqp::PhysicalConstraintTolerance
  & solver_tolerance) noexcept
{
  if ((initial_rate_sign != -1 && initial_rate_sign != 1) ||
    first_switch_stage <= 0 || second_switch_stage <= first_switch_stage ||
    second_switch_stage >= maximum_braking_stop.request.horizon_steps ||
    maximum_braking_stop.request.states.size() !=
    maximum_braking_stop.request.inputs.size() + 1U)
  {
    return reject_schedule(
      Reason::InvalidSchedule,
      "invalid Stop control lattice schedule");
  }

  double steering_rad = maximum_braking_stop.request.current_steering_rad;
  if (!std::isfinite(steering_rad)) {
    return reject_schedule(
      Reason::InvalidSchedule,
      "initial Stop lattice steering non-finite");
  }
  const auto prefix_bounds = mpcc_rate_resolved_adapter::resolve_steering_prefix_bounds(
    steering_rad, maximum_braking_stop.request.maximum_abs_steering_rad, solver_tolerance);
  if (!prefix_bounds.has_value()) {
    return reject_schedule(Reason::InvalidSchedule, "Stop lattice steering prefix unavailable");
  }
  const double initial_steering_rad = steering_rad;
  ScheduleResult result;
  result.schedule.initial_rate_sign = initial_rate_sign;
  result.schedule.first_switch_stage = first_switch_stage;
  result.schedule.second_switch_stage = second_switch_stage;
  result.schedule.steering_rate_radps.reserve(
    maximum_braking_stop.request.inputs.size());
  for (std::size_t stage = 0U;
    stage < maximum_braking_stop.request.inputs.size(); ++stage)
  {
    const auto & input = maximum_braking_stop.request.inputs[stage];
    if (!std::isfinite(input.stage_dt_sec) || input.stage_dt_sec <= 0.0) {
      return reject_schedule(
        Reason::InvalidSchedule,
        "invalid Stop lattice stage duration");
    }
    const double physical_rate_lower =
      std::max(
      -maximum_braking_stop.request.maximum_abs_steering_rate_radps,
      (-maximum_braking_stop.request.maximum_abs_steering_rad -
      steering_rad) /
      input.stage_dt_sec);
    const double physical_rate_upper = std::min(
      maximum_braking_stop.request.maximum_abs_steering_rate_radps,
      (maximum_braking_stop.request.maximum_abs_steering_rad - steering_rad) /
      input.stage_dt_sec);
    const auto steering_rate_bounds =
      mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
      physical_rate_lower, physical_rate_upper, solver_tolerance);
    if (!steering_rate_bounds.has_value()) {
      return reject_schedule(
        Reason::InvalidSchedule,
        "Stop lattice steering-rate inset unavailable");
    }
    const int stage_index = static_cast<int>(stage);
    const int rate_sign =
      stage_index < first_switch_stage ?
      initial_rate_sign :
      (stage_index < second_switch_stage ? -initial_rate_sign : 0);
    // A rate inset is measured in rad/s; it cannot also stand in for the
    // cumulative radian inset enforced by the QP. Intersect both contracts
    // before fixing the input, otherwise saturation makes the QP infeasible.
    const double cumulative_delta_rad = steering_rad - initial_steering_rad;
    const double lower_rate = std::max(steering_rate_bounds->lower,
      (prefix_bounds->lower - cumulative_delta_rad) / input.stage_dt_sec);
    const double upper_rate = std::min(steering_rate_bounds->upper,
      (prefix_bounds->upper - cumulative_delta_rad) / input.stage_dt_sec);
    if (lower_rate > upper_rate) {
      return reject_schedule(Reason::InvalidSchedule, "Stop lattice rate/prefix intersection empty");
    }
    const double steering_rate_radps =
      rate_sign > 0 ? upper_rate :
      (rate_sign < 0 ? lower_rate : std::clamp(0.0, lower_rate, upper_rate));
    const double next_steering_rad =
      steering_rad + steering_rate_radps * input.stage_dt_sec;
    if (!std::isfinite(next_steering_rad) ||
      std::abs(next_steering_rad) >
      maximum_braking_stop.request.maximum_abs_steering_rad + 1e-9)
    {
      return reject_schedule(
        Reason::InvalidSchedule,
        "Stop lattice steering state outside actuator bounds");
    }
    result.schedule.steering_rate_radps.push_back(steering_rate_radps);
    steering_rad = next_steering_rad;
  }
  result.reason = Reason::Accepted;
  result.detail = "accepted";
  return result;
}

std::vector<ScheduleResult> build_population(
  const shadow::Snapshot & maximum_braking_stop,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance)
{
  const int horizon_steps = maximum_braking_stop.request.horizon_steps;
  std::vector<int> first_switch_stages;
  std::vector<int> second_switch_stages;
  const auto append_stage = [horizon_steps](std::vector<int> & stages,
      const double fraction) {
      const int stage =
        std::clamp(
        static_cast<int>(std::lround(fraction * horizon_steps)), 1,
        std::max(1, horizon_steps - 1));
      if (std::find(stages.begin(), stages.end(), stage) == stages.end()) {
        stages.push_back(stage);
      }
    };
  for (const double fraction : {0.10, 0.15, 0.20, 0.25, 0.30}) {
    append_stage(first_switch_stages, fraction);
  }
  for (const double fraction : {0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60}) {
    append_stage(second_switch_stages, fraction);
  }

  std::vector<ScheduleResult> population;
  for (const int initial_rate_sign : {1, -1}) {
    for (const int first_switch_stage : first_switch_stages) {
      for (const int second_switch_stage : second_switch_stages) {
        if (second_switch_stage > first_switch_stage) {
          population.push_back(
            build_schedule(
              maximum_braking_stop, initial_rate_sign, first_switch_stage,
              second_switch_stage, solver_tolerance));
        }
      }
    }
  }
  return population;
}

OrderedPopulation build_anytime_population(
  const shadow::Snapshot & maximum_braking_stop,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance)
{
  const auto legacy = build_population(maximum_braking_stop, solver_tolerance);
  OrderedPopulation result;
  result.preferred_initial_rate_sign =
    preferred_initial_rate_sign(maximum_braking_stop);
  result.candidates.reserve(legacy.size());
  result.legacy_rank_by_candidate.reserve(legacy.size());

  std::vector<ScheduleGeometry> geometries;
  std::vector<std::size_t> deferred_indices;
  for (std::size_t index = 0U; index < legacy.size(); ++index) {
    const auto & candidate = legacy[index];
    if (!candidate.accepted()) {
      deferred_indices.push_back(index);
      continue;
    }
    const auto found = std::find_if(
      geometries.begin(), geometries.end(),
      [&candidate](const ScheduleGeometry & geometry) {
        return
          geometry.first_switch_stage ==
          candidate.schedule.first_switch_stage &&
          geometry.second_switch_stage ==
          candidate.schedule.second_switch_stage;
      });
    auto * geometry = found == geometries.end() ? nullptr : &(*found);
    if (geometry == nullptr) {
      geometries.push_back(ScheduleGeometry{
        candidate.schedule.first_switch_stage,
        candidate.schedule.second_switch_stage, index, std::nullopt,
        std::nullopt});
      geometry = &geometries.back();
    }
    auto & sign_index = candidate.schedule.initial_rate_sign > 0 ?
      geometry->positive_legacy_index : geometry->negative_legacy_index;
    if (sign_index.has_value()) {
      // Preserve unexpected duplicates exactly, but keep them out of the
      // geometry traversal so the permutation remains one-to-one.
      deferred_indices.push_back(index);
    } else {
      sign_index = index;
    }
  }

  std::vector<std::size_t> geometry_order;
  geometry_order.reserve(geometries.size());
  std::vector<bool> selected(geometries.size(), false);
  if (!geometries.empty()) {
    std::size_t first = 0U;
    double first_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < geometries.size(); ++index) {
      const double distance = nominal_geometry_distance_squared(
        geometries[index], maximum_braking_stop.request.horizon_steps);
      if (
        distance < first_distance - 1e-15 ||
        (std::abs(distance - first_distance) <= 1e-15 &&
        geometries[index].first_legacy_index <
        geometries[first].first_legacy_index))
      {
        first = index;
        first_distance = distance;
      }
    }
    selected[first] = true;
    geometry_order.push_back(first);
  }
  while (geometry_order.size() < geometries.size()) {
    std::size_t next = geometries.size();
    double best_minimum_distance = -1.0;
    for (std::size_t index = 0U; index < geometries.size(); ++index) {
      if (selected[index]) {
        continue;
      }
      double minimum_distance = std::numeric_limits<double>::infinity();
      for (const auto selected_index : geometry_order) {
        minimum_distance = std::min(
          minimum_distance,
          normalized_geometry_distance_squared(
            geometries[index], geometries[selected_index],
            maximum_braking_stop.request.horizon_steps));
      }
      if (
        next == geometries.size() ||
        minimum_distance > best_minimum_distance + 1e-15 ||
        (std::abs(minimum_distance - best_minimum_distance) <= 1e-15 &&
        geometries[index].first_legacy_index <
        geometries[next].first_legacy_index))
      {
        next = index;
        best_minimum_distance = minimum_distance;
      }
    }
    if (next == geometries.size()) {
      break;
    }
    selected[next] = true;
    geometry_order.push_back(next);
  }

  const auto append_legacy = [&result, &legacy](
      const std::optional<std::size_t> index) {
      if (!index.has_value()) {
        return;
      }
      result.candidates.push_back(legacy[index.value()]);
      result.legacy_rank_by_candidate.push_back(index.value() + 1U);
    };
  for (const auto geometry_index : geometry_order) {
    const auto & geometry = geometries[geometry_index];
    if (result.preferred_initial_rate_sign > 0) {
      append_legacy(geometry.positive_legacy_index);
      append_legacy(geometry.negative_legacy_index);
    } else {
      append_legacy(geometry.negative_legacy_index);
      append_legacy(geometry.positive_legacy_index);
    }
  }
  for (const auto index : deferred_indices) {
    result.candidates.push_back(legacy[index]);
    result.legacy_rank_by_candidate.push_back(index + 1U);
  }
  return result;
}

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_stop_control_lattice
