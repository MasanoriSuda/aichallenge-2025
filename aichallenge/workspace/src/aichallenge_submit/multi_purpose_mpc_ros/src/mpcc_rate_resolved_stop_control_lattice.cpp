#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_control_lattice.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <cmath>
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
  double elapsed_sec = 0.0;
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
    const double stage_start_velocity_mps = std::max(
      0.0, initial_velocity_mps + minimum_acceleration_mps2 * elapsed_sec);
    elapsed_sec += input.stage_dt_sec;
    const double stage_end_velocity_mps = std::max(
      0.0, initial_velocity_mps + minimum_acceleration_mps2 * elapsed_sec);
    const double required_acceleration_mps2 =
      (stage_end_velocity_mps - stage_start_velocity_mps) /
      input.stage_dt_sec;
    if (required_acceleration_mps2 <
      input.lower[model::kAccelerationIndex] - 1e-9 ||
      required_acceleration_mps2 >
      input.upper[model::kAccelerationIndex] + 1e-9)
    {
      return reject_stop(
        Reason::InvalidBrakingEnvelope,
        "maximum-braking Stop acceleration outside source bounds");
    }
    input.reference[model::kAccelerationIndex] = required_acceleration_mps2;
    input.reference[model::kVirtualProgressSpeedIndex] =
      stage_start_velocity_mps;
    auto & next_state = candidate.request.states[stage + 1U];
    next_state.reference[model::kVelocityIndex] = stage_end_velocity_mps;
    next_state.lower[model::kVelocityIndex] = stage_end_velocity_mps;
    next_state.upper[model::kVelocityIndex] = stage_end_velocity_mps;
  }
  auto & terminal = candidate.request.states.back();
  terminal.reference[model::kVelocityIndex] = 0.0;
  terminal.lower[model::kVelocityIndex] = 0.0;
  terminal.upper[model::kVelocityIndex] = 0.0;
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
      source.request.current_response_steering_rad});
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
  candidate.course_progress_origin_m = prefix_progress_m;
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
    const double steering_rate_radps =
      rate_sign > 0 ? steering_rate_bounds->upper :
      (rate_sign < 0 ? steering_rate_bounds->lower : 0.0);
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
