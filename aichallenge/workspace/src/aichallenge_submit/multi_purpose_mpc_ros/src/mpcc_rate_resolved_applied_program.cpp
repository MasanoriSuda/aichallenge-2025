#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include "multi_purpose_mpc_ros/mpcc_wire_command.hpp"

#include "multi_purpose_mpc_ros/detail/mpcc_footprint_enclosure.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_applied_program {
namespace {
namespace numeric = vehicle::numerical;
namespace recovery = recovery_footprint;
namespace physical = mpcc_rate_resolved_physical_wall;
using numeric::I;

numeric::Box box(const vehicle::BodyRanges &values) {
  numeric::Box result;
  for (std::size_t i = 0; i < values.size(); ++i)
    result[i] = {values[i].lower, values[i].upper};
  return result;
}

I square(I a) {
  const double maximum = std::max(std::abs(a.lo), std::abs(a.hi));
  const double minimum =
      a.lo <= 0 && a.hi >= 0 ? 0 : std::min(std::abs(a.lo), std::abs(a.hi));
  return {minimum == 0 ? 0 : numeric::down(minimum * minimum),
          numeric::up(maximum * maximum)};
}

/// Enclose nearest-polyline progress for every point in this Cartesian box.
/// Keep every segment which might be nearest; a crossing never selects one
/// convenient branch. Queries possibly outside the recorded window reject.
std::optional<I>
course_progress(const numeric::Box &b, const vehicle::State &origin,
                const std::vector<mpc_stage_geometry::CourseFrameKnot> &knots) {
  if (knots.size() < 2)
    return std::nullopt;
  const I c = numeric::cosine(I(origin.yaw_rad)),
          s = numeric::sine(I(origin.yaw_rad));
  struct Projection {
    I progress, distance2, fraction;
    std::size_t index;
  };
  std::vector<Projection> candidates;
  double best_upper = INFINITY;
  for (std::size_t i = 0; i + 1 < knots.size(); ++i) {
    const auto &a = knots[i];
    const auto &z = knots[i + 1];
    if (!std::isfinite(a.progress_m) || !std::isfinite(z.progress_m) ||
        z.progress_m <= a.progress_m || !std::isfinite(a.x_m) ||
        !std::isfinite(a.y_m) || !std::isfinite(z.x_m) || !std::isfinite(z.y_m))
      return std::nullopt;
    const I dx = I(origin.x_m) - I(a.x_m) + c * b[0] - s * b[1];
    const I dy = I(origin.y_m) - I(a.y_m) + s * b[0] + c * b[1];
    const I lx = I(z.x_m) - I(a.x_m), ly = I(z.y_m) - I(a.y_m);
    const I length2 = square(lx) + square(ly);
    if (length2.lo <= 0 || !std::isfinite(length2.hi))
      return std::nullopt;
    const I dot = dx * lx + dy * ly;
    const I fraction =
        dot * I(numeric::down(1 / length2.hi), numeric::up(1 / length2.lo));
    const I t{std::clamp(fraction.lo, 0.0, 1.0),
              std::clamp(fraction.hi, 0.0, 1.0)};
    const I distance2 = square(dx - t * lx) + square(dy - t * ly);
    const I progress =
        I(a.progress_m) + t * (I(z.progress_m) - I(a.progress_m));
    candidates.push_back({progress, distance2, fraction, i});
    best_upper = std::min(best_upper, distance2.hi);
  }
  std::optional<I> result;
  for (const auto &candidate : candidates) {
    if (candidate.distance2.lo > best_upper)
      continue;
    if ((candidate.index == 0 && candidate.fraction.lo < 0) ||
        (candidate.index + 2 == knots.size() && candidate.fraction.hi > 1))
      return std::nullopt;
    result = result ? numeric::hull(*result, candidate.progress)
                    : candidate.progress;
  }
  return result;
}

std::uint64_t nominal_fingerprint(const retained::Proof &proof) {
  std::uint64_t hash = 14695981039346656037ULL;
  const auto number = [&](double value) {
    std::uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned shift = 0; shift < 64; shift += 8)
      hash =
          (hash ^ static_cast<unsigned char>(bits >> shift)) * 1099511628211ULL;
  };
  for (const double value :
       {proof.actuation.predicted_speed_mps, proof.actuation.acceleration_mps2,
        proof.actuation.steering_rate_radps, proof.actuation.steering_rad,
        proof.actuation.curvature_radpm,
        proof.actuation.virtual_progress_speed_mps, proof.cursor.elapsed_sec,
        proof.cursor.stage_elapsed_sec,
        proof.lifted_control_origin_physical_progress_m})
    number(value);
  number(proof.terminal_stop_certified);
  number(proof.terminal_stop_uses_solved_suffix);
  number(proof.terminal_stop_constant_steering_program);
  number(proof.terminal_stop_source_horizon_program);
  number(proof.terminal_stop_forward_velocity_ceiling_mps.has_value());
  if (proof.terminal_stop_forward_velocity_ceiling_mps)
    number(*proof.terminal_stop_forward_velocity_ceiling_mps);
  number(proof.terminal_stop_publisher_interval_sample_count);
  number(proof.cursor.control_stage_index);
  number(proof.cursor.remaining_control_stage_count);
  number(proof.terminal_stop_actuation_samples.size());
  for (const auto &sample : proof.terminal_stop_actuation_samples) {
    for (const double value :
         {sample.elapsed_time_sec, sample.duration_sec,
          sample.acceleration_mps2, sample.effective_acceleration_mps2,
          sample.steering_rate_radps, sample.end_velocity_mps,
          sample.end_steering_rad, sample.end_response_steering_rad,
          sample.path_curvature_radpm, sample.virtual_progress_speed_mps,
          sample.end_lateral_velocity_mps, sample.end_yaw_rate_radps})
      number(value);
    number(sample.command_interval_index);
  }
  const auto &trajectory = proof.terminal_stop_trajectory;
  for (const auto *values :
       {&trajectory.elapsed_time_sec, &trajectory.path_distance_m,
        &trajectory.lateral_m, &trajectory.lag_m,
        &trajectory.heading_offset_rad, &trajectory.velocity_mps,
        &trajectory.progress_m, &trajectory.lateral_lower_m,
        &trajectory.lateral_upper_m}) {
    number(values->size());
    for (const double value : *values)
      number(value);
  }
  return hash;
}

bool same_world(const retained::Request &a, const retained::Request &b) {
  if (a.plan != b.plan || a.decision_id != b.decision_id ||
      a.now_sec != b.now_sec || a.control_origin_sec != b.control_origin_sec ||
      a.current_intent != b.current_intent ||
      a.current_wall_grid != b.current_wall_grid ||
      a.obstacles.generation != b.obstacles.generation ||
      a.obstacles.observed_sec != b.obstacles.observed_sec ||
      a.obstacles.current != b.obstacles.current ||
      a.obstacles.obstacles.size() != b.obstacles.obstacles.size() ||
      a.control_origin_physical_progress_m !=
          b.control_origin_physical_progress_m ||
      a.follow_target.has_value() != b.follow_target.has_value() ||
      a.minimum_acceleration_mps2 != b.minimum_acceleration_mps2 ||
      a.maximum_acceleration_mps2 != b.maximum_acceleration_mps2 ||
      a.previous_published_steering_rad != b.previous_published_steering_rad ||
      a.previous_published_command_age_sec !=
          b.previous_published_command_age_sec ||
      a.control_pose.x_m != b.control_pose.x_m ||
      a.control_pose.y_m != b.control_pose.y_m ||
      a.control_pose.yaw_rad != b.control_pose.yaw_rad)
    return false;
  const auto &x = a.current_footprint;
  const auto &y = b.current_footprint;
  if (x.front_extent_m != y.front_extent_m ||
      x.rear_extent_m != y.rear_extent_m ||
      x.left_extent_m != y.left_extent_m ||
      x.right_extent_m != y.right_extent_m || x.margin_m != y.margin_m)
    return false;
  for (std::size_t i = 0; i < a.obstacles.obstacles.size(); ++i) {
    const auto &x = a.obstacles.obstacles[i];
    const auto &y = b.obstacles.obstacles[i];
    if (x.id != y.id || x.circle.x_m != y.circle.x_m ||
        x.circle.y_m != y.circle.y_m ||
        x.circle.velocity_x_mps != y.circle.velocity_x_mps ||
        x.circle.velocity_y_mps != y.circle.velocity_y_mps ||
        x.circle.acceleration_x_mps2 != y.circle.acceleration_x_mps2 ||
        x.circle.acceleration_y_mps2 != y.circle.acceleration_y_mps2 ||
        x.circle.acceleration_horizon_sec !=
            y.circle.acceleration_horizon_sec ||
        x.circle.radius_m != y.circle.radius_m)
      return false;
  }
  if (a.follow_target) {
    const auto &x = *a.follow_target;
    const auto &y = *b.follow_target;
    if (x.target_id != y.target_id ||
        x.observation_generation != y.observation_generation ||
        x.observed_sec != y.observed_sec ||
        x.current_target_gap_m != y.current_target_gap_m ||
        x.hard_gap_m != y.hard_gap_m ||
        x.target_speed_mps != y.target_speed_mps ||
        x.elapsed_time_sec != y.elapsed_time_sec ||
        x.target_progress_from_current_origin_m !=
            y.target_progress_from_current_origin_m ||
        x.current != y.current)
      return false;
  }
  return true;
}
} // namespace

bool Certificate::matches(const retained::Request &request) const noexcept {
  if (!request_ || !same_world(*request_, request) ||
      !request.publication_prefix || !request.plan ||
      !request.plan->execution_artifact)
    return false;
  const auto expected_ceiling = source_horizon_program_ ? retained::source_horizon_velocity_ceiling(request) :
    (request.plan->execution_artifact->applied_stop_program ?
    request.plan->execution_artifact->applied_stop_program->forward_velocity_ceiling_mps : std::nullopt);
  if (expected_ceiling != forward_velocity_ceiling_mps_) return false;
  if (request.applied_program_required && !request.input_application_profile)
    return false;
  if (request.input_application_profile) {
    const auto &profile = *request.input_application_profile;
    if (profile.profile_id != tube_.profile.profile_id ||
        profile.acceleration_age_sec != tube_.profile.acceleration_age_sec ||
        profile.steering_receipt_age_sec !=
            tube_.profile.steering_receipt_age_sec ||
        profile.steering_mechanical_delay_sec !=
            tube_.profile.steering_mechanical_delay_sec)
      return false;
  }
  return vehicle::applied_input_context_fingerprint(
             request.publication_prefix->observation, prepared_.program,
             tube_.profile, request.plan->execution_artifact->vehicle_model) ==
         tube_.context_fingerprint;
}

bool Certificate::matches(const retained::Proof &proof) const noexcept {
  if (!request_ || nominal_fingerprint(proof) != nominal_fingerprint_ ||
      request_->plan != proof.plan ||
      prepared_.decision_id != proof.decision_id ||
      request_->obstacles.generation != proof.obstacle_generation ||
      request_->obstacles.observed_sec != proof.observed_sec ||
      request_->now_sec != proof.observation_origin_sec ||
      request_->control_origin_sec != proof.control_origin_sec ||
      !proof.publication_prefix || !proof.plan ||
      !proof.plan->execution_artifact)
    return false;
  const auto &model = proof.plan->execution_artifact->vehicle_model;
  const auto &packet = prepared_.program.commands.front();
  return packet.wire_acceleration_mps2 ==
             static_cast<float>(proof.actuation.acceleration_mps2) &&
         packet.wire_steering_rad ==
             mpcc_wire_command::steering(proof.actuation.steering_rad,
                                         model.steering_wire_gain) &&
         vehicle::applied_input_context_fingerprint(
             proof.publication_prefix->observation, prepared_.program,
             tube_.profile, model) == tube_.context_fingerprint;
}

Result certify_terminal_stop(const retained::Request &request,
                             const retained::Proof &nominal,
                             const vehicle::InputApplicationProfile &profile) {
  return certify_terminal_stop(request, nominal, profile, nullptr);
}

Result certify_terminal_stop(const retained::Request &request,
                             const retained::Proof &nominal,
                             const vehicle::InputApplicationProfile &profile,
                             const Certificate *materialized_from) {
  Result result;
  if (!request.plan || !request.plan->execution_artifact ||
      !request.plan->physical_snapshot ||
      mpcc_rate_resolved_certified_plan::validate(*request.plan) !=
          mpcc_rate_resolved_certified_plan::RejectReason::None ||
      nominal.plan != request.plan ||
      nominal.decision_id != request.decision_id ||
      nominal.observation_origin_sec != request.now_sec ||
      nominal.control_origin_sec != request.control_origin_sec ||
      nominal.obstacle_generation != request.obstacles.generation ||
      nominal.observed_sec != request.obstacles.observed_sec ||
      !nominal.terminal_stop_certified || !request.publication_prefix ||
      !nominal.publication_prefix ||
      nominal.terminal_stop_actuation_samples.empty())
    return result;
  const auto &execution = *request.plan->execution_artifact;
  const auto &physical_source = *request.plan->physical_snapshot;
  const auto expected_ceiling = nominal.terminal_stop_source_horizon_program ?
    retained::source_horizon_velocity_ceiling(request) :
    (execution.applied_stop_program ? execution.applied_stop_program->forward_velocity_ceiling_mps : std::nullopt);
  if ((nominal.terminal_stop_source_horizon_program &&
    (!nominal.terminal_stop_constant_steering_program || nominal.terminal_stop_uses_solved_suffix || !expected_ceiling)) ||
    nominal.terminal_stop_forward_velocity_ceiling_mps != expected_ceiling) return result;
  if (expected_ceiling) {
    if (!std::isfinite(*expected_ceiling) || *expected_ceiling <= 0 ||
      request.control_origin_speed_mps > *expected_ceiling) return result;
    for (const double velocity : nominal.terminal_stop_trajectory.velocity_mps)
      if (!std::isfinite(velocity) || velocity > *expected_ceiling) return result;
  }
  program::Request input{execution.identity,
                         request.decision_id,
                         request.control_origin_sec,
                         execution.publication_interval_sec,
                         request.publication_prefix->proposed_packet,
                         execution.vehicle_model.steering_wire_gain,
                         request.minimum_acceleration_mps2,
                         request.maximum_acceleration_mps2,
                         execution.maximum_abs_steering_rad,
                         execution.maximum_abs_steering_rate_radps,
                         execution.physical_global_tolerance,
                         nominal.terminal_stop_actuation_samples};
  const auto prepared = program::prepare(input);
  result.program_reason = prepared.reason;
  if (!prepared.prepared) {
    result.reason = Reason::ProgramUnavailable;
    return result;
  }
  if (!request.obstacles.current ||
      !mpcc_rate_resolved_dynamic_proof::observation_valid(request.obstacles) ||
      request.obstacles.observed_sec > request.now_sec ||
      !request.current_wall_grid || !request.current_wall_grid->valid() ||
      !request.current_footprint.valid()) {
    result.reason = Reason::InvalidWorld;
    return result;
  }
  const auto wall_footprint = physical::resolve_clearance_footprint(
      request.current_footprint, physical_source.hard_wall_clearance_m);
  if (!wall_footprint) {
    result.reason = Reason::InvalidWorld;
    return result;
  }
  auto certificate = std::shared_ptr<Certificate>(new Certificate);
  certificate->nominal_fingerprint_ = nominal_fingerprint(nominal);
  certificate->source_horizon_program_ = nominal.terminal_stop_source_horizon_program;
  certificate->forward_velocity_ceiling_mps_ = expected_ceiling;
  certificate->prepared_ = *prepared.prepared;
  certificate->request_ = std::make_shared<const retained::Request>(request);
  const auto & observation = request.publication_prefix->observation;
  // Authenticate the immutable input context before running physical checks.
  // This private object cannot escape until prediction reaches full rest.
  certificate->tube_.profile = profile;
  certificate->tube_.context_fingerprint = vehicle::applied_input_context_fingerprint(
    observation, prepared.prepared->program, profile, execution.vehicle_model);
  if (certificate->tube_.context_fingerprint != 0 && !certificate->matches(nominal))
    return result;
  const auto to_world = [&](const recovery::Pose2D &local) {
    const auto &origin = observation.initial.state;
    return recovery::Pose2D{origin.x_m + std::cos(origin.yaw_rad) * local.x_m -
                                std::sin(origin.yaw_rad) * local.y_m,
                            origin.y_m + std::sin(origin.yaw_rad) * local.x_m +
                                std::cos(origin.yaw_rad) * local.y_m,
                            origin.yaw_rad + local.yaw_rad};
  };
  const auto check = [&](const vehicle::BodyRanges &ranges,
                         const vehicle::FootprintRanges &corners,
                         const double begin, const double end) {
    result.rejected_sec = begin;
    const auto state = box(ranges);
    const double tolerance =
        std::max(1e-9, execution.physical_global_tolerance);
    for (const auto &x : state) {
      if (!std::isfinite(x.lo) || !std::isfinite(x.hi) || x.lo > x.hi)
        return Reason::StateBoundRejected;
    }
    if (state[3].lo < -tolerance ||
        (expected_ceiling && state[3].hi > *expected_ceiling) ||
        std::max(std::abs(state[6].lo), std::abs(state[6].hi)) >
            execution.maximum_abs_steering_rad + tolerance ||
        std::max(std::abs(state[7].lo), std::abs(state[7].hi)) >
            execution.vehicle_model.maximum_wire_steering_rad *
                    execution.vehicle_model.tire_grip +
                tolerance)
      return Reason::StateBoundRejected;
    const auto wall = numeric::footprint(state, *wall_footprint);
    const auto cells = recovery::sample_footprint(
        *request.current_wall_grid, wall.extents, to_world(wall.pose));
    if (!cells.valid || cells.out_of_map) return Reason::WallRejected;
    for (const auto cell : cells.contact_cells)
      if (!numeric::separating_oriented_cell_clearance(state, *wall_footprint, box(corners), *request.current_wall_grid,
            cell, {observation.initial.state.x_m, observation.initial.state.y_m, observation.initial.state.yaw_rad}))
        return Reason::WallRejected;
    const auto ego = numeric::footprint(state, request.current_footprint);
    for (const auto &obstacle : request.obstacles.obstacles) {
      auto circle = obstacle.circle;
      const double t0 = begin - request.obstacles.observed_sec,
                   t1 = end - request.obstacles.observed_sec;
      circle.radius_m = numeric::up(
          circle.radius_m + numeric::up(circle.maximum_speed(t0, t1) *
                                        numeric::up((end - begin) / 2)));
      auto clearance = recovery::circle_obstacle_clearance_at_time(
          ego.extents, to_world(ego.pose), circle, (t0 + t1) / 2);
      if (clearance && *clearance < 0) {
        const auto center = circle.predicted_center((t0 + t1) / 2);
        const auto &origin = observation.initial.state;
        const auto separated = numeric::separating_circle_clearance(
            state, request.current_footprint,
            {origin.x_m, origin.y_m, origin.yaw_rad}, ego,
            center[0], center[1], circle.radius_m);
        if (separated)
          clearance = *separated;
      }
      if (!clearance || *clearance < 0) {
        result.rejected_peer_id = obstacle.id;
        return Reason::PeerRejected;
      }
      certificate->minimum_peer_clearance_m_ =
          std::min(certificate->minimum_peer_clearance_m_, *clearance);
    }
    if (request.follow_target) {
      const auto ego_progress = course_progress(
          state, observation.initial.state, physical_source.course_frame_knots);
      const auto target = retained::follow_target_progress_at(
          *request.follow_target, begin - request.now_sec);
      if (!ego_progress || !target)
        return Reason::FollowProjectionUnavailable;
      const I gap = I(nominal.lifted_control_origin_physical_progress_m) +
                    I(*target) - I(ego_progress->hi);
      certificate->minimum_follow_gap_m_ =
          std::min(certificate->minimum_follow_gap_m_, gap.lo);
      if (gap.lo + 1e-9 < request.follow_target->hard_gap_m)
        return Reason::FollowGapRejected;
    }
    ++certificate->checked_samples_;
    return Reason::Accepted;
  };
  vehicle::AppliedFootprintValidation footprint_validation;
  const auto vertices = numeric::footprint_vertex_offsets(*wall_footprint);
  for (size_t i = 0; i < vertices.size(); ++i)
    footprint_validation.local_offsets[i] = {vertices[i].lo, vertices[i].hi};
  footprint_validation.validate = [&](const vehicle::BodyRanges &ranges,
      const vehicle::FootprintRanges &corners, double begin, double end) {
    try {
      result.reason = check(ranges, corners, begin, end);
    } catch (const std::exception &) {
      result.reason = Reason::InvalidWorld;
    }
    return result.reason == Reason::Accepted;
  };
  auto prediction = [&]() -> vehicle::AppliedInputPrediction {
    const auto *previous = materialized_from;
    const auto *provenance = execution.applied_stop_program.get();
    if (previous && provenance && execution.terminal_body_rest_required &&
        previous->request_ && previous->request_->plan &&
        previous->request_->plan->physical_snapshot &&
        provenance->nominal_solution_id == previous->prepared_.source.sequence &&
        provenance->nominal_problem_fingerprint ==
            previous->prepared_.source.source_context.fingerprint &&
        provenance->proved_rest_sec == previous->tube_.rest_sec &&
        provenance->forward_velocity_ceiling_mps == expected_ceiling &&
        previous->tube_.context_fingerprint != 0 &&
        previous->tube_.context_fingerprint == certificate->tube_.context_fingerprint &&
        vehicle::applied_input_context_fingerprint(provenance->observation,
            provenance->program, provenance->profile, execution.vehicle_model) ==
            certificate->tube_.context_fingerprint &&
        previous->request_->plan->physical_snapshot->hard_wall_clearance_m ==
            physical_source.hard_wall_clearance_m &&
        previous->tube_.publication_footprint &&
        !previous->tube_.source_to_rest.empty()) {
      // Only plan ownership changes. The new nominal proof has already checked
      // its own cursor, trajectory and first packet. Corner geometry includes
      // both the current footprint and the source's full wall clearance.
      auto original_request = request;
      original_request.plan = previous->request_->plan;
      bool complete = previous->matches(original_request);
      for (const auto &sample : previous->tube_.source_to_rest)
        complete = complete && sample.swept_footprint && sample.endpoint_footprint;
      if (complete) {
        if (!footprint_validation.validate(previous->tube_.publication_body,
                *previous->tube_.publication_footprint, request.now_sec, request.now_sec))
          return {vehicle::AppliedInputRejectReason::ValidationRejected, {}};
        for (const auto &sample : previous->tube_.source_to_rest)
          if (sample.begin_sec >= request.now_sec &&
              !footprint_validation.validate(sample.swept_body,
                  *sample.swept_footprint, sample.begin_sec, sample.end_sec))
            return {vehicle::AppliedInputRejectReason::ValidationRejected, {}};
        certificate->reused_numerical_tube_ = true;
        return {vehicle::AppliedInputRejectReason::None, previous->tube_};
      }
    }
    return vehicle::predict_applied_inputs_to_rest(
        observation, prepared.prepared->program, profile, execution.vehicle_model,
        {}, &footprint_validation);
  }();
  result.prediction_reason = prediction.reason;
  if (!prediction.tube) {
    if (prediction.reason != vehicle::AppliedInputRejectReason::ValidationRejected)
      result.reason = Reason::InputPredictionRejected;
    return result;
  }
  certificate->tube_ = std::move(*prediction.tube);
  if (!certificate->matches(nominal)) {
    result.reason = Reason::InvalidNominalProof;
    return result;
  }
  result.rejected_sec = std::numeric_limits<double>::quiet_NaN();
  result.certificate = std::move(certificate);
  return result;
}

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_applied_program
