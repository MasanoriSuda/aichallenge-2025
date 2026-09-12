#include "multi_purpose_mpc_ros/mpcc_scheduled_observation.hpp"
#include "multi_purpose_mpc_ros/mpcc_scheduled_dispatch.hpp"
#include <cstring>
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include "multi_purpose_mpc_ros/mpcc_wire_command.hpp"

#include "multi_purpose_mpc_ros/detail/mpcc_footprint_enclosure.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_scheduled.hpp"

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
struct WorldCheckStatistics {
  double minimum_peer_clearance_m{std::numeric_limits<double>::infinity()};
  double minimum_follow_gap_m{std::numeric_limits<double>::infinity()};
  std::size_t checked_samples{};
};

struct AppliedWorldCheck {
  const retained::Request &request;
  const vehicle::ObservationProvenance &observation;
  const recovery::FootprintExtents &wall_footprint;
  const std::optional<double> &expected_ceiling;
  double follow_reference_progress_m;
  WorldCheckStatistics &statistics;
  Result &result;
  recovery::Pose2D to_world(const recovery::Pose2D &local) const {
    const auto &origin = observation.initial.state;
    return recovery::Pose2D{origin.x_m + std::cos(origin.yaw_rad) * local.x_m -
                                std::sin(origin.yaw_rad) * local.y_m,
                            origin.y_m + std::sin(origin.yaw_rad) * local.x_m +
                                std::cos(origin.yaw_rad) * local.y_m,
                            origin.yaw_rad + local.yaw_rad};
  }
  Reason check(const vehicle::BodyRanges &ranges,
                         const vehicle::FootprintRanges &corners,
                         const double begin, const double end) {
    const auto &execution = *request.plan->execution_artifact;
    const auto &physical_source = *request.plan->physical_snapshot;
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
    const auto wall = numeric::footprint(state, wall_footprint);
    const auto cells = recovery::sample_footprint(
        *request.current_wall_grid, wall.extents, to_world(wall.pose));
    if (!cells.valid || cells.out_of_map) return Reason::WallRejected;
    for (const auto cell : cells.contact_cells)
      if (!numeric::separating_oriented_cell_clearance(state, wall_footprint, box(corners), *request.current_wall_grid,
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
      statistics.minimum_peer_clearance_m =
          std::min(statistics.minimum_peer_clearance_m, *clearance);
    }
    if (request.follow_target) {
      const auto ego_progress = course_progress(
          state, observation.initial.state, physical_source.course_frame_knots);
      const auto target = retained::follow_target_progress_at(
          *request.follow_target, begin - request.now_sec);
      if (!ego_progress || !target)
        return Reason::FollowProjectionUnavailable;
      const I gap = I(follow_reference_progress_m) +
                    I(*target) - I(ego_progress->hi);
      statistics.minimum_follow_gap_m =
          std::min(statistics.minimum_follow_gap_m, gap.lo);
      if (gap.lo + 1e-9 < request.follow_target->hard_gap_m)
        return Reason::FollowGapRejected;
    }
    ++statistics.checked_samples;
    return Reason::Accepted;
  }
};
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
  input.maximum_publication_delay_sec = execution.publication_interval_sec;
  // Seconds outside the uniquely representable ns domain keep their existing
  // continuous-clock semantics. Never guess an integer origin for them.
  input.nanosecond_clock = mpcc_vehicle_model::publication_nanosecond_clock(
    input.first_packet.published_sec, input.publication_interval_sec,
    input.maximum_publication_delay_sec);
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
  WorldCheckStatistics statistics;
  AppliedWorldCheck world_check{request, observation, *wall_footprint,
    expected_ceiling, nominal.lifted_control_origin_physical_progress_m, statistics, result};
  vehicle::AppliedFootprintValidation footprint_validation;
  const auto vertices = numeric::footprint_vertex_offsets(*wall_footprint);
  for (size_t i = 0; i < vertices.size(); ++i)
    footprint_validation.local_offsets[i] = {vertices[i].lo, vertices[i].hi};
  footprint_validation.validate = [&](const vehicle::BodyRanges &ranges,
      const vehicle::FootprintRanges &corners, double begin, double end) {
    try {
      result.reason = world_check.check(ranges, corners, begin, end);
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
  certificate->minimum_peer_clearance_m_ = statistics.minimum_peer_clearance_m;
  certificate->minimum_follow_gap_m_ = statistics.minimum_follow_gap_m;
  certificate->checked_samples_ = statistics.checked_samples;
  certificate->tube_ = std::move(*prediction.tube);
  if (!certificate->matches(nominal)) {
    result.reason = Reason::InvalidNominalProof;
    return result;
  }
  result.rejected_sec = std::numeric_limits<double>::quiet_NaN();
  result.certificate = std::move(certificate);
  return result;
}

ScheduledResult certify_scheduled_terminal_stop(
    std::shared_ptr<const mpcc_rate_resolved_scheduled::NominalProof> nominal) {
  ScheduledResult result;
  if (!nominal) return result;
  const auto &request = nominal->observed();
  const auto &view = nominal->nominal_view();
  const auto &proof = nominal->proof();
  const auto &forecast = nominal->forecast();
  if (!request.plan || !request.plan->execution_artifact || !request.plan->physical_snapshot ||
      mpcc_rate_resolved_certified_plan::validate(*request.plan) !=
        mpcc_rate_resolved_certified_plan::RejectReason::None ||
      view.plan != request.plan || proof.plan != request.plan ||
      view.decision_id != request.decision_id || proof.decision_id != request.decision_id ||
      proof.observation_origin_sec != forecast.publication_sec ||
      proof.control_origin_sec != forecast.control_origin_sec ||
      view.now_sec != forecast.publication_sec || view.control_origin_sec != forecast.control_origin_sec ||
      proof.obstacle_generation != request.obstacles.generation ||
      proof.observed_sec != request.obstacles.observed_sec ||
      !proof.terminal_stop_certified || proof.terminal_stop_actuation_samples.empty() ||
      !proof.publication_prefix_required || proof.publication_prefix ||
      !proof.applied_program_required || proof.applied_program ||
      !request.publication_prefix || !request.input_application_profile ||
      forecast.observation.now_sec != request.now_sec ||
      forecast.publication_sec < request.now_sec ||
      forecast.nominal_prefix.commands.empty() || forecast.nominal_prefix.repeat_last_until_rest ||
      forecast.nominal_prefix.commands.back().published_sec != forecast.publication_sec)
    return result;
  const auto &execution = *request.plan->execution_artifact;
  const auto &physical_source = *request.plan->physical_snapshot;
  const auto &profile = *request.input_application_profile;
  const auto &first_packet = forecast.nominal_prefix.commands.back();
  if (forecast.vehicle_model_fingerprint != vehicle::fingerprint(execution.vehicle_model) ||
      first_packet.wire_acceleration_mps2 != static_cast<float>(proof.actuation.acceleration_mps2) ||
      first_packet.wire_steering_rad != mpcc_wire_command::steering(
        proof.actuation.steering_rad, execution.vehicle_model.steering_wire_gain) ||
      forecast.nominal_prefix.publication_interval_sec != execution.publication_interval_sec ||
      forecast.nominal_prefix.maximum_publication_delay_sec != execution.publication_interval_sec)
    return result;
  const auto ceiling = proof.terminal_stop_source_horizon_program ?
    retained::source_horizon_velocity_ceiling(view) :
    (execution.applied_stop_program ? execution.applied_stop_program->forward_velocity_ceiling_mps : std::nullopt);
  if ((proof.terminal_stop_source_horizon_program &&
      (!proof.terminal_stop_constant_steering_program || proof.terminal_stop_uses_solved_suffix || !ceiling)) ||
      proof.terminal_stop_forward_velocity_ceiling_mps != ceiling) return result;
  if (ceiling) {
    if (!std::isfinite(*ceiling) || *ceiling <= 0 || view.control_origin_speed_mps > *ceiling) return result;
    for (double velocity : proof.terminal_stop_trajectory.velocity_mps)
      if (!std::isfinite(velocity) || velocity > *ceiling) return result;
  }
  program::Request input{execution.identity, request.decision_id, forecast.control_origin_sec,
    execution.publication_interval_sec, first_packet, execution.vehicle_model.steering_wire_gain,
    request.minimum_acceleration_mps2, request.maximum_acceleration_mps2,
    execution.maximum_abs_steering_rad, execution.maximum_abs_steering_rate_radps,
    execution.physical_global_tolerance, proof.terminal_stop_actuation_samples};
  input.maximum_publication_delay_sec = forecast.nominal_prefix.maximum_publication_delay_sec;
  if (forecast.nominal_prefix.nanosecond_clock) {
    input.nanosecond_clock = vehicle::publication_nanosecond_clock(first_packet.published_sec,
      input.publication_interval_sec, input.maximum_publication_delay_sec);
    if (!input.nanosecond_clock) return result;
  }
  const auto prepared = program::prepare(input);
  result.program_reason = prepared.reason;
  if (!prepared.prepared) { result.reason = Reason::ProgramUnavailable; return result; }
  const auto prefix_count = forecast.nominal_prefix.commands.size() - 1;
  const auto composite = vehicle::prepend_publication_prefix(forecast.nominal_prefix,
    0, prefix_count, prepared.prepared->program);
  if (!composite) { result.reason = Reason::ProgramUnavailable; return result; }
  const auto context = vehicle::scheduled_input_context_fingerprint(
    forecast.observation, *composite, profile, execution.vehicle_model);
  if (!context || context != vehicle::scheduled_input_context_fingerprint(
      request.publication_prefix->observation, *composite, profile, execution.vehicle_model))
    return result;
  if (!request.obstacles.current ||
      !mpcc_rate_resolved_dynamic_proof::observation_valid(request.obstacles) ||
      request.obstacles.observed_sec > request.now_sec ||
      !request.current_wall_grid || !request.current_wall_grid->valid() ||
      !request.current_footprint.valid() ||
      !std::isfinite(nominal->original_follow_reference_progress_m())) {
    result.reason = Reason::InvalidWorld; return result;
  }
  const auto wall_footprint = physical::resolve_clearance_footprint(
    request.current_footprint, physical_source.hard_wall_clearance_m);
  if (!wall_footprint) { result.reason = Reason::InvalidWorld; return result; }
  Result diagnostic;
  WorldCheckStatistics statistics;
  AppliedWorldCheck checker{request, forecast.observation, *wall_footprint, ceiling,
    nominal->original_follow_reference_progress_m(), statistics, diagnostic};
  vehicle::AppliedFootprintValidation footprint;
  const auto vertices = numeric::footprint_vertex_offsets(*wall_footprint);
  for (std::size_t i = 0; i < vertices.size(); ++i)
    footprint.local_offsets[i] = {vertices[i].lo, vertices[i].hi};
  footprint.validate = [&](const vehicle::BodyRanges &ranges,
      const vehicle::FootprintRanges &corners, double begin, double end) {
    try { diagnostic.reason = checker.check(ranges, corners, begin, end); }
    catch (const std::exception &) { diagnostic.reason = Reason::InvalidWorld; }
    return diagnostic.reason == Reason::Accepted;
  };
  auto prediction = vehicle::predict_scheduled_inputs_to_rest(forecast.observation,
    *composite, profile, execution.vehicle_model, {}, &footprint);
  result.prediction_reason = prediction.reason;
  result.rejected_sec = diagnostic.rejected_sec;
  result.rejected_peer_id = diagnostic.rejected_peer_id;
  if (!prediction.tube) {
    result.reason = prediction.reason == vehicle::AppliedInputRejectReason::ValidationRejected ?
      diagnostic.reason : Reason::InputPredictionRejected;
    return result;
  }
  if (prediction.tube->context_fingerprint != context ||
      prediction.tube->rest_sec <= forecast.publication_sec) return result;
  auto certificate = std::shared_ptr<ScheduledCertificate>(new ScheduledCertificate);
  certificate->nominal_ = std::move(nominal);
  certificate->suffix_ = *prepared.prepared;
  certificate->tube_ = std::move(*prediction.tube);
  certificate->first_suffix_index_ = prefix_count;
  certificate->minimum_peer_clearance_m_ = statistics.minimum_peer_clearance_m;
  certificate->minimum_follow_gap_m_ = statistics.minimum_follow_gap_m;
  certificate->checked_samples_ = statistics.checked_samples;
  result.reason = Reason::Accepted;
  result.rejected_sec = std::numeric_limits<double>::quiet_NaN();
  result.rejected_peer_id.clear();
  result.certificate = std::move(certificate);
  return result;
}

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_applied_program

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
MeasurementCheck check_measurement_consistency(
    const applied::ScheduledCertificate &certificate,
    const vehicle::ObservationProvenance &fresh) {
  MeasurementCheck result;
  if (!vehicle::valid(fresh) || !certificate.nominal()) return result;
  const auto &tube = certificate.tube();
  const auto &old = tube.observation;
  if (fresh.now_sec < old.now_sec || fresh.now_sec > tube.rest_sec) {
    result.reason = MeasurementReason::TimeOutsideProof; return result;
  }
  if (fresh.acceleration_delay_sec != old.acceleration_delay_sec ||
      fresh.steering_delay_sec != old.steering_delay_sec) {
    result.reason = MeasurementReason::InputDelayMismatch; return result;
  }
  const auto &model = certificate.nominal()->observed().plan->execution_artifact->vehicle_model;
  if (mpcc_wire_command::steering(fresh.initial.state.desired_steering_rad,
      model.steering_wire_gain) != fresh.commands.back().wire_steering_rad) {
    result.reason = MeasurementReason::IssuedSteeringMismatch; return result;
  }
  const auto ranges_at = [&](const double time) -> const vehicle::BodyRanges * {
    const auto it = std::lower_bound(tube.source_to_rest.begin(), tube.source_to_rest.end(), time,
      [](const vehicle::AppliedInputSample &sample, const double epoch) { return sample.end_sec < epoch; });
    if (it == tube.source_to_rest.end() || time < it->begin_sec) return nullptr;
    return time == it->end_sec ? &it->endpoint_body : &it->swept_body;
  };
  const auto same_or_new = [&](double original, double next) { return next >= original && next <= tube.rest_sec; };
  const auto scalar_matches = [&](double old_source, double fresh_source, double original, double value,
      std::size_t component) {
    if (!same_or_new(old_source, fresh_source)) return false;
    if (old_source == fresh_source) return original == value;
    const auto *ranges = ranges_at(fresh_source);
    return ranges && (*ranges)[component].lower <= value && value <= (*ranges)[component].upper;
  };
  const auto &initial = old.initial.state;
  const auto &measured = fresh.initial.state;
  const double pose_time = fresh.initial.source_sec;
  bool pose_matches = same_or_new(old.initial.source_sec, pose_time);
  if (pose_matches && pose_time == old.initial.source_sec) {
    pose_matches = initial.x_m == measured.x_m && initial.y_m == measured.y_m && initial.yaw_rad == measured.yaw_rad;
  } else if (pose_matches) {
    const auto *ranges = ranges_at(pose_time);
    pose_matches = ranges != nullptr;
    if (ranges) {
      namespace n = vehicle::numerical;
      const auto body = applied::box(*ranges);
      const auto &origin = tube.coordinate_origin;
      const n::I yaw{origin.yaw_rad, origin.yaw_rad};
      const auto x = n::I{origin.x_m, origin.x_m} + n::cos(yaw) * body[0] - n::sin(yaw) * body[1];
      const auto y = n::I{origin.y_m, origin.y_m} + n::sin(yaw) * body[0] + n::cos(yaw) * body[1];
      const auto absolute_yaw = yaw + body[2];
      const auto c = n::cos(absolute_yaw), s = n::sin(absolute_yaw);
      const double measured_c = std::cos(measured.yaw_rad), measured_s = std::sin(measured.yaw_rad);
      pose_matches = x.lo <= measured.x_m && measured.x_m <= x.hi &&
        y.lo <= measured.y_m && measured.y_m <= y.hi &&
        c.lo <= measured_c && measured_c <= c.hi && s.lo <= measured_s && measured_s <= s.hi;
    }
  }
  if (!pose_matches) return {MeasurementReason::PoseMismatch, pose_time};
  if (!scalar_matches(old.velocity_source_sec, fresh.velocity_source_sec,
      initial.forward_velocity_mps, measured.forward_velocity_mps, 3) ||
      !scalar_matches(old.velocity_source_sec, fresh.velocity_source_sec,
      initial.lateral_velocity_mps, measured.lateral_velocity_mps, 4))
    return {MeasurementReason::VelocityMismatch, fresh.velocity_source_sec};
  if (!scalar_matches(old.yaw_rate_source_sec, fresh.yaw_rate_source_sec,
      initial.yaw_rate_radps, measured.yaw_rate_radps, 5))
    return {MeasurementReason::YawRateMismatch, fresh.yaw_rate_source_sec};
  if (!scalar_matches(old.tire_source_sec, fresh.tire_source_sec,
      initial.tire_steering_rad, measured.tire_steering_rad, 7))
    return {MeasurementReason::TireMismatch, fresh.tire_source_sec};
  result.reason = MeasurementReason::Compatible;
  return result;
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
std::optional<vehicle::PublishedProgramSource> scheduled_program_source(
    const applied::ScheduledCertificate &certificate, const std::size_t index) {
  const auto &suffix = certificate.suffix();
  const auto epoch = vehicle::publication_epoch(suffix.program, index);
  if (!certificate.nominal() || !epoch || *epoch > certificate.tube().rest_sec ||
      suffix.program.commands.empty() ||
      (index >= suffix.program.commands.size() && !suffix.program.repeat_last_until_rest))
    return std::nullopt;
  return vehicle::PublishedProgramSource{certificate.nominal()->observed().decision_id,
    suffix.source.sequence, suffix.source.source_context.fingerprint,
    certificate.tube().context_fingerprint, index};
}

PrefixReason check_actual_publication_prefix(
    const applied::ScheduledCertificate &certificate,
    const vehicle::PublishedInputLedger &ledger,
    const vehicle::PublishedInputLedger::Snapshot &original_cursor,
    const vehicle::ObservationProvenance &fresh,
    const std::vector<std::optional<vehicle::PublishedProgramSource>> &prior_sources,
    const std::size_t already_published_suffix_packets) {
  const auto same_history = [](const std::vector<vehicle::PublishedCommand> &a,
      const std::vector<vehicle::PublishedCommand> &b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
      for (auto field : {&vehicle::PublishedCommand::published_sec,
          &vehicle::PublishedCommand::wire_acceleration_mps2, &vehicle::PublishedCommand::wire_steering_rad})
        if (std::memcmp(&(a[i].*field), &(b[i].*field), sizeof(double)) != 0) return false;
    }
    return true;
  };
  if (!same_history(certificate.tube().observation.commands, original_cursor.history()))
    return PrefixReason::OriginalHistoryMismatch;
  if (!vehicle::valid(fresh) || fresh.now_sec < certificate.tube().observation.now_sec ||
      !same_history(fresh.commands, ledger.history()))
    return PrefixReason::CurrentHistoryMismatch;
  if (prior_sources.size() != certificate.first_suffix_index() || prior_sources.size() > 10000 ||
      already_published_suffix_packets > 10000 - prior_sources.size())
    return PrefixReason::InvalidDeclaredPrefix;
  auto sources = prior_sources;
  for (std::size_t i = 0; i < already_published_suffix_packets; ++i) {
    const auto source = scheduled_program_source(certificate, i);
    if (!source) return PrefixReason::InvalidDeclaredPrefix;
    sources.push_back(source);
  }
  if (!ledger.matches_prefix(original_cursor, certificate.tube().program, sources))
    return PrefixReason::ActualPrefixMismatch;
  return PrefixReason::Consistent;
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
namespace {
bool prepare_current_world(
    const applied::ScheduledCertificate &certificate, const retained::Request &fresh,
    CurrentWorldCheck &result, double &follow_reference,
    std::optional<recovery_footprint::FootprintExtents> &footprint) {
  if (!certificate.nominal()) return false;
  const auto &original = certificate.nominal()->observed();
  const auto &tube = certificate.tube();
  if (fresh.plan != original.plan || !fresh.plan || !fresh.plan->physical_snapshot ||
      fresh.current_intent != original.current_intent ||
      !std::isfinite(fresh.now_sec) || fresh.now_sec < original.now_sec || fresh.now_sec > tube.rest_sec ||
      !retained::current_publication_prefix_matches(fresh) || !fresh.applied_program_required ||
      !fresh.input_application_profile ||
      fresh.input_application_profile->profile_id != tube.profile.profile_id ||
      fresh.input_application_profile->acceleration_age_sec != tube.profile.acceleration_age_sec ||
      fresh.input_application_profile->steering_receipt_age_sec != tube.profile.steering_receipt_age_sec ||
      fresh.input_application_profile->steering_mechanical_delay_sec != tube.profile.steering_mechanical_delay_sec ||
      fresh.minimum_acceleration_mps2 != original.minimum_acceleration_mps2 ||
      fresh.maximum_acceleration_mps2 != original.maximum_acceleration_mps2 ||
      fresh.path_length_m != original.path_length_m || fresh.circular != original.circular ||
      fresh.progress_continuity_tolerance_m != original.progress_continuity_tolerance_m ||
      !std::isfinite(fresh.control_origin_physical_progress_m) ||
      !fresh.current_wall_grid || !fresh.current_wall_grid->valid() ||
      !fresh.obstacles.current || !mpcc_rate_resolved_dynamic_proof::observation_valid(fresh.obstacles) ||
      fresh.obstacles.observed_sec > fresh.now_sec || tube.source_to_rest.empty()) return false;
  using Footprint = recovery_footprint::FootprintExtents;
  for (auto field : {&Footprint::front_extent_m, &Footprint::rear_extent_m, &Footprint::left_extent_m,
      &Footprint::right_extent_m, &Footprint::margin_m})
    if (fresh.current_footprint.*field != original.current_footprint.*field) return false;
  using Policy = race_mpcc_foundation::StopPathTrackingPolicy;
  for (auto field : {&Policy::wheelbase_m, &Policy::maximum_abs_steering_rad, &Policy::maximum_abs_steering_rate_radps,
      &Policy::maximum_lateral_acceleration_mps2, &Policy::steering_command_gain, &Policy::lateral_gain, &Policy::heading_gain})
    if (fresh.stop_lateral_policy.*field != original.stop_lateral_policy.*field) return false;
  follow_reference = 0;
  const bool follow_required = fresh.current_intent == retained::contract::ControlIntent::Follow;
  if (fresh.follow_target.has_value() != follow_required || original.follow_target.has_value() != follow_required)
    return false;
  if (follow_required) {
    result.reason = CurrentWorldReason::InvalidFollowObservation;
    const auto &target = *fresh.follow_target;
    const auto &context = fresh.plan->execution_artifact->identity.source_context;
    if (!target.current || !retained::follow_target_progress_at(target, 0) ||
        target.target_id != context.target_id || target.target_id != original.follow_target->target_id ||
        target.observation_generation != fresh.obstacles.generation ||
        target.observed_sec != fresh.obstacles.observed_sec || target.observed_sec > fresh.now_sec ||
        target.hard_gap_m != original.follow_target->hard_gap_m ||
        !std::any_of(fresh.obstacles.obstacles.begin(), fresh.obstacles.obstacles.end(),
          [&](const auto &peer) { return peer.id == target.target_id; })) return false;
    if (target.current_target_gap_m + 1e-9 < target.hard_gap_m) {
      result.reason = CurrentWorldReason::PhysicalRejected;
      result.physical_reason = applied::Reason::FollowGapRejected;
      result.rejected_sec = fresh.now_sec; return false;
    }
    // This current-world API anchors the forecast at physical control-origin
    // progress. An offset-form forecast must not silently move the peer away.
    if (target.elapsed_time_sec.front() != 0.0 ||
        target.target_progress_from_current_origin_m.front() != target.current_target_gap_m) return false;
    // Associate the fresh canonical physical coordinate with the immutable
    // source window. Every potentially nearest branch must agree; a crossing
    // cannot supply a convenient alternate lap/branch for the target forecast.
    vehicle::numerical::Box point{};
    vehicle::State origin{};
    origin.x_m = fresh.control_pose.x_m; origin.y_m = fresh.control_pose.y_m;
    origin.yaw_rad = fresh.control_pose.yaw_rad;
    const auto projected = applied::course_progress(point, origin, fresh.plan->physical_snapshot->course_frame_knots);
    result.reason = CurrentWorldReason::FollowOriginUnavailable;
    if (!projected || !std::isfinite(projected->lo) || !std::isfinite(projected->hi)) return false;
    follow_reference = fresh.control_origin_physical_progress_m;
    if (fresh.circular) {
      if (!std::isfinite(fresh.path_length_m) || fresh.path_length_m <= 0) return false;
      const double laps = std::round(((projected->lo / 2 + projected->hi / 2) - follow_reference) / fresh.path_length_m);
      follow_reference += laps * fresh.path_length_m;
    }
    if (!std::isfinite(follow_reference) ||
        std::abs(follow_reference - projected->lo) > fresh.progress_continuity_tolerance_m ||
        std::abs(follow_reference - projected->hi) > fresh.progress_continuity_tolerance_m) return false;
  }
  footprint = mpcc_rate_resolved_physical_wall::resolve_clearance_footprint(
    fresh.current_footprint, fresh.plan->physical_snapshot->hard_wall_clearance_m);
  if (!footprint) return false;
  return true;
}
} // namespace
CurrentWorldCheck recheck_remaining_world(
    const applied::ScheduledCertificate &certificate, const retained::Request &fresh) {
  CurrentWorldCheck result;
  double follow_reference{};
  std::optional<recovery_footprint::FootprintExtents> footprint;
  if (!prepare_current_world(certificate, fresh, result, follow_reference, footprint)) return result;
  const auto &tube = certificate.tube();
  applied::Result diagnostic;
  applied::WorldCheckStatistics statistics;
  const auto &ceiling = certificate.nominal()->proof().terminal_stop_forward_velocity_ceiling_mps;
  applied::AppliedWorldCheck checker{fresh, tube.observation, *footprint, ceiling, follow_reference, statistics, diagnostic};
  for (const auto &sample : tube.source_to_rest) {
    if (sample.end_sec < fresh.now_sec) continue;
    const double begin = std::max(sample.begin_sec, fresh.now_sec);
    const bool endpoint = begin == sample.end_sec;
    const auto &corners = endpoint ? sample.endpoint_footprint : sample.swept_footprint;
    if (!corners) { result.reason = CurrentWorldReason::InvalidContext; return result; }
    try {
      diagnostic.reason = checker.check(endpoint ? sample.endpoint_body : sample.swept_body,
        *corners, begin, sample.end_sec);
    } catch (const std::exception &) { diagnostic.reason = applied::Reason::InvalidWorld; }
    if (diagnostic.reason != applied::Reason::Accepted) {
      result.reason = CurrentWorldReason::PhysicalRejected;
      result.physical_reason = diagnostic.reason; result.rejected_sec = diagnostic.rejected_sec;
      result.rejected_peer_id = diagnostic.rejected_peer_id; return result;
    }
  }
  if (!statistics.checked_samples) return result;
  result.reason = CurrentWorldReason::Current; result.physical_reason = applied::Reason::Accepted;
  result.minimum_peer_clearance_m = statistics.minimum_peer_clearance_m;
  result.minimum_follow_gap_m = statistics.minimum_follow_gap_m;
  result.checked_samples = statistics.checked_samples;
  return result;
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
namespace {
ContextReason compare_selected_problem_context(
    const retained::contract::MpccProblemContext &source,
    const retained::contract::MpccProblemContext &fresh, const ContextUse use) {
  if (!retained::contract::problem_context_complete(source) ||
      !retained::contract::problem_context_complete(fresh) ||
      (use != ContextUse::NewSource && use != ContextUse::PublishedRemainder)) return ContextReason::InvalidProblem;
  // Fresh observations are authenticated by the component/ledger/world gates.
  // Their generations and decision ID necessarily change. They cannot change
  // the selected problem's model, intent, mission, target, side or formulation.
  if (fresh.intent != source.intent || fresh.intent_generation != source.intent_generation ||
      fresh.target_id != source.target_id || fresh.execution_side_sign != source.execution_side_sign ||
      fresh.dynamic_obstacle_constraint_active != source.dynamic_obstacle_constraint_active ||
      fresh.dynamic_obstacle_id != source.dynamic_obstacle_id ||
      fresh.dynamic_obstacle_side_sign != source.dynamic_obstacle_side_sign ||
      fresh.horizon_steps != source.horizon_steps || fresh.formulation != source.formulation ||
      fresh.vehicle_model_fingerprint != source.vehicle_model_fingerprint ||
      fresh.state_schema_id != source.state_schema_id || fresh.input_schema_id != source.input_schema_id ||
      fresh.bounds_schema_id != source.bounds_schema_id || fresh.cost_schema_id != source.cost_schema_id)
    return ContextReason::SemanticChanged;
  if (use == ContextUse::NewSource && fresh.stage_geometry_id != source.stage_geometry_id)
    return ContextReason::GeometryChanged;
  return ContextReason::Compatible;
}
} // namespace

std::optional<retained::contract::MpccProblemContext> select_new_source_context(
    const retained::contract::MpccProblemContext &source,
    const retained::contract::MpccProblemContext &proposed) {
  if (!retained::contract::problem_context_complete(source) ||
      !retained::contract::problem_context_complete(proposed)) return std::nullopt;
  auto selected = proposed;
  using Intent = retained::contract::ControlIntent;
  // The normal avoidance producer enumerates both sides of this unresolved
  // Cruise/Follow request. Select one candidate; never overwrite a fixed side
  // or infer permission for an opposite-side mission transition.
  if ((proposed.intent == Intent::Cruise || proposed.intent == Intent::Follow) &&
      source.intent == proposed.intent && proposed.execution_side_sign == 0 && source.execution_side_sign == 0 &&
      proposed.dynamic_obstacle_constraint_active && source.dynamic_obstacle_constraint_active &&
      proposed.dynamic_obstacle_side_sign == 0 &&
      (source.dynamic_obstacle_side_sign == -1 || source.dynamic_obstacle_side_sign == 1)) {
    selected.dynamic_obstacle_side_sign = source.dynamic_obstacle_side_sign;
    selected = retained::contract::seal_problem_context(std::move(selected));
  }
  if (compare_selected_problem_context(source, selected, ContextUse::NewSource) != ContextReason::Compatible)
    return std::nullopt;
  return selected;
}

ContextReason check_current_context(
    const applied::ScheduledCertificate &certificate, const ContextSnapshot &fresh_generation,
    const retained::contract::MpccProblemContext &fresh, const ContextUse use) {
  if (!certificate.nominal() || !certificate.nominal()->source_context().valid() || !fresh_generation.valid())
    return ContextReason::MissingGeneration;
  if (!certificate.nominal()->source_context().same_generation(fresh_generation))
    return ContextReason::GenerationChanged;
  const auto &source = certificate.suffix().source.source_context;
  return compare_selected_problem_context(source, fresh, use);
}

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
std::shared_ptr<const StartingDomainEvidence> StartingDomainEvidence::build(
    std::shared_ptr<const applied::ScheduledCertificate> certificate) {
  if (!certificate || !certificate->nominal()) return {};
  const auto &source = certificate->tube();
  const auto &original = certificate->nominal()->observed();
  const bool composite = certificate->first_suffix_index() != 0;
  // The complete-composite theorem retains every original pending packet and
  // its receiver-delay window. Its starting state/time population begins at
  // the NEW suffix, not at the composite's first prior packet. Follow's older
  // first-window theorem does not support that distinct time/input domain.
  if (composite && original.follow_target) return {};
  if (!original.plan || !original.plan->execution_artifact || !original.plan->physical_snapshot) return {};
  const auto footprint = mpcc_rate_resolved_physical_wall::resolve_clearance_footprint(
    original.current_footprint, original.plan->physical_snapshot->hard_wall_clearance_m);
  const auto begin = vehicle::publication_epoch(certificate->suffix().program, 0);
  const auto end = vehicle::publication_epoch(certificate->suffix().program, 0, true);
  if (!footprint || !begin || !end) return {};
  vehicle::StartingDomainRequest request;
  request.source_observation = source.observation;
  request.program = composite ? source.program : certificate->suffix().program;
  request.profile = source.profile;
  request.coordinate_origin = source.coordinate_origin;
  request.starting_sec = {*begin, *end};
  const auto offsets = vehicle::numerical::footprint_vertex_offsets(*footprint);
  for (std::size_t i = 0; i < offsets.size(); ++i)
    request.footprint_offsets[i] = {offsets[i].lo, offsets[i].hi};
  bool initialized = false;
  for (const auto &sample : source.source_to_rest) {
    if (sample.end_sec < source.observation.now_sec || sample.begin_sec > *end) continue;
    if (!initialized) { request.body = sample.swept_body; initialized = true; }
    else for (std::size_t i = 0; i < request.body.size(); ++i) {
      request.body[i].lower = std::min(request.body[i].lower, sample.swept_body[i].lower);
      request.body[i].upper = std::max(request.body[i].upper, sample.swept_body[i].upper);
    }
  }
  if (!initialized) return {};
  // A candidate starting set, never a claimed sensor-error bound. Include the
  // raw component origins as well as their later model predictions. Pose can
  // lead or lag the source motion; declare both directions and prove the
  // entire new set independently below. Body/actuator coordinates keep their
  // original raw-inclusive hull and all physical limits remain unchanged.
  const auto &state = source.observation.initial.state;
  const std::array<double, 8> center{0, 0, 0, state.forward_velocity_mps,
    state.lateral_velocity_mps, state.yaw_rate_radps, state.desired_steering_rad,
    state.tire_steering_rad};
  for (std::size_t i = 0; i < request.body.size(); ++i) {
    auto &range = request.body[i];
    range.lower = std::min(range.lower, center[i]);
    range.upper = std::max(range.upper, center[i]);
    if (i >= 3) continue;
    const auto delta = vehicle::numerical::I(range.lower, range.upper) - vehicle::numerical::I(center[i]);
    const double radius = std::max(std::abs(delta.lo), std::abs(delta.hi));
    const auto symmetric = vehicle::numerical::I(center[i]) + vehicle::numerical::I(-radius, radius);
    range = {symmetric.lo, symmetric.hi};
  }
  auto result = std::shared_ptr<StartingDomainEvidence>(new StartingDomainEvidence);
  result->original_rest_sec_ = source.rest_sec;
  result->includes_pending_prior_ = composite;
  if (original.follow_target) {
    // Keep the existing first-window theorem for Follow. A whole-state/time
    // Cartesian product destroys its progress/time relation and fails the
    // existing fresh-gap regression; late Follow still requires complete
    // current reproof until a time-dependent theorem covers that relation.
    auto prediction = vehicle::predict_starting_domain_to_rest(
      request, original.plan->execution_artifact->vehicle_model);
    if (!prediction.tube) return {};
    result->tube_ = std::move(*prediction.tube);
  } else {
    // Declare all source-derived states across the complete original programme,
    // including repeated braking packets. Old samples propose only this set;
    // its complete Cartesian population/time/corners are freshly proved below.
    for (const auto &sample : source.source_to_rest)
      for (std::size_t i = 0; i < request.body.size(); ++i) {
        request.body[i].lower = std::min(request.body[i].lower, sample.swept_body[i].lower);
        request.body[i].upper = std::max(request.body[i].upper, sample.swept_body[i].upper);
      }
    request.starting_sec.upper = source.rest_sec;
    auto prediction = vehicle::compile_relative_programme_domain_to_rest(
        {request, source.rest_sec},
        original.plan->execution_artifact->vehicle_model);
    if (!prediction.tube) return {};
    result->relative_ = std::move(*prediction.tube);
    result->first_window_only_ = false;
    if (certificate->nominal()->proof().terminal_stop_source_horizon_program) {
      const auto early_end = vehicle::publication_epoch(
        certificate->suffix().program, certificate->first_suffix_index(), true);
      if (early_end) {
        auto early_request = request;
        early_request.starting_sec.upper = std::min(*early_end, source.rest_sec);
        for (std::size_t i = 3; i < early_request.body.size(); ++i)
          early_request.body[i] = {center[i], center[i]};
        for (const auto &sample : source.source_to_rest) {
          if (sample.end_sec < source.observation.now_sec ||
              sample.begin_sec > early_request.starting_sec.upper) continue;
          for (std::size_t i = 3; i < early_request.body.size(); ++i) {
            early_request.body[i].lower = std::min(early_request.body[i].lower, sample.swept_body[i].lower);
            early_request.body[i].upper = std::max(early_request.body[i].upper, sample.swept_body[i].upper);
          }
        }
        auto early_prediction = vehicle::predict_relative_programme_domain_to_rest(
          {early_request, source.rest_sec}, original.plan->execution_artifact->vehicle_model);
        if (early_prediction.tube) {
          auto early = std::shared_ptr<StartingDomainEvidence>(new StartingDomainEvidence);
          early->certificate_ = certificate;
          early->original_rest_sec_ = source.rest_sec;
          early->includes_pending_prior_ = composite;
          early->first_window_only_ = false;
          early->relative_ = std::move(*early_prediction.tube);
          result->early_ = std::move(early);
        }
      }
    }
  }
  result->certificate_ = std::move(certificate);
  return result;
}

namespace {
// Exact remaining suffix for the fresh prefix, shared with complete current
// reproof. The independent programme-domain future still retains ALL original
// packet windows, including authenticated past packets' delayed effects.
std::optional<vehicle::PublishedInputProgram> remaining_programme(
    const applied::ScheduledCertificate &certificate, std::size_t index) {
  auto remaining = certificate.suffix().program;
  const auto epoch = vehicle::publication_epoch(remaining, index);
  if (!epoch || remaining.commands.empty()) return std::nullopt;
  if (index < remaining.commands.size()) {
    remaining.commands.erase(remaining.commands.begin(), remaining.commands.begin() + index);
  } else {
    if (!remaining.repeat_last_until_rest) return std::nullopt;
    remaining.commands = {remaining.commands.back()};
    remaining.commands.front().published_sec = *epoch;
  }
  if (remaining.nanosecond_clock) {
    const auto ns = vehicle::publication_nanoseconds(*epoch);
    if (!ns) return std::nullopt;
    remaining.nanosecond_clock->first_ns = *ns;
  }
  return remaining;
}
std::optional<vehicle::CurrentInputPrefix> check_current_domain(
    const applied::ScheduledCertificate &certificate, const retained::Request &fresh,
    std::size_t index, const std::shared_ptr<const StartingDomainEvidence> &evidence,
    DomainUseReason &use, CurrentWorldCheck &result,
    std::shared_ptr<const StartingDomainEvidence> *selected_evidence) {
  use = DomainUseReason::Missing;
  if (!evidence) return std::nullopt;
  use = DomainUseReason::UnsupportedSuffix;
  if ((certificate.first_suffix_index() != 0 && !evidence->includes_pending_prior()) ||
      (index != 0 && evidence->first_window_only())) return std::nullopt;
  use = DomainUseReason::SourceMismatch;
  if (evidence->certificate().get() != &certificate ||
      evidence->original_rest_sec() != certificate.tube().rest_sec) return std::nullopt;
  if (evidence->early()) {
    DomainUseReason early_use;
    CurrentWorldCheck early_world;
    auto prefix = check_current_domain(certificate, fresh, index, evidence->early(),
      early_use, early_world, selected_evidence);
    if (prefix) {
      use = early_use;
      result = early_world;
      return prefix;
    }
  }
  double follow_reference{};
  std::optional<recovery_footprint::FootprintExtents> footprint;
  use = DomainUseReason::WorldRejected;
  if (!prepare_current_world(certificate, fresh, result, follow_reference, footprint)) return std::nullopt;
  if (fresh.follow_target) {
    use = DomainUseReason::FollowNotMonotone;
    const auto &target = *fresh.follow_target;
    if (target.target_speed_mps < 0 ||
        !std::is_sorted(target.target_progress_from_current_origin_m.begin(),
          target.target_progress_from_current_origin_m.end())) return std::nullopt;
  }
  const auto &domain = evidence->tube();
  // Rebuild original clearance offsets from the freshly authenticated plan.
  // Numeric membership must not borrow unchecked offsets from a supplied prefix.
  vehicle::FootprintRanges offsets;
  const auto vertices = vehicle::numerical::footprint_vertex_offsets(*footprint);
  for (std::size_t i = 0; i < offsets.size(); ++i) offsets[i] = {vertices[i].lo, vertices[i].hi};
  use = DomainUseReason::PrefixUnavailable;
  const auto remaining = remaining_programme(certificate, index);
  if (!remaining) return std::nullopt;
  auto prediction = vehicle::predict_pending_input_prefix(fresh.publication_prefix->observation,
    *remaining, certificate.tube().profile,
    fresh.plan->execution_artifact->vehicle_model, offsets);
  if (!prediction.prefix) return std::nullopt;
  use = DomainUseReason::OutsideDomain;
  std::optional<vehicle::RelativeDomainTransform> relative;
  if (evidence->relative()) {
    relative = vehicle::RelativeDomainTransform::build(*evidence->relative(), *prediction.prefix);
    if (!relative) return std::nullopt;
  } else if (!vehicle::starting_domain_contains_prefix(domain.request, *prediction.prefix)) return std::nullopt;
  const auto &ceiling = certificate.nominal()->proof().terminal_stop_forward_velocity_ceiling_mps;
  applied::Result diagnostic;
  applied::WorldCheckStatistics statistics;
  applied::AppliedWorldCheck checker{fresh, relative ? prediction.prefix->observation : domain.request.source_observation, *footprint,
    ceiling, follow_reference, statistics, diagnostic};
  const auto check = [&](const vehicle::BodyRanges &body, const vehicle::FootprintRanges &corners,
      double begin, double end) {
    try { diagnostic.reason = relative ?
      checker.check(relative->body(body), relative->footprint(corners), begin, end) :
      checker.check(body, corners, begin, end); }
    catch (const std::exception &) { diagnostic.reason = applied::Reason::InvalidWorld; }
    if (diagnostic.reason == applied::Reason::Accepted) return true;
    result.reason = CurrentWorldReason::PhysicalRejected;
    result.physical_reason = diagnostic.reason;
    result.rejected_sec = diagnostic.rejected_sec;
    result.rejected_peer_id = diagnostic.rejected_peer_id;
    return false;
  };
  use = DomainUseReason::WorldRejected;
  if (!check(domain.request.body, domain.initial_footprint,
      std::max(domain.request.starting_sec.lower, fresh.now_sec), domain.request.starting_sec.upper)) return std::nullopt;
  for (const auto &sample : domain.source_to_rest) {
    if (sample.absolute_end_sec < fresh.now_sec) continue;
    if (!check(sample.swept_body, sample.swept_footprint,
        std::max(sample.absolute_begin_sec, fresh.now_sec), sample.absolute_end_sec)) return std::nullopt;
  }
  if (!statistics.checked_samples) return std::nullopt;
  result.reason = CurrentWorldReason::Current;
  result.physical_reason = applied::Reason::Accepted;
  result.minimum_peer_clearance_m = statistics.minimum_peer_clearance_m;
  result.minimum_follow_gap_m = statistics.minimum_follow_gap_m;
  result.checked_samples = statistics.checked_samples;
  use = DomainUseReason::Accepted;
  if (selected_evidence) *selected_evidence = evidence;
  return std::move(prediction.prefix);
}

// New samples may define a new physical population. Older stamps, or changed
// values under an unchanged stamp, cannot supply independent current evidence.
bool component_epochs_are_current(const vehicle::ObservationProvenance &old,
                                  const vehicle::ObservationProvenance &fresh) {
  const auto same_or_new = [](double old_time, double new_time,
                             std::initializer_list<std::pair<double, double>> values) {
    if (new_time < old_time) return false;
    if (new_time > old_time) return true;
    return std::all_of(values.begin(), values.end(), [](const auto &v) { return v.first == v.second; });
  };
  const auto &a = old.initial.state;
  const auto &b = fresh.initial.state;
  return same_or_new(old.initial.source_sec, fresh.initial.source_sec,
      {{a.x_m,b.x_m},{a.y_m,b.y_m},{a.yaw_rad,b.yaw_rad}}) &&
    same_or_new(old.velocity_source_sec, fresh.velocity_source_sec,
      {{a.forward_velocity_mps,b.forward_velocity_mps},{a.lateral_velocity_mps,b.lateral_velocity_mps}}) &&
    same_or_new(old.yaw_rate_source_sec, fresh.yaw_rate_source_sec, {{a.yaw_rate_radps,b.yaw_rate_radps}}) &&
    same_or_new(old.tire_source_sec, fresh.tire_source_sec, {{a.tire_steering_rad,b.tire_steering_rad}});
}

std::optional<vehicle::PendingInputTube> prove_current_remaining_program(
    const applied::ScheduledCertificate &certificate,
    const retained::Request &fresh, std::size_t index,
    CurrentWorldCheck &result,
    const std::shared_ptr<const vehicle::CompiledInputMaps> &maps) {
  double follow_reference{};
  std::optional<recovery_footprint::FootprintExtents> footprint;
  if (!prepare_current_world(certificate, fresh, result, follow_reference, footprint)) return std::nullopt;
  const auto remaining = remaining_programme(certificate, index);
  if (!remaining) return std::nullopt;
  const auto &observation = fresh.publication_prefix->observation;
  const auto &ceiling = certificate.nominal()->proof().terminal_stop_forward_velocity_ceiling_mps;
  applied::Result diagnostic;
  applied::WorldCheckStatistics statistics;
  applied::AppliedWorldCheck checker{fresh, observation, *footprint, ceiling,
    follow_reference, statistics, diagnostic};
  vehicle::AppliedFootprintValidation validation;
  const auto vertices = vehicle::numerical::footprint_vertex_offsets(*footprint);
  for (std::size_t i = 0; i < vertices.size(); ++i)
    validation.local_offsets[i] = {vertices[i].lo, vertices[i].hi};
  validation.validate = [&](const vehicle::BodyRanges &body,
      const vehicle::FootprintRanges &corners, double begin, double end) {
    try { diagnostic.reason = checker.check(body, corners, begin, end); }
    catch (const std::exception &) { diagnostic.reason = applied::Reason::InvalidWorld; }
    return diagnostic.reason == applied::Reason::Accepted;
  };
  auto prediction = vehicle::predict_pending_inputs_to_rest(
      observation, *remaining, certificate.tube().profile,
      fresh.plan->execution_artifact->vehicle_model, {}, &validation, maps);
  if (!prediction.tube || !statistics.checked_samples) {
    result.reason = CurrentWorldReason::PhysicalRejected;
    result.physical_reason = prediction.reason == vehicle::AppliedInputRejectReason::ValidationRejected ?
      diagnostic.reason : applied::Reason::InputPredictionRejected;
    result.rejected_sec = diagnostic.rejected_sec;
    result.rejected_peer_id = diagnostic.rejected_peer_id;
    return std::nullopt;
  }
  result.reason = CurrentWorldReason::Current;
  result.physical_reason = applied::Reason::Accepted;
  result.minimum_peer_clearance_m = statistics.minimum_peer_clearance_m;
  result.minimum_follow_gap_m = statistics.minimum_follow_gap_m;
  result.checked_samples = statistics.checked_samples;
  return std::move(prediction.tube);
}
CurrentCheck check_dispatch_evidence(
    const applied::ScheduledCertificate &certificate, const retained::Request &fresh,
    const retained::contract::MpccProblemContext &fresh_problem, const ContextSnapshot &fresh_generation,
    const vehicle::PublishedInputLedger &ledger, const vehicle::PublishedInputLedger::Snapshot &original_cursor,
    const std::vector<std::optional<vehicle::PublishedProgramSource>> &prior_sources,
    const std::size_t already_published_suffix_packets,
    std::optional<vehicle::PendingInputTube> *independent,
    const std::shared_ptr<const StartingDomainEvidence> &domain = {},
    std::optional<vehicle::CurrentInputPrefix> *domain_prefix = nullptr,
    DomainUseReason *domain_use = nullptr,
    std::shared_ptr<const StartingDomainEvidence> *selected_evidence = nullptr) {
  CurrentCheck result;
  if (!fresh.publication_prefix || fresh_problem.decision_id != fresh.decision_id ||
      fresh_problem.intent != fresh.current_intent ||
      fresh.publication_prefix->observation.now_sec != fresh.now_sec ||
      fresh.publication_prefix->observation.control_origin_sec != fresh.control_origin_sec ||
      (retained::contract::canonical_normal_intent_requires_target_observation(fresh_problem.intent) &&
        fresh_problem.target_obstacle_generation != fresh.obstacles.generation) ||
      (fresh_problem.dynamic_obstacle_constraint_active &&
        fresh_problem.dynamic_obstacle_generation != fresh.obstacles.generation)) return result;
  const auto &observation = fresh.publication_prefix->observation;
  result.measurement = check_measurement_consistency(certificate, observation);
  const auto measurement = result.measurement.reason;
  const bool new_population = measurement != MeasurementReason::Compatible;
  const bool physical_mismatch = measurement == MeasurementReason::PoseMismatch ||
    measurement == MeasurementReason::VelocityMismatch || measurement == MeasurementReason::YawRateMismatch ||
    measurement == MeasurementReason::TireMismatch;
  if (new_population && (!independent || !physical_mismatch ||
      !component_epochs_are_current(certificate.tube().observation, observation))) {
    result.reason = CurrentReason::MeasurementRejected; return result;
  }
  result.prefix = check_actual_publication_prefix(certificate, ledger, original_cursor,
    observation, prior_sources, already_published_suffix_packets);
  if (result.prefix != PrefixReason::Consistent) { result.reason = CurrentReason::PrefixRejected; return result; }
  const auto use = already_published_suffix_packets ? ContextUse::PublishedRemainder : ContextUse::NewSource;
  result.context = check_current_context(certificate, fresh_generation, fresh_problem, use);
  if (result.context != ContextReason::Compatible) { result.reason = CurrentReason::ContextRejected; return result; }
  if (new_population) {
    if (domain_prefix && domain_use)
      *domain_prefix = check_current_domain(certificate, fresh, already_published_suffix_packets,
        domain, *domain_use, result.world, selected_evidence);
    if (!domain_prefix || !*domain_prefix) {
      result.world = CurrentWorldCheck{};
      // Equations are only a numerical aid owned by this independently proved
      // source domain. Current context/history/world/complete rest remain the
      // authority of this fresh prediction, even when the broad domain failed.
      std::shared_ptr<const vehicle::CompiledInputMaps> maps;
      if (domain && domain->certificate().get() == &certificate &&
          domain->relative()) {
        const auto &candidate = domain->relative()->compiled_maps;
        if (candidate && candidate->source_domain_fingerprint() ==
                             domain->tube().context_fingerprint)
          maps = candidate;
      }
      *independent = prove_current_remaining_program(
          certificate, fresh, already_published_suffix_packets, result.world,
          maps);
    }
  } else {
    result.world = recheck_remaining_world(certificate, fresh);
  }
  if (result.world.reason != CurrentWorldReason::Current) { result.reason = CurrentReason::WorldRejected; return result; }
  // Revocation can be observed while a worker is checking physical evidence.
  // The final main-thread dispatcher must check the generation again at send.
  result.context = check_current_context(certificate, fresh_generation, fresh_problem, use);
  result.reason = result.context == ContextReason::Compatible ? CurrentReason::Compatible : CurrentReason::ContextRejected;
  return result;
}
} // namespace
CurrentCheck check_current_evidence(
    const applied::ScheduledCertificate &certificate, const retained::Request &fresh,
    const retained::contract::MpccProblemContext &fresh_problem, const ContextSnapshot &fresh_generation,
    const vehicle::PublishedInputLedger &ledger, const vehicle::PublishedInputLedger::Snapshot &original_cursor,
    const std::vector<std::optional<vehicle::PublishedProgramSource>> &prior_sources,
    const std::size_t already_published_suffix_packets) {
  return check_dispatch_evidence(certificate, fresh, fresh_problem, fresh_generation,
    ledger, original_cursor, prior_sources, already_published_suffix_packets, nullptr);
}

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled


namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
namespace {
bool same_dispatch_wire(double lhs, double rhs) noexcept {
  if (!std::isfinite(lhs) || !std::isfinite(rhs)) return false;
  const float a = static_cast<float>(lhs), b = static_cast<float>(rhs);
  if (static_cast<double>(a) != lhs || static_cast<double>(b) != rhs) return false;
  std::uint32_t a_bits{}, b_bits{};
  std::memcpy(&a_bits, &a, sizeof(a_bits));
  std::memcpy(&b_bits, &b, sizeof(b_bits));
  return a_bits == b_bits;
}
bool same_dispatch_source(const vehicle::PublishedProgramSource &a,
                          const vehicle::PublishedProgramSource &b) noexcept {
  return a.decision_id == b.decision_id && a.solution_id == b.solution_id &&
    a.problem_fingerprint == b.problem_fingerprint &&
    a.input_context_fingerprint == b.input_context_fingerprint && a.packet_index == b.packet_index;
}
} // namespace

DispatchResult prepare_dispatch(
    std::shared_ptr<const applied::ScheduledCertificate> certificate,
    const retained::Request &fresh, const retained::contract::MpccProblemContext &fresh_problem,
    const ContextSnapshot &current_generation, const vehicle::PublishedInputLedger &ledger,
    const vehicle::PublishedInputLedger::Snapshot &original_cursor,
    const std::vector<std::optional<vehicle::PublishedProgramSource>> &prior_sources,
    std::size_t already_published_suffix_packets,
    std::shared_ptr<const StartingDomainEvidence> domain) {
  DispatchResult result;
  if (!certificate || !certificate->nominal()) return result;
  const auto &suffix = certificate->suffix();
  const auto &program = suffix.program;
  const auto index = already_published_suffix_packets;
  const auto epoch = vehicle::publication_epoch(program, index);
  const auto deadline = vehicle::publication_epoch(program, index, true);
  const auto source = scheduled_program_source(*certificate, index);
  result.reason = DispatchReason::InvalidPacket;
  if (!epoch || !deadline || !source || program.commands.empty() ||
      suffix.physical_steering_rad.size() != program.commands.size() ||
      (index >= program.commands.size() && !program.repeat_last_until_rest) ||
      !fresh.publication_prefix) return result;
  const auto explicit_index = std::min(index, program.commands.size() - 1);
  auto packet = program.commands[explicit_index]; packet.published_sec = *epoch;
  const double steering = suffix.physical_steering_rad[explicit_index];
  const auto &execution = *certificate->nominal()->observed().plan->execution_artifact;
  const auto &proposed = fresh.publication_prefix->proposed_packet;
  if (!std::isfinite(steering) ||
      !same_dispatch_wire(mpcc_wire_command::steering(steering,
        execution.vehicle_model.steering_wire_gain), packet.wire_steering_rad) ||
      !same_dispatch_wire(proposed.wire_steering_rad, packet.wire_steering_rad) ||
      !same_dispatch_wire(proposed.wire_acceleration_mps2, packet.wire_acceleration_mps2) ||
      proposed.published_sec != fresh.now_sec || !std::isfinite(fresh.control_origin_speed_mps) ||
      fresh.control_origin_speed_mps < 0) return result;
  // The complete declared send interval must have physical evidence. A
  // repeated terminal packet can start before rest but have a deadline after
  // it; an instantaneous before check cannot authorize that partial interval.
  if (*deadline > certificate->tube().rest_sec || fresh.now_sec > certificate->tube().rest_sec) {
    result.reason = DispatchReason::RestExpired; return result;
  }
  std::optional<vehicle::PendingInputTube> independent;
  std::optional<vehicle::CurrentInputPrefix> domain_prefix;
  auto selected_domain = domain;
  result.current = check_dispatch_evidence(*certificate, fresh, fresh_problem, current_generation,
    ledger, original_cursor, prior_sources, index, &independent, domain, &domain_prefix,
    &result.domain_use, &selected_domain);
  if (result.current.reason != CurrentReason::Compatible) {
    result.reason = DispatchReason::CurrentEvidenceRejected; return result;
  }
  if ((independent && *deadline > independent->numerical.rest_sec) ||
      (domain_prefix && (!selected_domain || *deadline > selected_domain->tube().rest_sec))) {
    result.reason = DispatchReason::RestExpired; return result;
  }
  const auto cursor = ledger.snapshot();
  const auto *previous = ledger.latest_transaction();
  if (!cursor || !previous || previous->sequence != cursor->sequence() ||
      !std::isfinite(previous->published.published_sec) || previous->published.published_sec > fresh.now_sec ||
      !same_dispatch_wire(mpcc_wire_command::steering(fresh.previous_published_steering_rad,
        execution.vehicle_model.steering_wire_gain), previous->nominal.wire_steering_rad)) {
    result.reason = DispatchReason::InvalidPredecessor; return result;
  }
  auto candidate = std::shared_ptr<DispatchCandidate>(new DispatchCandidate);
  if (independent) {
    auto physical = std::shared_ptr<CurrentPhysicalProof>(new CurrentPhysicalProof);
    physical->observed_ = fresh;
    physical->tube_ = std::move(*independent);
    physical->original_input_fingerprint_ = certificate->tube().context_fingerprint;
    physical->first_suffix_index_ = index;
    candidate->current_physical_proof_ = std::move(physical);
  }
  if (domain_prefix) {
    auto physical = std::shared_ptr<CurrentDomainProof>(new CurrentDomainProof);
    physical->observed_ = fresh;
    physical->prefix_ = std::move(*domain_prefix);
    physical->first_suffix_index_ = index;
    physical->evidence_ = std::move(selected_domain);
    // Namespace-separated observation identity, never a caller-supplied token.
    std::uint64_t hash = 14695981039346656037ULL;
    for (const std::uint64_t value : {std::uint64_t{0x435552444f4d3031},
        physical->evidence_->tube().context_fingerprint, physical->prefix_.context_fingerprint,
        certificate->tube().context_fingerprint, fresh.decision_id})
      for (unsigned shift = 0; shift < 64; shift += 8)
        hash = (hash ^ static_cast<unsigned char>(value >> shift)) * 1099511628211ULL;
    physical->physical_input_fingerprint_ = hash;
    candidate->current_domain_proof_ = std::move(physical);
  }
  candidate->certificate_ = std::move(certificate);
  candidate->ledger_cursor_ = *cursor;
  candidate->current_generation_ = current_generation;
  candidate->packet_ = packet;
  candidate->source_ = *source;
  candidate->packet_index_ = index;
  candidate->dispatch_decision_id_ = fresh.decision_id;
  candidate->observed_sec_ = fresh.now_sec;
  candidate->physical_steering_rad_ = steering;
  candidate->predicted_speed_mps_ = fresh.control_origin_speed_mps;
  candidate->previous_wire_steering_rad_ = previous->nominal.wire_steering_rad;
  // The ledger already applies the historic causal floor when /clock was
  // behind a received observation. That stored epoch is also the predecessor
  // used by scheduled nominal selection; do not borrow an earlier raw clock.
  candidate->previous_publication_sec_ = previous->published.published_sec;
  result.candidate = std::move(candidate);
  result.reason = DispatchReason::Candidate;
  return result;
}

bool DispatchCandidate::clock_and_slew_match(double before_clock_sec, double after_clock_sec) const noexcept {
  if (!certificate_ || !certificate_->nominal() || !std::isfinite(before_clock_sec) ||
      !std::isfinite(after_clock_sec) || before_clock_sec < observed_sec_ ||
      before_clock_sec < previous_publication_sec_ || after_clock_sec > certificate_->tube().rest_sec ||
      (current_physical_proof_ && (after_clock_sec > current_physical_proof_->tube().numerical.rest_sec ||
        current_physical_proof_->original_input_fingerprint() != certificate_->tube().context_fingerprint ||
        current_physical_proof_->first_suffix_index() != packet_index_ ||
        current_physical_proof_->observed().decision_id != dispatch_decision_id_)) ||
      (current_domain_proof_ && (current_domain_proof_->first_suffix_index() != packet_index_ ||
        (packet_index_ != 0 && current_domain_proof_->evidence()->first_window_only()) ||
        current_domain_proof_->evidence()->certificate() != certificate_ ||
        current_domain_proof_->observed().decision_id != dispatch_decision_id_ ||
        current_domain_proof_->prefix().observation.now_sec != observed_sec_ ||
        after_clock_sec > current_domain_proof_->evidence()->tube().rest_sec)) ||
      !vehicle::scheduled_publication_bracket_admitted(certificate_->suffix().program, packet_index_,
        certificate_->nominal()->observed().now_sec, before_clock_sec, after_clock_sec)) return false;
  const auto &execution = *certificate_->nominal()->observed().plan->execution_artifact;
  const double step = std::abs(packet_.wire_steering_rad - previous_wire_steering_rad_) /
    execution.vehicle_model.steering_wire_gain;
  const double allowed = execution.maximum_abs_steering_rate_radps *
    (before_clock_sec - previous_publication_sec_) + execution.physical_global_tolerance;
  return std::isfinite(step) && std::isfinite(allowed) && step <= allowed;
}

bool DispatchCandidate::matches_before_publication(const vehicle::PublishedInputLedger &ledger, const ContextSnapshot &current_generation, std::uint64_t dispatch_decision_id, double before_clock_sec, double wire_acceleration_mps2, double wire_steering_rad) const {
  if (!ledger_cursor_ || dispatch_decision_id != dispatch_decision_id_ ||
      !current_generation_.same_generation(current_generation) ||
      !same_dispatch_wire(wire_acceleration_mps2, packet_.wire_acceleration_mps2) ||
      !same_dispatch_wire(wire_steering_rad, packet_.wire_steering_rad) ||
      !clock_and_slew_match(before_clock_sec, before_clock_sec)) return false;
  const auto changes = ledger.since(*ledger_cursor_);
  return changes && changes->empty();
}

bool DispatchCandidate::matches_after_publication(const vehicle::PublishedInputLedger &ledger, const ContextSnapshot &current_generation) const {
  if (!ledger_cursor_ || !current_generation_.same_generation(current_generation)) return false;
  const auto changes = ledger.since(*ledger_cursor_);
  if (!changes || changes->size() != 1) return false;
  const auto &actual = changes->front();
  return actual.source && same_dispatch_source(*actual.source, source_) &&
    actual.nominal.published_sec == packet_.published_sec &&
    same_dispatch_wire(actual.nominal.wire_acceleration_mps2, packet_.wire_acceleration_mps2) &&
    same_dispatch_wire(actual.nominal.wire_steering_rad, packet_.wire_steering_rad) &&
    clock_and_slew_match(actual.before_clock_sec, actual.after_clock_sec);
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
std::optional<double> project_progress(
    const ProgressFrame &frame, const recovery_footprint::Pose2D &pose) {
  for (double value : {frame.pose.x_m, frame.pose.y_m, frame.pose.yaw_rad,
      frame.progress_m, pose.x_m, pose.y_m, pose.yaw_rad})
    if (!std::isfinite(value)) return std::nullopt;
  const double lag = std::cos(frame.pose.yaw_rad) * (pose.x_m - frame.pose.x_m) +
    std::sin(frame.pose.yaw_rad) * (pose.y_m - frame.pose.y_m);
  const double progress = frame.progress_m + lag;
  return std::isfinite(progress) ? std::optional{progress} : std::nullopt;
}

std::optional<double> publication_control_origin(
    const vehicle::PublishedInputProgram &program, std::size_t index,
    double prediction_delay_sec) {
  const auto epoch=vehicle::publication_epoch(program,index);
  const auto epoch_ns=epoch?vehicle::publication_nanoseconds(*epoch):std::nullopt;
  const auto delay_ns=vehicle::publication_nanoseconds(prediction_delay_sec);
  if (!epoch_ns || !delay_ns) return std::nullopt;
  // Both inputs are in the unique ns domain (<=2^51), so their sum cannot
  // overflow int64. Reject a result outside that domain rather than round it.
  const auto origin_ns=*epoch_ns+*delay_ns;
  const double origin=static_cast<double>(origin_ns)/1e9;
  if (vehicle::publication_nanoseconds(origin)!=origin_ns) return std::nullopt;
  return origin;
}

std::optional<retained::Request> bind_current_observation(
    retained::Request request, const vehicle::ObservationProvenance &observation,
    const ProgressFrame &frame, std::optional<vehicle::PublishedCommand> exact_next_packet) {
  if (!request.plan || !request.plan->execution_artifact ||
      !vehicle::valid(observation) ||
      !std::isfinite(frame.pose.x_m) || !std::isfinite(frame.pose.y_m) ||
      !std::isfinite(frame.pose.yaw_rad) || !std::isfinite(frame.progress_m)) return std::nullopt;
  const auto &model = request.plan->execution_artifact->vehicle_model;
  const auto held = vehicle::predict_published_history(observation.initial,
    observation.now_sec, observation.control_origin_sec, observation.commands, model,
    observation.acceleration_delay_sec,
    observation.steering_delay_sec);
  if (!held) return std::nullopt;
  request.now_sec = observation.now_sec;
  request.control_origin_sec = observation.control_origin_sec;
  request.previous_published_steering_rad = observation.initial.state.desired_steering_rad;
  request.previous_published_command_age_sec = observation.now_sec - observation.commands.back().published_sec;
  request.publication_prefix_required = true;
  request.publication_prefix.reset();
  request.follow_target.reset();
  const auto assign_prediction = [&](const auto &prediction) {
    const auto &origin = prediction.control_origin;
    request.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
    request.control_origin_physical_progress_m = project_progress(frame, request.control_pose)
      .value_or(std::numeric_limits<double>::quiet_NaN());
    request.current_speed_mps = prediction.current.forward_velocity_mps;
    request.control_origin_speed_mps = origin.forward_velocity_mps;
    request.current_time_steering_rad = prediction.current.tire_steering_rad;
    request.current_steering_rad = origin.desired_steering_rad;
    request.current_response_steering_rad = origin.tire_steering_rad;
    request.current_lateral_velocity_mps = origin.lateral_velocity_mps;
    request.current_yaw_rate_radps = origin.yaw_rate_radps;
    request.measured_to_control_path.clear();
    request.measured_to_control_elapsed_sec.clear();
    for (const auto &sample : prediction.current_to_control) {
      request.measured_to_control_path.push_back({sample.state.x_m, sample.state.y_m, sample.state.yaw_rad});
      request.measured_to_control_elapsed_sec.push_back(sample.source_sec - observation.now_sec);
    }
  };
  assign_prediction(*held);
  if (!exact_next_packet) exact_next_packet = retained::prospective_artifact_packet(request);
  if (!exact_next_packet || exact_next_packet->published_sec != observation.now_sec) return std::nullopt;
  auto prospective = vehicle::predict_prospective_publication(observation, *exact_next_packet, model);
  if (!prospective) return std::nullopt;
  assign_prediction(*prospective);
  request.publication_prefix = std::move(prospective);
  if (!retained::current_publication_prefix_matches(request)) return std::nullopt;
  return request;
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
v2x_overtake_core::ForwardCourseProjection project_follow_at_observation(
    const std::vector<v2x_overtake_core::CoursePoint> &course,
    v2x_overtake_core::ForwardCourseProjectionRequest current,
    const FollowCourseAnchor &anchor, double observed_sec) {
  if (anchor.target_id.empty() || !std::isfinite(anchor.observed_sec) ||
      !std::isfinite(observed_sec) || observed_sec < anchor.observed_sec ||
      !std::isfinite(anchor.target_path_progress_m) ||
      !std::isfinite(anchor.along_track_speed_mps) || anchor.along_track_speed_mps < 0 ||
      !std::isfinite(current.max_target_path_progress_change_m) ||
      current.max_target_path_progress_change_m < 0) return {};
  current.preferred_target_path_progress_m = anchor.target_path_progress_m +
    anchor.along_track_speed_mps * (observed_sec - anchor.observed_sec);
  try { return v2x_overtake_core::project_forward_course_progress(course, current); }
  catch (const std::exception &) { return {}; }
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
retained::contract::CanonicalNormalCommand DispatchCandidate::canonical_command() const {
  const auto &execution=*certificate_->nominal()->observed().plan->execution_artifact;
  const auto &source=execution.identity.source_context;
  retained::contract::CanonicalNormalCommand command;
  command.decision_id=dispatch_decision_id_; command.execution_plan_id=execution.identity.sequence;
  command.execution_certificate_decision_id=certificate_->suffix().decision_id;
  command.problem_fingerprint=source.fingerprint; command.solution_id=execution.identity.sequence;
  command.source=retained::contract::CanonicalNormalAuthoritySource::RetainedCertified;
  command.intent=source.intent; command.formulation=source.formulation; command.retained_solution=true;
  command.predicted_speed_mps=predicted_speed_mps_; command.acceleration_mps2=packet_.wire_acceleration_mps2;
  command.steering_tire_angle_rad=physical_steering_rad_;
  command.curvature_radpm=std::tan(physical_steering_rad_)/execution.wheelbase_m;
  command.virtual_progress_speed_mps=std::max(0.0,certificate_->nominal()->proof().actuation.virtual_progress_speed_mps);
  return command;
}
std::shared_ptr<const retained::contract::PublishedScheduledIdentity> DispatchCandidate::publication_identity(const vehicle::PublishedInputLedger &ledger, const ContextSnapshot &current_generation) const {
  if (!matches_after_publication(ledger, current_generation)) return nullptr;
  return std::shared_ptr<const retained::contract::PublishedScheduledIdentity>(
    new retained::contract::PublishedScheduledIdentity(canonical_command()));
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled


namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
std::shared_ptr<const SourceReservation> SourceReservation::build(
    std::shared_ptr<const applied::ScheduledCertificate> active, const ContextSnapshot &generation,
    const vehicle::PublishedInputLedger &ledger, std::size_t next_index,
    std::size_t preceding_packet_count, double observed_now_sec, double prediction_delay_sec) {
  if (!active || !active->nominal() || !active->nominal()->source_context().same_generation(generation) ||
      !std::isfinite(observed_now_sec) || !std::isfinite(prediction_delay_sec) || prediction_delay_sec < 0 ||
      next_index == 0 || next_index > 10000 || preceding_packet_count == 0 ||
      preceding_packet_count > 10000 - next_index) return {};
  const auto cursor = ledger.snapshot();
  const auto *last = ledger.latest_transaction();
  const auto predecessor = scheduled_program_source(*active, next_index - 1);
  const auto &program = active->suffix().program;
  const auto next = vehicle::publication_epoch(program, next_index);
  const auto first = vehicle::publication_epoch(program, next_index + preceding_packet_count);
  const auto deadline = vehicle::publication_epoch(program, next_index + preceding_packet_count, true);
  const auto origin = publication_control_origin(program, next_index + preceding_packet_count, prediction_delay_sec);
  const auto remaining = remaining_programme(*active, next_index);
  if (!cursor || !last || !last->source || !predecessor || !next || !first || !deadline || !origin || !remaining ||
      !same_dispatch_source(*last->source, *predecessor) ||
      observed_now_sec < last->published.published_sec || *next < observed_now_sec ||
      last->after_clock_sec > active->tube().rest_sec ||
      !vehicle::scheduled_publication_bracket_admitted(program, next_index - 1,
        active->nominal()->observed().now_sec, last->before_clock_sec, last->after_clock_sec)) return {};
  const auto previous_epoch = vehicle::publication_epoch(program, next_index - 1);
  const auto &previous_packet = program.commands[std::min(next_index - 1, program.commands.size() - 1)];
  if (!previous_epoch || last->nominal.published_sec != *previous_epoch ||
      !same_dispatch_wire(last->nominal.wire_acceleration_mps2, previous_packet.wire_acceleration_mps2) ||
      !same_dispatch_wire(last->nominal.wire_steering_rad, previous_packet.wire_steering_rad)) return {};
  std::vector<std::optional<vehicle::PublishedProgramSource>> sources;
  sources.reserve(preceding_packet_count);
  for (std::size_t i = 0; i < preceding_packet_count; ++i) {
    const auto source = scheduled_program_source(*active, next_index + i);
    const auto end = vehicle::publication_epoch(program, next_index + i, true);
    if (!source || !end || *end > active->tube().rest_sec) return {};
    sources.push_back(source);
  }
  auto result = std::shared_ptr<SourceReservation>(new SourceReservation);
  result->active_ = std::move(active); result->generation_ = generation; result->cursor_ = *cursor;
  result->remaining_prior_ = *remaining; result->prior_sources_ = std::move(sources);
  result->prior_index_ = next_index; result->observed_sec_ = observed_now_sec;
  result->first_publication_sec_ = *first; result->publication_deadline_sec_ = *deadline;
  result->control_origin_sec_ = *origin;
  return result;
}

ReservationStatus SourceReservation::status(const vehicle::PublishedInputLedger &ledger,
    const ContextSnapshot &generation, double now_sec) const {
  if (!cursor_ || !active_ || !generation_.same_generation(generation) ||
      !std::isfinite(now_sec) || now_sec < observed_sec_ || now_sec > publication_deadline_sec_)
    return ReservationStatus::Invalid;
  const auto actual = ledger.since(*cursor_);
  if (!actual || actual->size() > prior_sources_.size()) return ReservationStatus::Invalid;
  const std::vector<std::optional<vehicle::PublishedProgramSource>> sent(
    prior_sources_.begin(), prior_sources_.begin() + actual->size());
  if (!ledger.matches_prefix(*cursor_, remaining_prior_, sent)) return ReservationStatus::Invalid;
  if (actual->size() != prior_sources_.size()) {
    const auto next_deadline = vehicle::publication_epoch(remaining_prior_, actual->size(), true);
    return next_deadline && now_sec <= *next_deadline ? ReservationStatus::Waiting : ReservationStatus::Invalid;
  }
  return now_sec < first_publication_sec_ ? ReservationStatus::Waiting : ReservationStatus::Ready;
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled


namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
retained::contract::ControlIntent programme_publication_intent(
    const applied::ScheduledCertificate &certificate, std::size_t suffix_index) noexcept {
  const auto &proof = certificate.nominal()->proof();
  const auto &program = certificate.suffix().program;
  // The source-horizon builder holds the first word through its solved source
  // horizon, then appends one braking word. Preparation trims only identical
  // trailing words; the retained last word therefore starts its brake tail.
  if (suffix_index == 0 || (proof.terminal_stop_source_horizon_program &&
      program.commands.size() > 1 && suffix_index < program.commands.size() - 1))
    return certificate.suffix().source.source_context.intent;
  return retained::contract::ControlIntent::Stop;
}
} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled
