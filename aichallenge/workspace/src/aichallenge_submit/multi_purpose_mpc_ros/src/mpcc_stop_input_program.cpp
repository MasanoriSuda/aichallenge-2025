#include "multi_purpose_mpc_ros/mpcc_stop_input_program.hpp"
#include "multi_purpose_mpc_ros/mpcc_wire_command.hpp"

#include <algorithm>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_stop_input_program {
namespace {
constexpr double kClockTolerance = 1e-9;
bool finite(double value) { return std::isfinite(value); }
bool serialized(double value) {
  return finite(value) &&
         value == static_cast<double>(static_cast<float>(value));
}
} // namespace

Result prepare(const Request &r) noexcept {
  using R = Reason;
  if (!artifact::identity_valid(r.source) || r.decision_id == 0 ||
      r.decision_id < r.source.source_context.decision_id)
    return {R::InvalidIdentity, {}};
  const double now = r.first_packet.published_sec;
  const double period = r.publication_interval_sec;
  if (!finite(now) || now < 0 || !finite(r.nominal_control_origin_sec) ||
      r.nominal_control_origin_sec < now || !finite(period) || period <= 0 ||
      !finite(r.maximum_publication_delay_sec) || r.maximum_publication_delay_sec < 0 ||
      r.maximum_publication_delay_sec > period)
    return {R::InvalidTiming, {}};
  if (!finite(r.steering_wire_gain) || r.steering_wire_gain <= 0 ||
      !finite(r.minimum_acceleration_mps2) ||
      r.minimum_acceleration_mps2 >= 0 ||
      !finite(r.maximum_acceleration_mps2) || r.maximum_acceleration_mps2 < 0 ||
      !finite(r.maximum_abs_steering_rad) || r.maximum_abs_steering_rad <= 0 ||
      !finite(r.maximum_abs_steering_rate_radps) ||
      r.maximum_abs_steering_rate_radps <= 0 || !finite(r.actuator_tolerance) ||
      r.actuator_tolerance < 0)
    return {R::InvalidLimits, {}};
  if (!serialized(r.first_packet.wire_acceleration_mps2) ||
      !serialized(r.first_packet.wire_steering_rad) ||
      r.nominal_stop_samples.empty() || r.nominal_stop_samples.size() > 10000)
    return {R::InvalidSamples, {}};
  const auto &samples = r.nominal_stop_samples;
  double previous = 0;
  for (const auto &s : samples) {
    if (!finite(s.elapsed_time_sec) || !finite(s.duration_sec) ||
        s.duration_sec <= 0 || s.elapsed_time_sec <= previous ||
        std::abs(s.elapsed_time_sec - previous - s.duration_sec) >
            kClockTolerance ||
        !finite(s.acceleration_mps2) || !finite(s.end_steering_rad) ||
        !finite(s.steering_rate_radps) || !finite(s.end_velocity_mps) ||
        !finite(s.end_lateral_velocity_mps) || !finite(s.end_yaw_rate_radps))
      return {R::InvalidSamples, {}};
    previous = s.elapsed_time_sec;
  }
  if (previous + kClockTolerance < period)
    return {R::InvalidTiming, {}};
  const auto &last = samples.back();
  if (last.end_velocity_mps != 0 || last.end_lateral_velocity_mps != 0 ||
      last.end_yaw_rate_radps != 0 || last.acceleration_mps2 > 0)
    return {R::NominalRestUnavailable, {}};

  Prepared out{r.source,
               r.decision_id,
               r.nominal_control_origin_sec,
               {period, {}, true, r.maximum_publication_delay_sec, r.nanosecond_clock},
               false};
  const auto command_count = static_cast<std::size_t>(
      std::ceil((previous - kClockTolerance) / period));
  if (command_count == 0 || command_count > 10000)
    return {R::InvalidTiming, {}};
  out.program.commands.reserve(command_count);
  out.physical_steering_rad.reserve(command_count);
  std::size_t start_index = 0, end_index = 0;
  for (std::size_t command = 0; command < command_count; ++command) {
    const double begin = command * period,
                 end = std::min((command + 1) * period, previous);
    while (start_index + 1 < samples.size() &&
           samples[start_index].elapsed_time_sec <= begin + kClockTolerance)
      ++start_index;
    while (end_index + 1 < samples.size() &&
           samples[end_index].elapsed_time_sec < end - kClockTolerance)
      ++end_index;
    const auto &first = samples[start_index];
    const auto &final = samples[end_index];
    // A held input is represented by equal dense endpoints. A continuous
    // reference has changing endpoints and is explicitly sampled at the next
    // publication boundary, as the existing command extractor does.
    double steering = final.end_steering_rad;
    bool held = true;
    for (std::size_t j = start_index; j <= end_index; ++j) {
      held = held && samples[j].end_steering_rad == first.end_steering_rad;
      out.resampled_controls =
          out.resampled_controls ||
          samples[j].acceleration_mps2 != first.acceleration_mps2;
    }
    if (!held) {
      steering -= final.steering_rate_radps * (final.elapsed_time_sec - end);
      out.resampled_controls = true;
    }
    vehicle::PublishedCommand packet{
        now + command * period, static_cast<float>(first.acceleration_mps2),
        mpcc_wire_command::steering(steering, r.steering_wire_gain)};
    if (r.nanosecond_clock) {
      const auto epoch = vehicle::publication_epoch(out.program, command);
      if (!epoch) return {R::InvalidTiming, {}};
      packet.published_sec = *epoch;
    }
    if (command == 0) {
      // The caller already selected the actual first publication. Sampling a
      // future angle must not silently change it or its first held interval.
      if (!held ||
          packet.wire_acceleration_mps2 !=
              r.first_packet.wire_acceleration_mps2 ||
          packet.wire_steering_rad != r.first_packet.wire_steering_rad)
        return {R::FirstPacketMismatch, {}};
      for (std::size_t j = start_index; j <= end_index; ++j) {
        if (static_cast<float>(samples[j].acceleration_mps2) !=
            packet.wire_acceleration_mps2)
          return {R::FirstPacketMismatch, {}};
      }
    }
    const double tolerance = r.actuator_tolerance;
    if (packet.wire_acceleration_mps2 <
            r.minimum_acceleration_mps2 - tolerance ||
        packet.wire_acceleration_mps2 > r.maximum_acceleration_mps2 + tolerance)
      return {R::AccelerationOutsideBounds, {}};
    if (std::abs(packet.wire_steering_rad / r.steering_wire_gain) >
        r.maximum_abs_steering_rad + tolerance)
      return {R::SteeringOutsideBounds, {}};
    if (!out.program.commands.empty() &&
        std::abs(packet.wire_steering_rad -
                 out.program.commands.back().wire_steering_rad) /
                r.steering_wire_gain >
            r.maximum_abs_steering_rate_radps * period + tolerance)
      return {R::SteeringStepOutsideBounds, {}};
    out.program.commands.push_back(packet);
    out.physical_steering_rad.push_back(steering);
  }
  // Sampling cannot change a delayed-braking reference into early braking,
  // and a sampled positive final packet cannot become an implicit brake tail.
  if (!vehicle::valid(out.program, now))
    return {R::NominalRestUnavailable, {}};
  while (out.program.commands.size() > 1) {
    const auto &last = out.program.commands.back();
    const auto &prior = out.program.commands[out.program.commands.size() - 2];
    if (last.wire_acceleration_mps2 != prior.wire_acceleration_mps2 ||
        last.wire_steering_rad != prior.wire_steering_rad)
      break;
    out.program.commands.pop_back();
    out.physical_steering_rad.pop_back();
  }
  return {R::Available, std::move(out)};
}

std::optional<vehicle::PublishedInputProgram>
remaining_program(const vehicle::PublishedInputProgram &source,
                  const double now) noexcept {
  if (source.commands.empty() ||
      !vehicle::valid(source, source.commands.front().published_sec) ||
      !source.repeat_last_until_rest || !finite(now) ||
      now < source.commands.front().published_sec)
    return std::nullopt;
  const auto upper = std::upper_bound(
      source.commands.begin(), source.commands.end(), now,
      [](double stamp, const vehicle::PublishedCommand &packet) {
        return stamp < packet.published_sec;
      });
  const std::size_t index =
      static_cast<std::size_t>(upper - source.commands.begin() - 1);
  vehicle::PublishedInputProgram result{
      source.publication_interval_sec, {}, true, source.maximum_publication_delay_sec};
  if (source.nanosecond_clock) {
    result.nanosecond_clock = vehicle::publication_nanosecond_clock(now,
      source.publication_interval_sec, source.maximum_publication_delay_sec);
    if (!result.nanosecond_clock) return std::nullopt;
  }
  result.commands.reserve(source.commands.size() - index);
  for (std::size_t i = index; i < source.commands.size(); ++i) {
    auto packet = source.commands[i];
    packet.published_sec = now + (i - index) * source.publication_interval_sec;
    result.commands.push_back(packet);
    if (result.nanosecond_clock) {
      const auto epoch = vehicle::publication_epoch(result, i - index);
      if (!epoch) return std::nullopt;
      result.commands.back().published_sec = *epoch;
    }
  }
  return vehicle::valid(result, now) ? std::optional{std::move(result)} : std::nullopt;
}

} // namespace multi_purpose_mpc_ros::mpcc_stop_input_program
