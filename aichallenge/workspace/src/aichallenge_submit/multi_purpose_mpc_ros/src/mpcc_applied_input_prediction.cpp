#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {
namespace {
bool nonnegative(const double value) {
  return std::isfinite(value) && value >= 0;
}

bool serialized(const double value) {
  return std::isfinite(value) &&
         value == static_cast<double>(static_cast<float>(value));
}

bool history_valid(const std::vector<PublishedCommand> &history,
                   const double now) {
  if (history.empty())
    return false;
  double previous = -1;
  for (const auto &packet : history) {
    if (!nonnegative(packet.published_sec) || packet.published_sec < previous ||
        packet.published_sec > now ||
        !serialized(packet.wire_acceleration_mps2) ||
        !serialized(packet.wire_steering_rad))
      return false;
    previous = packet.published_sec;
  }
  return true;
}

struct ChannelBounds {
  numerical::I hull;
  std::array<std::optional<numerical::I>, 3> sign_groups;
};

std::optional<ChannelBounds>
channel_bounds(const std::vector<PublishedCommand> &history,
               const PublishedInputProgram &program, const double begin,
               const double end, const double age, const double delay,
               const bool steering) {
  const double threshold = begin - delay - age;
  if (history.front().published_sec > threshold)
    return std::nullopt;
  const double first = numerical::down(numerical::down(begin - delay) - age);
  const double last = numerical::up(end - delay);
  std::optional<numerical::I> range;
  std::array<std::optional<numerical::I>, 3> sign_groups;
  const double tail_start = program.commands.back().published_sec + delay;
  const bool tail_covers_begin = program.repeat_last_until_rest && tail_start <= begin;
  double covered_until = begin;
  bool coverage_started = false;
  const auto add = [&](const PublishedCommand &packet) {
    const numerical::I value(steering ? packet.wire_steering_rad
                                      : packet.wire_acceleration_mps2);
    range = range ? numerical::hull(*range, value) : value;
    if (!steering) {
      const std::size_t sign = value.lo < 0 ? 0 : value.lo > 0 ? 2 : 1;
      auto & group = sign_groups[sign];
      group = group ? numerical::hull(*group, value) : value;
    }
  };
  for (const auto *packets : {&history, &program.commands}) {
    auto packet =
        std::lower_bound(packets->begin(), packets->end(), first,
                         [](const PublishedCommand &value, const double stamp) {
                           return value.published_sec < stamp;
                         });
    for (; packet != packets->end() && packet->published_sec <= last; ++packet) {
      add(*packet);
      const double available_from = packet->published_sec + delay;
      const double available_until = available_from + age;
      if (!tail_covers_begin && (!coverage_started || covered_until < end) &&
        available_until >= begin)
      {
        // A value which becomes admissible later in this substep cannot fill
        // an earlier interval where the declared age has no possible packet.
        if (available_from > covered_until) return std::nullopt;
        coverage_started = true;
        covered_until = std::max(covered_until, available_until);
      }
    }
  }
  // A constant tail republishes at the same period. Each declared channel age
  // covers at least one period, so beyond its first repetition there is always
  // a tail packet in the admissible window. Never introduce it before then.
  if (program.repeat_last_until_rest &&
      program.commands.back().published_sec +
              program.publication_interval_sec <=
          last) {
    add(program.commands.back());
  }
  if (!tail_covers_begin &&
    (!coverage_started || (covered_until < end &&
    !(program.repeat_last_until_rest && tail_start <= covered_until)))) return std::nullopt;
  if (!range) return std::nullopt;
  return ChannelBounds{*range, sign_groups};
}

std::optional<AppliedInputBounds>
bounds(const std::vector<PublishedCommand> &history,
       const PublishedInputProgram &program,
       const InputApplicationProfile &profile, const double begin,
       const double end) {
  const auto acceleration = channel_bounds(
      history, program, begin, end, profile.acceleration_age_sec, 0, false);
  const auto steering = channel_bounds(
      history, program, begin, end, profile.steering_receipt_age_sec,
      profile.steering_mechanical_delay_sec, true);
  if (!acceleration || !steering)
    return std::nullopt;
  AppliedInputBounds result{{acceleration->hull.lo, acceleration->hull.hi},
                            {steering->hull.lo, steering->hull.hi}, {}};
  for (std::size_t i = 0; i < result.acceleration_sign_groups.size(); ++i) {
    if (const auto & group = acceleration->sign_groups[i])
      result.acceleration_sign_groups[i] = ScalarRange{group->lo, group->hi};
  }
  return result;
}

bool compatible(const InputApplicationProfile &profile,
                const PublishedInputProgram &program) {
  return profile.acceleration_age_sec >= program.publication_interval_sec &&
         profile.steering_receipt_age_sec >= program.publication_interval_sec;
}

BodyRanges body_ranges(const numerical::Box &box) {
  BodyRanges ranges;
  for (std::size_t i = 0; i < box.size(); ++i)
    ranges[i] = {box[i].lo, box[i].hi};
  return ranges;
}

struct Hash {
  std::uint64_t value{14695981039346656037ULL};
  void integer(const std::uint64_t number) {
    for (unsigned shift = 0; shift < 64; shift += 8) {
      value = (value ^ static_cast<unsigned char>(number >> shift)) *
              1099511628211ULL;
    }
  }
  void number(const double number) {
    std::uint64_t bits{};
    static_assert(sizeof(bits) == sizeof(number));
    std::memcpy(&bits, &number, sizeof(bits));
    integer(bits);
  }
  void string(const std::string_view text) {
    integer(text.size());
    for (const unsigned char byte : text)
      value = (value ^ byte) * 1099511628211ULL;
  }
  void commands(const std::vector<PublishedCommand> &packets) {
    integer(packets.size());
    for (const auto &packet : packets) {
      number(packet.published_sec);
      number(packet.wire_acceleration_mps2);
      number(packet.wire_steering_rad);
    }
  }
};
} // namespace

bool valid(const InputApplicationProfile &profile) noexcept {
  return !profile.profile_id.empty() &&
         nonnegative(profile.acceleration_age_sec) &&
         nonnegative(profile.steering_receipt_age_sec) &&
         nonnegative(profile.steering_mechanical_delay_sec);
}

bool valid(const PublishedInputProgram &program,
           const double publication) noexcept {
  if (!nonnegative(publication) ||
      !std::isfinite(program.publication_interval_sec) ||
      program.publication_interval_sec <= 0 || program.commands.empty() ||
      program.commands.size() > 10000 ||
      program.commands.front().published_sec != publication) {
    return false;
  }
  for (std::size_t i = 0; i < program.commands.size(); ++i) {
    const auto &packet = program.commands[i];
    if (!nonnegative(packet.published_sec) ||
        packet.published_sec !=
            publication + i * program.publication_interval_sec ||
        !serialized(packet.wire_acceleration_mps2) ||
        !serialized(packet.wire_steering_rad))
      return false;
  }
  return !program.repeat_last_until_rest ||
         program.commands.back().wire_acceleration_mps2 <= 0;
}

std::uint64_t
applied_input_context_fingerprint(const ObservationProvenance &observation,
                                  const PublishedInputProgram &program,
                                  const InputApplicationProfile &profile,
                                  const Parameters &parameters) noexcept {
  if (!valid(parameters) || !valid(observation) || !valid(profile) ||
      !valid(program, observation.now_sec) || !compatible(profile, program) ||
      !history_valid(observation.commands, observation.now_sec))
    return 0;
  Hash hash;
  hash.string(kAppliedInputSchema);
  hash.integer(fingerprint(parameters));
  hash.string(profile.profile_id);
  hash.number(profile.acceleration_age_sec);
  hash.number(profile.steering_receipt_age_sec);
  hash.number(profile.steering_mechanical_delay_sec);
  for (const double value : kernel::values(observation.initial.state))
    hash.number(value);
  for (const double value :
       {observation.initial.source_sec, observation.velocity_source_sec,
        observation.yaw_rate_source_sec, observation.tire_source_sec,
        observation.now_sec, observation.control_origin_sec,
        observation.acceleration_delay_sec, observation.steering_delay_sec}) {
    hash.number(value);
  }
  hash.commands(observation.commands);
  hash.number(program.publication_interval_sec);
  hash.commands(program.commands);
  hash.integer(program.repeat_last_until_rest);
  return hash.value == 0 ? 1 : hash.value;
}

std::uint64_t applied_program_provenance_fingerprint(
    const AppliedProgramProvenance & provenance, const Parameters & parameters) noexcept {
  const auto input = applied_input_context_fingerprint(provenance.observation,
    provenance.program, provenance.profile, parameters);
  if (input == 0 || provenance.nominal_solution_id == 0 || provenance.nominal_problem_fingerprint == 0 ||
    !std::isfinite(provenance.proved_rest_sec) ||
    provenance.proved_rest_sec <= provenance.observation.now_sec) return 0;
  Hash hash;
  hash.string("applied-stop-provenance-v1");
  hash.integer(input); hash.integer(provenance.nominal_solution_id);
  hash.integer(provenance.nominal_problem_fingerprint); hash.number(provenance.proved_rest_sec);
  if (provenance.forward_velocity_ceiling_mps) {
    const double ceiling = *provenance.forward_velocity_ceiling_mps;
    if (!std::isfinite(ceiling) || ceiling <= 0) return 0;
    hash.string("source-forward-velocity-ceiling-v1");
    hash.number(ceiling);
  }
  return hash.value == 0 ? 1 : hash.value;
}

std::optional<AppliedInputBounds>
applied_input_bounds(const std::vector<PublishedCommand> &history,
                     const PublishedInputProgram &program,
                     const InputApplicationProfile &profile, const double begin,
                     const double end) noexcept {
  if (!nonnegative(begin) || !nonnegative(end) || end < begin ||
      !valid(profile) || program.commands.empty() ||
      !valid(program, program.commands.front().published_sec) ||
      !compatible(profile, program) ||
      !history_valid(history, program.commands.front().published_sec)) {
    return std::nullopt;
  }
  return bounds(history, program, profile, begin, end);
}

AppliedInputPrediction
predict_applied_inputs_to_rest(const ObservationProvenance &observation,
                               const PublishedInputProgram &program,
                               const InputApplicationProfile &profile,
                               const Parameters &parameters) noexcept {
  return predict_applied_inputs_to_rest(observation, program, profile, parameters, {});
}

AppliedInputPrediction
predict_applied_inputs_to_rest(const ObservationProvenance &observation,
                               const PublishedInputProgram &program,
                               const InputApplicationProfile &profile,
                               const Parameters &parameters,
                               const AppliedInputValidator &validator) noexcept {
  return predict_applied_inputs_to_rest(observation, program, profile, parameters, validator, nullptr);
}

AppliedInputPrediction
predict_applied_inputs_to_rest(const ObservationProvenance &observation,
                               const PublishedInputProgram &program,
                               const InputApplicationProfile &profile,
                               const Parameters &parameters,
                               const AppliedInputValidator &validator,
                               const AppliedFootprintValidation *footprint) noexcept {
  using Reason = AppliedInputRejectReason;
  if (!valid(parameters) || !parameters.nominal_settled_contact)
    return {Reason::InvalidModel, {}};
  if (!valid(observation) ||
      !history_valid(observation.commands, observation.now_sec)) {
    return {Reason::InvalidObservation, {}};
  }
  if (!valid(profile))
    return {Reason::InvalidProfile, {}};
  if (!valid(program, observation.now_sec) || !program.repeat_last_until_rest) {
    return {Reason::InvalidProgram, {}};
  }
  if (!compatible(profile, program))
    return {Reason::InvalidProfile, {}};
  try {
    AppliedInputTube tube;
    tube.context_fingerprint = applied_input_context_fingerprint(
        observation, program, profile, parameters);
    if (tube.context_fingerprint == 0)
      return {Reason::InvalidObservation, {}};
    tube.observation = observation;
    tube.program = program;
    tube.profile = profile;
    tube.coordinate_origin = observation.initial.state;
    auto initial = observation.initial.state;
    initial.x_m = 0;
    initial.y_m = 0;
    initial.yaw_rad = 0;
    std::vector<numerical::Box> population{numerical::point(initial)};
    std::optional<numerical::CornerPopulation> corners;
    if (footprint) {
      if (!footprint->validate) return {Reason::NumericalFailure, {}};
      numerical::Box offsets, seed;
      for (size_t i = 0; i < offsets.size(); ++i) {
        const auto &value = footprint->local_offsets[i];
        if (!std::isfinite(value.lower) || !std::isfinite(value.upper) || value.lower > value.upper)
          return {Reason::NumericalFailure, {}};
        offsets[i] = {value.lower, value.upper};
      }
      const double yaw = observation.initial.state.yaw_rad;
      const auto co = numerical::cosine(numerical::I(yaw));
      const auto so = numerical::sine(numerical::I(yaw));
      for (size_t i = 0; i < offsets.size(); i += 2) {
        seed[i] = co * offsets[i] - so * offsets[i + 1];
        seed[i + 1] = so * offsets[i] + co * offsets[i + 1];
      }
      corners = numerical::CornerPopulation{{offsets, yaw}, {seed}, seed};
    }
    tube.maximum_body_partitions = 1;
    const double prefix_duration = observation.now_sec - observation.initial.source_sec;
    const auto prefix_steps = integration_steps(prefix_duration, parameters.maximum_step_sec);
    if (prefix_duration > 0 && prefix_steps == 0) return {Reason::StepLimit, {}};
    const double prefix_dt = prefix_steps == 0 ? 0 : prefix_duration / prefix_steps;
    double stamp = observation.initial.source_sec;
    if (stamp == observation.now_sec) {
      tube.publication_body = body_ranges(population.front());
      if (corners) tube.publication_footprint = body_ranges(corners->states.front());
    }
    // Even an already stationary member may still receive a positive packet.
    // Every explicitly scheduled packet owns one complete publisher interval,
    // even if the body is already stationary before that interval ends.
    double rest_not_before = observation.now_sec +
      program.commands.size() * program.publication_interval_sec;
    for (const auto *packets : {&observation.commands, &program.commands}) {
      for (const auto &packet : *packets) {
        if (packet.wire_acceleration_mps2 > 0) {
          rest_not_before =
              std::max(rest_not_before,
                       packet.published_sec + profile.acceleration_age_sec);
        }
      }
    }
    tube.source_to_rest.reserve(512);
    for (std::size_t step = 0; step < 10000; ++step) {
      // Bridge to the exact publication epoch with a counted native interval,
      // then use integer-indexed publisher-relative steps. Repeated additions
      // can leave a sub-ULP bridge remainder whose absolute endpoints are equal.
      // Integrating that invented remainder would alter the tire update count.
      const bool in_prefix = step < prefix_steps;
      const double duration = in_prefix ? prefix_dt : parameters.maximum_step_sec;
      const double end = in_prefix ?
        (step + 1 == prefix_steps ? observation.now_sec :
        observation.initial.source_sec + (step + 1) * prefix_dt) :
        observation.now_sec + (step - prefix_steps + 1) * parameters.maximum_step_sec;
      if (!std::isfinite(end) || end <= stamp)
        return {Reason::StepLimit, {}};
      const auto inputs =
          bounds(observation.commands, program, profile, stamp, end);
      if (!inputs)
        return {Reason::HistoryUnavailable, {}};
      std::vector<numerical::I> accelerations;
      accelerations.reserve(inputs->acceleration_sign_groups.size());
      for (const auto & group : inputs->acceleration_sign_groups)
        if (group) accelerations.emplace_back(group->lower, group->upper);
      const numerical::I desired =
          numerical::I(inputs->wire_steering_rad.lower,
                       inputs->wire_steering_rad.upper) /
          parameters.steering_wire_gain;
      for (auto &state : population)
        state[kernel::Desired] = desired;
      if (stamp == observation.now_sec) {
        tube.publication_body = body_ranges(numerical::joined(population));
        if (validator && !validator(tube.publication_body, stamp, stamp))
          return {Reason::ValidationRejected, {}};
        if (corners) {
          tube.publication_footprint = body_ranges(numerical::joined(corners->states));
          if (!footprint->validate(tube.publication_body, *tube.publication_footprint, stamp, stamp))
            return {Reason::ValidationRejected, {}};
        }
      }
      numerical::Box swept;
      population = numerical::advance_partitioned_inputs(
          std::move(population), accelerations, parameters, duration, &swept,
          corners ? &*corners : nullptr);
      const auto endpoint = numerical::joined(population);
      tube.maximum_body_partitions =
          std::max(tube.maximum_body_partitions, population.size());
      tube.source_to_rest.push_back({stamp, end, duration, *inputs,
                                     body_ranges(swept),
                                     body_ranges(endpoint)});
      if (corners) {
        tube.source_to_rest.back().swept_footprint = body_ranges(corners->swept);
        tube.source_to_rest.back().endpoint_footprint = body_ranges(numerical::joined(corners->states));
      }
      if (stamp >= observation.now_sec && validator &&
        !validator(tube.source_to_rest.back().swept_body, stamp, end))
        return {Reason::ValidationRejected, {}};
      if (stamp >= observation.now_sec && footprint &&
        !footprint->validate(tube.source_to_rest.back().swept_body,
          *tube.source_to_rest.back().swept_footprint, stamp, end))
        return {Reason::ValidationRejected, {}};
      stamp = end;
      if (stamp == observation.now_sec) {
        tube.publication_body = body_ranges(endpoint);
        if (corners) tube.publication_footprint = tube.source_to_rest.back().endpoint_footprint;
      }
      if (stamp > observation.now_sec && stamp > rest_not_before &&
          numerical::at_rest(endpoint)) {
        tube.rest_sec = stamp;
        return {Reason::None, std::move(tube)};
      }
    }
    return {Reason::StepLimit, {}};
  } catch (const std::exception &) {
    return {Reason::NumericalFailure, {}};
  }
}

} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
