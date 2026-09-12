#include "multi_purpose_mpc_ros/mpcc_starting_domain.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string_view>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {
namespace {
namespace n = numerical;
using Reason = AppliedInputRejectReason;
bool valid_ranges(const BodyRanges &ranges) {
  return std::all_of(ranges.begin(), ranges.end(), [](const auto &v) {
    return std::isfinite(v.lower) && std::isfinite(v.upper) &&
           v.lower <= v.upper;
  });
}
n::Box box(const BodyRanges &ranges) {
  n::Box out;
  for (std::size_t i = 0; i < 8; ++i)
    out[i] = {ranges[i].lower, ranges[i].upper};
  return out;
}
BodyRanges ranges(const n::Box &box) {
  BodyRanges out;
  for (std::size_t i = 0; i < 8; ++i)
    out[i] = {box[i].lo, box[i].hi};
  return out;
}
struct Hash {
  std::uint64_t value{14695981039346656037ULL};
  void integer(std::uint64_t v) {
    for (unsigned s = 0; s < 64; s += 8)
      value = (value ^ static_cast<unsigned char>(v >> s)) * 1099511628211ULL;
  }
  void number(double v) {
    std::uint64_t bits{};
    std::memcpy(&bits, &v, sizeof(v));
    integer(bits);
  }
  void string(std::string_view v) {
    integer(v.size());
    for (unsigned char c : v)
      value = (value ^ c) * 1099511628211ULL;
  }
  void bounds(const BodyRanges &v) {
    for (const auto &r : v) {
      number(r.lower);
      number(r.upper);
    }
  }
  void state(const State &v) {
    for (double x : {v.x_m, v.y_m, v.yaw_rad, v.forward_velocity_mps,
                     v.lateral_velocity_mps, v.yaw_rate_radps,
                     v.desired_steering_rad, v.tire_steering_rad})
      number(x);
  }
};
std::vector<n::I> acceleration_groups(const AppliedInputBounds &inputs) {
  std::vector<n::I> result;
  for (const auto &g : inputs.acceleration_sign_groups)
    if (g)
      result.emplace_back(g->lower, g->upper);
  return result;
}
void assign_steering(std::vector<n::Box> &population,
                     const AppliedInputBounds &inputs,
                     const Parameters &parameters) {
  for (auto &state : population)
    state[6] =
        n::I(inputs.wire_steering_rad.lower, inputs.wire_steering_rad.upper) /
        parameters.steering_wire_gain;
}
n::CornerPopulation point_corners(const State &origin, const n::Box &offsets) {
  const auto co = n::cosine(n::I(origin.yaw_rad)),
             so = n::sine(n::I(origin.yaw_rad));
  n::Box seed;
  for (std::size_t i = 0; i < 8; i += 2) {
    seed[i] = co * offsets[i] - so * offsets[i + 1];
    seed[i + 1] = so * offsets[i] + co * offsets[i + 1];
  }
  return {{offsets, origin.yaw_rad}, {seed}, seed};
}
n::CornerPopulation domain_corners(const State &origin, const n::Box &initial,
                                   const n::Box &offsets) {
  const auto co = n::cosine(n::I(origin.yaw_rad)),
             so = n::sine(n::I(origin.yaw_rad));
  const auto ca = n::cosine(initial[2] + n::I(origin.yaw_rad)),
             sa = n::sine(initial[2] + n::I(origin.yaw_rad));
  n::Box seed;
  for (std::size_t i = 0; i < 8; i += 2) {
    seed[i] = co * initial[0] - so * initial[1] + ca * offsets[i] -
              sa * offsets[i + 1];
    seed[i + 1] = so * initial[0] + co * initial[1] + sa * offsets[i] +
                  ca * offsets[i + 1];
  }
  return {{offsets, origin.yaw_rad}, {seed}, seed};
}
} // namespace

CurrentInputPrefixPrediction predict_pending_input_prefix(
    const ObservationProvenance &observation,
    const PublishedInputProgram &program,
    const InputApplicationProfile &profile, const Parameters &parameters,
    const FootprintRanges &footprint_offsets) noexcept {
  if (!valid(parameters) || !parameters.nominal_settled_contact)
    return {Reason::InvalidModel, {}};
  if (!valid_ranges(footprint_offsets))
    return {Reason::InvalidObservation, {}};
  const auto context = pending_input_context_fingerprint(observation, program,
                                                         profile, parameters);
  if (!context || !program.repeat_last_until_rest)
    return {Reason::InvalidObservation, {}};
  try {
    auto initial = observation.initial.state;
    initial.x_m = initial.y_m = initial.yaw_rad = 0;
    std::vector<n::Box> population{n::point(initial)};
    auto corners =
        point_corners(observation.initial.state, box(footprint_offsets));
    const double duration =
        observation.now_sec - observation.initial.source_sec;
    const auto count = integration_steps(duration, parameters.maximum_step_sec);
    if (duration > 0 && !count)
      return {Reason::StepLimit, {}};
    const double dt = count ? duration / count : 0;
    double stamp = observation.initial.source_sec;
    for (std::size_t step = 0; step < count; ++step) {
      const double end = step + 1 == count
                             ? observation.now_sec
                             : observation.initial.source_sec + (step + 1) * dt;
      if (!std::isfinite(end) || end <= stamp)
        return {Reason::StepLimit, {}};
      const auto inputs = applied_input_bounds(observation.commands, program,
                                               profile, stamp, end);
      if (!inputs)
        return {Reason::HistoryUnavailable, {}};
      assign_steering(population, *inputs, parameters);
      n::Box swept;
      population = n::advance_partitioned_inputs(
          std::move(population), acceleration_groups(*inputs), parameters, dt,
          &swept, &corners);
      stamp = end;
    }
    const auto inputs = applied_input_bounds(
        observation.commands, program, profile, observation.now_sec,
        observation.now_sec + parameters.maximum_step_sec);
    if (!inputs)
      return {Reason::HistoryUnavailable, {}};
    assign_steering(population, *inputs, parameters);
    Hash hash;
    hash.string("pending-current-short-prefix-v1");
    hash.integer(context);
    hash.bounds(footprint_offsets);
    CurrentInputPrefix prefix{hash.value ? hash.value : 1,
                              observation,
                              program,
                              profile,
                              observation.initial.state,
                              footprint_offsets,
                              ranges(n::joined(population)),
                              ranges(n::joined(corners.states))};
    return {Reason::None, std::move(prefix)};
  } catch (const std::exception &) {
    return {Reason::NumericalFailure, {}};
  }
}

std::uint64_t
starting_domain_context_fingerprint(const StartingDomainRequest &request,
                                    const Parameters &parameters) noexcept {
  const auto base = scheduled_input_context_fingerprint(
      request.source_observation, request.program, request.profile, parameters);
  const auto first = publication_epoch(request.program, 0),
             last = publication_epoch(request.program, 0, true);
  const auto &t = request.starting_sec;
  if (!base || !first || !last || !parameters.nominal_settled_contact ||
      !request.program.repeat_last_until_rest ||
      !finite(request.coordinate_origin) || !valid_ranges(request.body) ||
      !valid_ranges(request.footprint_offsets) || !std::isfinite(t.lower) ||
      !std::isfinite(t.upper) || t.lower > t.upper || t.lower < *first ||
      t.upper > *last)
    return 0;
  Hash hash;
  hash.string("independent-starting-state-time-domain-v1");
  hash.integer(base);
  hash.state(request.coordinate_origin);
  hash.bounds(request.body);
  hash.bounds(request.footprint_offsets);
  hash.number(t.lower);
  hash.number(t.upper);
  return hash.value ? hash.value : 1;
}

namespace {
StartingDomainPrediction predict_domain_to_rest(
    const StartingDomainRequest &request, const Parameters &parameters,
    const std::uint64_t context) noexcept {
  if (!valid(parameters) || !parameters.nominal_settled_contact)
    return {Reason::InvalidModel, {}};
  if (!context)
    return {Reason::InvalidObservation, {}};
  try {
    StartingDomainTube tube;
    tube.context_fingerprint = context;
    tube.request = request;
    std::vector<n::Box> population{box(request.body)};
    auto corners = domain_corners(request.coordinate_origin, population.front(),
                                  box(request.footprint_offsets));
    tube.initial_footprint = ranges(corners.states.front());
    tube.maximum_body_partitions = 1;
    double rest_not_before = n::up(*publication_epoch(
        request.program, request.program.commands.size(), true));
    for (const auto &p : request.source_observation.commands)
      if (p.wire_acceleration_mps2 > 0)
        rest_not_before = std::max(
            rest_not_before,
            (n::I(p.published_sec) + n::I(request.profile.acceleration_age_sec))
                .hi);
    for (std::size_t i = 0; i < request.program.commands.size(); ++i)
      if (request.program.commands[i].wire_acceleration_mps2 > 0)
        rest_not_before =
            std::max(rest_not_before,
                     (n::I(*publication_epoch(request.program, i, true)) +
                      n::I(request.profile.acceleration_age_sec))
                         .hi);
    tube.source_to_rest.reserve(512);
    for (std::size_t step = 0; step < 10000; ++step) {
      const double dt = parameters.maximum_step_sec;
      const auto begin =
          n::I(request.starting_sec.lower) + n::I(double(step)) * n::I(dt);
      const auto end =
          n::I(request.starting_sec.upper) + n::I(double(step + 1)) * n::I(dt);
      const auto earliest_end =
          n::I(request.starting_sec.lower) + n::I(double(step + 1)) * n::I(dt);
      if (!std::isfinite(begin.lo) || !std::isfinite(end.hi) ||
          end.hi <= begin.lo)
        return {Reason::StepLimit, {}};
      const auto inputs = applied_input_bounds(
          request.source_observation.commands, request.program, request.profile,
          begin.lo, end.hi);
      if (!inputs)
        return {Reason::HistoryUnavailable, {}};
      assign_steering(population, *inputs, parameters);
      n::Box swept;
      population = n::advance_partitioned_inputs(
          std::move(population), acceleration_groups(*inputs), parameters, dt,
          &swept, &corners);
      const auto endpoint = n::joined(population);
      tube.maximum_body_partitions =
          std::max(tube.maximum_body_partitions, population.size());
      tube.source_to_rest.push_back({step * dt, (step + 1) * dt, begin.lo,
                                     end.hi, *inputs, ranges(swept),
                                     ranges(endpoint), ranges(corners.swept),
                                     ranges(n::joined(corners.states))});
      if (earliest_end.lo > rest_not_before && n::at_rest(endpoint)) {
        tube.rest_sec = end.hi;
        return {Reason::None, std::move(tube)};
      }
    }
    return {Reason::StepLimit, {}};
  } catch (const std::exception &) {
    return {Reason::NumericalFailure, {}};
  }
}

} // namespace

StartingDomainPrediction predict_starting_domain_to_rest(
    const StartingDomainRequest &request, const Parameters &parameters) noexcept {
  return predict_domain_to_rest(request, parameters,
      starting_domain_context_fingerprint(request, parameters));
}

std::uint64_t programme_starting_domain_context_fingerprint(
    const ProgrammeStartingDomainRequest &request,
    const Parameters &parameters) noexcept {
  const auto &domain = request.domain;
  const auto first = publication_epoch(domain.program, 0);
  if (!first || !std::isfinite(request.original_rest_sec) ||
      request.original_rest_sec < *first ||
      domain.starting_sec.upper > request.original_rest_sec ||
      !std::isfinite(domain.starting_sec.lower) ||
      !std::isfinite(domain.starting_sec.upper) ||
      domain.starting_sec.lower < *first ||
      domain.starting_sec.lower > domain.starting_sec.upper)
    return 0;
  // Reuse original model/history/body/offset validation, without changing the
  // first-window API. Hash the explicitly different time theorem separately.
  auto first_window = domain;
  first_window.starting_sec = {*first, *first};
  const auto base = starting_domain_context_fingerprint(first_window, parameters);
  if (!base) return 0;
  Hash hash;
  hash.string("independent-original-programme-starting-domain-v1");
  hash.integer(base);
  hash.number(domain.starting_sec.lower);
  hash.number(domain.starting_sec.upper);
  hash.number(request.original_rest_sec);
  return hash.value ? hash.value : 1;
}

ProgrammeStartingDomainPrediction predict_programme_starting_domain_to_rest(
    const ProgrammeStartingDomainRequest &request,
    const Parameters &parameters) noexcept {
  auto prediction = predict_domain_to_rest(request.domain, parameters,
      programme_starting_domain_context_fingerprint(request, parameters));
  if (!prediction.tube) return {prediction.reason, {}};
  return {prediction.reason, ProgrammeStartingDomainTube{
      std::move(*prediction.tube), request.original_rest_sec}};
}

RelativeProgrammeStartingDomainPrediction predict_relative_programme_domain_to_rest(
    const ProgrammeStartingDomainRequest &request, const Parameters &parameters) noexcept {
  // Keep original input history, time intervals, footprint and body population.
  // Pose normalization is a separate theorem, not a relaxed global membership.
  if (!finite(request.domain.coordinate_origin) || !valid_ranges(request.domain.body))
    return {Reason::InvalidObservation, {}};
  try {
    auto normalized = request;
    for (std::size_t i = 0; i < 3; ++i) normalized.domain.body[i] = {0, 0};
    normalized.domain.coordinate_origin.x_m = 0;
    normalized.domain.coordinate_origin.y_m = 0;
    normalized.domain.coordinate_origin.yaw_rad = 0;
    auto prediction = predict_programme_starting_domain_to_rest(normalized, parameters);
    if (!prediction.tube) return {prediction.reason, {}};
    Hash hash;
    hash.string("independent-relative-programme-body-time-domain-v1");
    hash.integer(prediction.tube->numerical.context_fingerprint);
    prediction.tube->numerical.context_fingerprint = hash.value ? hash.value : 1;
    return {Reason::None, RelativeProgrammeStartingDomainTube{std::move(*prediction.tube)}};
  } catch (const std::exception &) { return {Reason::NumericalFailure, {}}; }
}

std::optional<RelativeDomainTransform> RelativeDomainTransform::build(
    const RelativeProgrammeStartingDomainTube &domain, const CurrentInputPrefix &prefix) noexcept {
  const auto &tube = domain.normalized.numerical;
  const auto &request = tube.request;
  const auto &t = request.starting_sec;
  const auto now = prefix.observation.now_sec;
  if (!tube.context_fingerprint || tube.source_to_rest.empty() ||
      !finite(request.coordinate_origin) || !finite(prefix.coordinate_origin) ||
      !valid_ranges(request.body) || !valid_ranges(prefix.body) ||
      !valid_ranges(request.footprint_offsets) || !valid_ranges(prefix.footprint_offsets) ||
      !std::isfinite(t.lower) || !std::isfinite(t.upper) || t.lower > t.upper ||
      !std::isfinite(now) || now < t.lower || now > t.upper ||
      request.coordinate_origin.x_m != 0 || request.coordinate_origin.y_m != 0 ||
      request.coordinate_origin.yaw_rad != 0) return std::nullopt;
  for (std::size_t i = 0; i < 8; ++i) {
    if (request.footprint_offsets[i].lower != prefix.footprint_offsets[i].lower ||
        request.footprint_offsets[i].upper != prefix.footprint_offsets[i].upper) return std::nullopt;
    if (i < 3) {
      if (request.body[i].lower != 0 || request.body[i].upper != 0) return std::nullopt;
    } else if (prefix.body[i].lower < request.body[i].lower ||
               prefix.body[i].upper > request.body[i].upper) return std::nullopt;
  }
  try {
    const auto p = box(prefix.body);
    const auto co = n::cosine(p[2]), si = n::sine(p[2]);
    const auto yaw = n::I(prefix.coordinate_origin.yaw_rad);
    const auto oc = n::cosine(yaw), os = n::sine(yaw);
    const std::array<n::I, 9> values{p[0], p[1], p[2], co, si,
      oc * p[0] - os * p[1], os * p[0] + oc * p[1],
      n::cosine(yaw + p[2]), n::sine(yaw + p[2])};
    RelativeDomainTransform result;
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (!std::isfinite(values[i].lo) || !std::isfinite(values[i].hi)) return std::nullopt;
      result.coefficients_[i] = {values[i].lo, values[i].hi};
    }
    return result;
  } catch (const std::exception &) { return std::nullopt; }
}

BodyRanges RelativeDomainTransform::body(const BodyRanges &relative) const {
  auto b = box(relative);
  const auto q = [&](std::size_t i) { return n::I(coefficients_[i].lower, coefficients_[i].upper); };
  const auto x = b[0], y = b[1];
  b[0] = q(0) + q(3) * x - q(4) * y;
  b[1] = q(1) + q(4) * x + q(3) * y;
  b[2] = q(2) + b[2];
  return ranges(b);
}
FootprintRanges RelativeDomainTransform::footprint(const FootprintRanges &relative) const {
  auto b = box(relative);
  const auto q = [&](std::size_t i) { return n::I(coefficients_[i].lower, coefficients_[i].upper); };
  for (std::size_t i = 0; i < 8; i += 2) {
    const auto x = b[i], y = b[i + 1];
    b[i] = q(5) + q(7) * x - q(8) * y;
    b[i + 1] = q(6) + q(8) * x + q(7) * y;
  }
  return ranges(b);
}

bool starting_domain_contains_prefix(
    const StartingDomainRequest &domain,
    const CurrentInputPrefix &prefix) noexcept {
  const auto &t = domain.starting_sec;
  const double now = prefix.observation.now_sec;
  if (!finite(domain.coordinate_origin) || !finite(prefix.coordinate_origin) ||
      !valid_ranges(domain.body) || !valid_ranges(prefix.body) ||
      !valid_ranges(domain.footprint_offsets) ||
      !valid_ranges(prefix.footprint_offsets) || !std::isfinite(t.lower) ||
      !std::isfinite(t.upper) || t.lower > t.upper || !std::isfinite(now) ||
      now < t.lower || now > t.upper)
    return false;
  for (std::size_t i = 0; i < 8; ++i)
    if (domain.footprint_offsets[i].lower !=
            prefix.footprint_offsets[i].lower ||
        domain.footprint_offsets[i].upper != prefix.footprint_offsets[i].upper)
      return false;
  try {
    auto current = box(prefix.body);
    const auto &a = domain.coordinate_origin, &b = prefix.coordinate_origin;
    if (a.x_m != b.x_m || a.y_m != b.y_m || a.yaw_rad != b.yaw_rad) {
      const auto dx = n::I(b.x_m) - n::I(a.x_m), dy = n::I(b.y_m) - n::I(a.y_m);
      const auto co = n::cosine(n::I(a.yaw_rad)), so = n::sine(n::I(a.yaw_rad));
      const auto theta = n::I(b.yaw_rad) - n::I(a.yaw_rad),
                 cd = n::cosine(theta), sd = n::sine(theta);
      const auto x = current[0], y = current[1];
      current[0] = co * dx + so * dy + cd * x - sd * y;
      current[1] = -so * dx + co * dy + sd * x + cd * y;
      current[2] = theta + current[2];
    }
    for (std::size_t i = 0; i < 8; ++i)
      if (current[i].lo < domain.body[i].lower ||
          current[i].hi > domain.body[i].upper)
        return false;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
