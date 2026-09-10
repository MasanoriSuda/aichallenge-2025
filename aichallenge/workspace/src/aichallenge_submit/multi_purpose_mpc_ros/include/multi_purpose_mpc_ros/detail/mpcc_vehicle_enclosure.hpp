#pragma once
// Numerical enclosure of the shared native midpoint map. Callers own the
// input/observation context and proof. These primitives grant no authority.
#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_model_kernel.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
namespace multi_purpose_mpc_ros::mpcc_vehicle_model::numerical {
namespace model = multi_purpose_mpc_ros::mpcc_vehicle_model;
struct I {
  double lo{}, hi{};
  I() = default;
  I(double v) : lo(v), hi(v) {}
  I(double l, double h) : lo(l), hi(h) {}
};
inline double adjacent(double x, bool upward) {
  static_assert(std::numeric_limits<double>::is_iec559 &&
                sizeof(double) == sizeof(std::uint64_t));
  if (std::isnan(x) || x == (upward ? INFINITY : -INFINITY))
    return x;
  if (x == 0)
    return upward ? std::numeric_limits<double>::denorm_min()
                  : -std::numeric_limits<double>::denorm_min();
  std::uint64_t bits;
  std::memcpy(&bits, &x, sizeof x);
  if ((x > 0) == upward)
    ++bits;
  else
    --bits;
  std::memcpy(&x, &bits, sizeof x);
  return x;
}
inline double down(double x) { return adjacent(x, false); }
inline double up(double x) { return adjacent(x, true); }
inline I operator+(I a, I b) {
  if (a.lo == 0 && a.hi == 0)
    return b;
  if (b.lo == 0 && b.hi == 0)
    return a;
  return {down(a.lo + b.lo), up(a.hi + b.hi)};
}
inline I operator-(I a, I b) { return {down(a.lo - b.hi), up(a.hi - b.lo)}; }
inline I operator-(I a) { return {-a.hi, -a.lo}; }
inline I operator*(I a, I b) {
  if ((a.lo == 0 && a.hi == 0) || (b.lo == 0 && b.hi == 0))
    return I(0);
  const std::array<double, 4> p{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo,
                                a.hi * b.hi};
  return {down(*std::min_element(p.begin(), p.end())),
          up(*std::max_element(p.begin(), p.end()))};
}
inline I operator/(I a, double b) {
  if (b == 0)
    throw std::runtime_error("zero divisor");
  return a * I(down(1 / b), up(1 / b));
}
inline I hull(I a, I b) { return {std::min(a.lo, b.lo), std::max(a.hi, b.hi)}; }
inline I sine(I a) {
  if (a.hi - a.lo >= 2 * M_PI)
    return {-1, 1};
  double l = std::min(std::sin(a.lo), std::sin(a.hi)),
         h = std::max(std::sin(a.lo), std::sin(a.hi));
  if (std::ceil((a.lo - M_PI / 2) / (2 * M_PI)) <=
      std::floor((a.hi - M_PI / 2) / (2 * M_PI)))
    h = 1;
  if (std::ceil((a.lo + M_PI / 2) / (2 * M_PI)) <=
      std::floor((a.hi + M_PI / 2) / (2 * M_PI)))
    l = -1;
  return {std::max(-1., down(down(l))), std::min(1., up(up(h)))};
}
inline I cosine(I a) {
  if (a.hi - a.lo >= 2 * M_PI)
    return {-1, 1};
  double l = std::min(std::cos(a.lo), std::cos(a.hi)),
         h = std::max(std::cos(a.lo), std::cos(a.hi));
  if (std::ceil(a.lo / (2 * M_PI)) <= std::floor(a.hi / (2 * M_PI)))
    h = 1;
  if (std::ceil((a.lo - M_PI) / (2 * M_PI)) <=
      std::floor((a.hi - M_PI) / (2 * M_PI)))
    l = -1;
  return {std::max(-1., down(down(l))), std::min(1., up(up(h)))};
}
constexpr size_t N =
    7; // yaw/body/steering plus input; Cartesian translation is exact
struct J {
  I v;
  std::array<I, N> d{};
  J() = default;
  J(double x) : v(x) {}
  J(I x) : v(x){};
};
inline J operator+(const J &a, const J &b) {
  J r(a.v + b.v);
  for (size_t i = 0; i < N; ++i)
    r.d[i] = a.d[i] + b.d[i];
  return r;
}
inline J operator-(const J &a, const J &b) {
  J r(a.v - b.v);
  for (size_t i = 0; i < N; ++i)
    r.d[i] = a.d[i] - b.d[i];
  return r;
}
inline J operator-(const J &a) {
  J r(-a.v);
  for (size_t i = 0; i < N; ++i)
    r.d[i] = -a.d[i];
  return r;
}
inline J operator*(const J &a, const J &b) {
  J r(a.v * b.v);
  for (size_t i = 0; i < N; ++i)
    r.d[i] = a.d[i] * b.v + a.v * b.d[i];
  return r;
}
inline J operator/(const J &a, double b) {
  J r(a.v / b);
  for (size_t i = 0; i < N; ++i)
    r.d[i] = a.d[i] / b;
  return r;
}
inline J sin(const J &a) {
  J r(sine(a.v));
  const I derivative = cosine(a.v);
  for (size_t i = 0; i < N; ++i)
    r.d[i] = derivative * a.d[i];
  return r;
}
inline J cos(const J &a) {
  J r(cosine(a.v));
  const I derivative = -sine(a.v);
  for (size_t i = 0; i < N; ++i)
    r.d[i] = derivative * a.d[i];
  return r;
}
inline J clamp(J a, double lo, double hi) {
  if (a.v.hi <= lo)
    return J(lo);
  if (a.v.lo >= hi)
    return J(hi);
  if (a.v.lo < lo || a.v.hi > hi)
    for (auto &d : a.d)
      d = hull(d, I(0));
  a.v = {std::max(a.v.lo, lo), std::min(a.v.hi, hi)};
  return a;
}
inline J max(const J &a, const J &b) {
  if (a.v.lo >= b.v.hi)
    return a;
  if (b.v.lo >= a.v.hi)
    return b;
  J r(I(std::max(a.v.lo, b.v.lo), std::max(a.v.hi, b.v.hi)));
  for (size_t i = 0; i < N; ++i)
    r.d[i] = hull(a.d[i], b.d[i]);
  return r;
}
inline I bounds(I a) { return a; }
inline I bounds(const J &a) { return a.v; }
inline I sin(I a) { return sine(a); }
inline I cos(I a) { return cosine(a); }
inline I clamp(I a, double lo, double hi) {
  return {std::clamp(a.lo, lo, hi), std::clamp(a.hi, lo, hi)};
}
inline I max(I a, I b) { return {std::max(a.lo, b.lo), std::max(a.hi, b.hi)}; }
inline I unite(I a, I b) { return hull(a, b); }
inline J unite(const J &a, const J &b) {
  J r(hull(a.v, b.v));
  for (size_t i = 0; i < N; ++i)
    r.d[i] = hull(a.d[i], b.d[i]);
  return r;
}
inline void intersect(I &a, I b) {
  a = {std::max(a.lo, b.lo), std::min(a.hi, b.hi)};
  if (a.lo > a.hi)
    throw std::runtime_error("disjoint independent enclosures");
}
inline void intersect(J &a, I b) { intersect(a.v, b); }
using JS = std::array<J, 8>;
using Box = std::array<I, 8>;
// Four rigid footprint vertices, XY pairs in world axes relative to the
// observation XY origin. Body boxes retain their original heading frame.
struct CornerProbe { Box offsets; double origin_yaw{}; };
struct CornerImage { Box endpoint; Box swept; };
struct CornerPopulation {
  CornerProbe probe;
  std::vector<Box> states;
  Box swept;
};
// Only nonsmooth scalar/set arithmetic differs. All wheel, body and tire
// equations below come from the same kernel as the production native model.
template <class T> struct Arithmetic {
  bool *discontinuous{};
  T sin(const T &a) const { return numerical::sin(a); }
  T cos(const T &a) const { return numerical::cos(a); }
  T clamp(const T &a, double lo, double hi) const {
    return numerical::clamp(a, lo, hi);
  }
  T reverse_correction(const T &u, const T &a, double interval) const {
    if (bounds(u).hi < 0)
      return max(a, -u / interval);
    if (bounds(u).lo < 0) {
      if (discontinuous && bounds(a).lo < 0)
        *discontinuous = true;
      return unite(a, max(a, -u / interval));
    }
    return a;
  }
  T rolling(const T &u, double interval, double limit) const {
    return clamp(-u / interval, -limit, limit);
  }
};
inline I tire_bounds(I desired, I tire, const model::Parameters &p, double dt) {
  // alpha=dt/(lag+dt) is in[0,1]. With positive gain/grip, the shared
  // clipped/slew-limited tire response is monotone in both input angles:
  // the desired-angle slope is nonnegative; the prior-tire slope is either
  // 1 or 1-alpha. Both input extrema therefore occur at opposite corners.
  // Evaluate just the two opposite corners using outward arithmetic. This
  // refines values only; the existing Jacobian derivative ranges remain.
  model::kernel::StateValues<I> low{}, high{};
  low[model::kernel::Desired] = I(desired.lo);
  low[model::kernel::Tire] = I(tire.lo);
  high[model::kernel::Desired] = I(desired.hi);
  high[model::kernel::Tire] = I(tire.hi);
  model::kernel::update_tire(low, p, dt, Arithmetic<I>{});
  model::kernel::update_tire(high, p, dt, Arithmetic<I>{});
  return {low[model::kernel::Tire].lo, high[model::kernel::Tire].hi};
}
template <class T>
inline std::array<T, 8> map(std::array<T, 8> s, T a, const model::Parameters &p,
                            double dt, bool rest,
                            bool *discontinuous = nullptr, T *yaw_increment = nullptr) {
  const Arithmetic<T> arithmetic{discontinuous};
  const auto tire = tire_bounds(bounds(s[6]), bounds(s[7]), p, dt);
  model::kernel::update_tire(s, p, dt, arithmetic);
  intersect(s[7], tire);
  if (rest) {
    if (yaw_increment) *yaw_increment = T(0);
    s[3] = T(0);
    s[4] = T(0);
    s[5] = T(0);
    return s;
  }
  const auto d1 = model::kernel::derivative(s, a, p, dt, arithmetic);
  const auto mid = model::kernel::body_increment(s, d1, .5 * dt);
  const auto rates = model::kernel::derivative(mid, a, p, dt, arithmetic);
  if (yaw_increment) *yaw_increment = rates[model::kernel::Yaw] * T(dt);
  return model::kernel::body_increment(s, rates, dt);
}

inline CornerImage advance_corners(
    const CornerProbe &probe, const Box &previous, const JS &range,
    const Box &point, const JS &inputs, const Box &center,
    const std::array<I, N> &offsets, const J &delta, I point_delta,
    bool discontinuous) {
  const auto enclose = [&](const J &value, I midpoint) {
    I image = midpoint;
    for (size_t j = 0; j < N; ++j) image = image + value.d[j] * offsets[j];
    if (discontinuous) image = value.v;
    else intersect(image, value.v);
    return image;
  };
  const I co = cosine(I(probe.origin_yaw)), so = sine(I(probe.origin_yaw));
  const J half_delta = delta / 2;
  const J mid_angle = J(I(probe.origin_yaw)) + inputs[2] + half_delta;
  // Preserve the common angle in both values and interval derivatives.
  const J dc = J(-2) * sin(mid_angle) * sin(half_delta);
  const J ds = J(2) * cos(mid_angle) * sin(half_delta);
  const I point_mid = I(probe.origin_yaw) + center[2] + point_delta / 2;
  const I pdc = I(-2) * sine(point_mid) * sine(point_delta / 2);
  const I pds = I(2) * cosine(point_mid) * sine(point_delta / 2);
  const I turn = enclose(delta, point_delta);
  const double max_turn = std::max(std::abs(turn.lo), std::abs(turn.hi));
  CornerImage out;
  for (size_t k = 0; k < 8; k += 2) {
    const I x = probe.offsets[k], y = probe.offsets[k + 1];
    const J dx = J(co) * range[0] - J(so) * range[1] + J(x) * dc - J(y) * ds;
    const J dy = J(so) * range[0] + J(co) * range[1] + J(x) * ds + J(y) * dc;
    const I px = co * point[0] - so * point[1] + x * pdc - y * pds;
    const I py = so * point[0] + co * point[1] + x * pds + y * pdc;
    out.endpoint[k] = previous[k] + enclose(dx, px);
    out.endpoint[k + 1] = previous[k + 1] + enclose(dy, py);
    const I radius2 = I(std::max(std::abs(x.lo), std::abs(x.hi))) *
                       I(std::max(std::abs(x.lo), std::abs(x.hi))) +
                     I(std::max(std::abs(y.lo), std::abs(y.hi))) *
                       I(std::max(std::abs(y.lo), std::abs(y.hi)));
    const double radius = up(std::sqrt(radius2.hi));
    const double sag = (I(radius) * I(max_turn) * I(max_turn) * I(.125)).hi;
    out.swept[k] = hull(previous[k], out.endpoint[k]) + I(-sag, sag);
    out.swept[k + 1] = hull(previous[k + 1], out.endpoint[k + 1]) + I(-sag, sag);
  }
  for (const auto *values : {&out.endpoint, &out.swept})
    for (const auto &value : *values)
      if (!std::isfinite(value.lo) || !std::isfinite(value.hi) || value.lo > value.hi)
        throw std::runtime_error("invalid corner enclosure");
  return out;
}

inline Box centered_step(const Box &b, I acceleration,
                         const model::Parameters &p, double dt, bool rest,
                         const CornerProbe *probe = nullptr,
                         const Box *previous_corners = nullptr,
                         CornerImage *corner_image = nullptr) {
  if ((probe != nullptr) != (previous_corners != nullptr) ||
      (probe != nullptr) != (corner_image != nullptr))
    throw std::runtime_error("incomplete corner context");
  if (probe) {
    if (!std::isfinite(probe->origin_yaw))
      throw std::runtime_error("invalid corner origin");
    for (const auto *values : {&probe->offsets, previous_corners})
      for (const auto &value : *values)
        if (!std::isfinite(value.lo) || !std::isfinite(value.hi) || value.lo > value.hi)
          throw std::runtime_error("invalid corner context");
  }
  if (rest) {
    if (corner_image) *corner_image = {*previous_corners, *previous_corners};
    // The selected native rest branch preserves pose and desired angle,
    // zeros body motion and updates only the tire. Its exact interval image
    // needs no body Jacobian or centered subtraction/readdition. Reuse the
    // same independently bounded tire formula; no rest threshold changes.
    auto result = b;
    result[3] = result[4] = result[5] = I(0);
    result[7] = tire_bounds(b[6], b[7], p, dt);
    for (const auto & value : result) {
      if (!std::isfinite(value.lo) || !std::isfinite(value.hi) || value.lo > value.hi)
        throw std::runtime_error("unbounded numerical enclosure");
    }
    return result;
  }
  JS inputs;
  Box center;
  std::array<I, N> offsets{};
  for (size_t i = 2; i < 8; ++i) {
    const double c = b[i].lo + (b[i].hi - b[i].lo) / 2;
    inputs[i] = J(b[i]);
    inputs[i].d[i - 2] = I(1);
    center[i] = I(c);
    offsets[i - 2] = b[i] - I(c);
  }
  const double ac = acceleration.lo + (acceleration.hi - acceleration.lo) / 2;
  J a(acceleration);
  a.d[6] = I(1);
  offsets[6] = acceleration - I(ac);
  bool discontinuous = false;
  J yaw_delta;
  I point_yaw_delta;
  const auto range = map(inputs, a, p, dt, rest, &discontinuous,
                         corner_image ? &yaw_delta : nullptr);
  const auto point = map(center, I(ac), p, dt, rest, nullptr,
                         corner_image ? &point_yaw_delta : nullptr);
  Box result;
  for (size_t i = 0; i < 8; ++i) {
    I delta;
    for (size_t j = 0; j < N; ++j)
      delta = delta + range[i].d[j] * offsets[j];
    result[i] = discontinuous ? range[i].v : point[i] + delta;
    // Both mean-value and direct natural extensions contain the same image.
    // Their intersection cannot remove a represented physical response.
    intersect(result[i], range[i].v);
    if (i < 2)
      result[i] = b[i] + result[i];
    if (rest && i >= 3 && i <= 5)
      result[i] = I(0);
    if (!std::isfinite(result[i].lo) || !std::isfinite(result[i].hi))
      throw std::runtime_error("unbounded numerical enclosure");
  }
  intersect(result[7], tire_bounds(b[6], b[7], p, dt));
  if (corner_image)
    *corner_image = advance_corners(*probe, *previous_corners, range, point,
      inputs, center, offsets, yaw_delta, point_yaw_delta, discontinuous);
  return result;
}
inline std::vector<Box> step_parts(const Box &b, I a,
                                   const model::Parameters &p, double dt,
                                   const CornerProbe *probe = nullptr,
                                   const Box *previous_corners = nullptr,
                                   std::vector<CornerImage> *corner_images = nullptr) {
  if ((probe != nullptr) != (previous_corners != nullptr) ||
      (probe != nullptr) != (corner_images != nullptr))
    throw std::runtime_error("incomplete corner branches");
  if (!model::valid(p) || !p.nominal_settled_contact || dt <= 0 ||
      model::integration_steps(dt, p.maximum_step_sec) != 1)
    throw std::runtime_error("unsupported enclosure context");
  std::vector<Box> parts;
  const auto add = [&](double low, double high, I wire, bool rest) {
    Box q = b;
    q[3] = {std::max(b[3].lo, low), std::min(b[3].hi, high)};
    if (q[3].lo > q[3].hi || wire.lo > wire.hi)
      return;
    CornerImage image;
    parts.push_back(centered_step(q, wire, p, dt, rest, probe, previous_corners,
                                 corner_images ? &image : nullptr));
    if (corner_images) corner_images->push_back(image);
  };
  if (a.lo <= 0) {
    const I wire(a.lo, std::min(0., a.hi));
    add(-p.sleep_speed_mps, p.sleep_speed_mps, wire, true);
    add(-INFINITY, -p.sleep_speed_mps, wire, false);
    add(p.sleep_speed_mps, INFINITY, wire, false);
  }
  if (a.hi > 0) {
    const I wire(std::max(0., a.lo), a.hi);
    add(-INFINITY, 0, wire, false);
    add(0, INFINITY, wire, false);
  }
  if (parts.empty())
    throw std::runtime_error("empty branch enclosure");
  return parts;
}
inline Box joined(const std::vector<Box> &parts) {
  if (parts.empty())
    throw std::runtime_error("empty body population");
  Box out = parts.front();
  for (size_t k = 1; k < parts.size(); ++k)
    for (size_t i = 0; i < 8; ++i)
      out[i] = hull(out[i], parts[k][i]);
  return out;
}
inline bool at_rest(const Box &s) {
  return s[3].lo == 0 && s[3].hi == 0 && s[4].lo == 0 && s[4].hi == 0 &&
         s[5].lo == 0 && s[5].hi == 0;
}
inline Box step(const Box &b, I a, const model::Parameters &p, double dt) {
  return joined(step_parts(b, a, p, dt));
}
inline std::vector<Box> advance_partitioned_inputs(std::vector<Box> states,
                                            const std::vector<I> & inputs,
                                            const model::Parameters &p,
                                            double duration, Box *swept,
                                            CornerPopulation *corners = nullptr) {
  if (corners && corners->states.size() != states.size())
    throw std::runtime_error("corner population mismatch");
  const auto count = model::integration_steps(duration, p.maximum_step_sec);
  if (count == 0 || inputs.empty())
    throw std::runtime_error("invalid partitioned duration");
  if (swept)
    *swept = joined(states);
  if (corners) corners->swept = joined(corners->states);
  const auto merge = [](std::optional<Box> &to, const Box &from) {
    if (!to)
      to = from;
    else
      for (size_t i = 0; i < 8; ++i)
        (*to)[i] = hull((*to)[i], from[i]);
  };
  for (size_t k = 0; k < count; ++k) {
    // Rest is a separate hybrid mode. Merging it with moving states before
    // the next map invents combinations of zero u and moving vy/r, which can
    // make a valid enclosure useless. Splitting/merging changes no trajectory.
    std::array<std::optional<Box>, 5> bins, corner_bins;
    const std::array<double, 5> boundaries{-INFINITY, -p.sleep_speed_mps, 0,
                                           p.sleep_speed_mps, INFINITY};
    // Each input arm is evaluated before merging body modes. Convexifying
    // across absent acceleration signs here invents new hybrid responses.
    for (const auto & a : inputs)
      for (size_t state_index = 0; state_index < states.size(); ++state_index) {
        std::vector<CornerImage> images;
        const auto parts = step_parts(states[state_index], a, p, duration / count,
          corners ? &corners->probe : nullptr,
          corners ? &corners->states[state_index] : nullptr,
          corners ? &images : nullptr);
        for (size_t part_index = 0; part_index < parts.size(); ++part_index) {
          const auto &next = parts[part_index];
          if (corners)
            for (size_t i = 0; i < 8; ++i)
              corners->swept[i] = hull(corners->swept[i], images[part_index].swept[i]);
          if (at_rest(next)) {
            merge(bins[0], next);
            if (corners) merge(corner_bins[0], images[part_index].endpoint);
            continue;
          }
          for (size_t j = 0; j < 4; ++j) {
            Box clipped = next;
            clipped[3] = {std::max(next[3].lo, boundaries[j]),
                          std::min(next[3].hi, boundaries[j + 1])};
            if (clipped[3].lo <= clipped[3].hi) {
              merge(bins[j + 1], clipped);
              if (corners) merge(corner_bins[j + 1], images[part_index].endpoint);
            }
          }
        }
      }
    states.clear();
    if (corners) corners->states.clear();
    for (size_t i = 0; i < bins.size(); ++i)
      if (bins[i]) {
        states.push_back(*bins[i]);
        if (corners) corners->states.push_back(*corner_bins[i]);
      }
    if (swept) {
      const auto all = joined(states);
      for (size_t i = 0; i < 8; ++i)
        (*swept)[i] = hull((*swept)[i], all[i]);
    }
  }
  return states;
}
inline std::vector<Box> advance_partitioned(std::vector<Box> states, I a,
                                            const model::Parameters &p,
                                            double duration, Box *swept) {
  // Preserve the full continuous-interval primitive and its callers.
  return advance_partitioned_inputs(std::move(states), {a}, p, duration, swept);
}
inline Box advance(Box state, I acceleration, const model::Parameters &p,
                   double duration, Box *swept = nullptr) {
  if (!std::isfinite(duration) || duration < 0)
    throw std::runtime_error("invalid range duration");
  if (swept)
    *swept = state;
  if (duration == 0)
    return state;
  const auto count = model::integration_steps(duration, p.maximum_step_sec);
  if (count == 0)
    throw std::runtime_error("invalid range step count");
  const double dt = duration / static_cast<double>(count);
  for (size_t i = 0; i < count; ++i) {
    state = step(state, acceleration, p, dt);
    if (swept)
      for (size_t j = 0; j < state.size(); ++j)
        (*swept)[j] = hull((*swept)[j], state[j]);
  }
  return state;
}
inline std::array<double, 8> values(const model::State &s) {
  return {s.x_m,
          s.y_m,
          s.yaw_rad,
          s.forward_velocity_mps,
          s.lateral_velocity_mps,
          s.yaw_rate_radps,
          s.desired_steering_rad,
          s.tire_steering_rad};
}
inline Box point(const model::State &s) {
  Box b;
  auto v = values(s);
  for (size_t i = 0; i < 8; ++i)
    b[i] = I(v[i]);
  return b;
}
} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model::numerical
