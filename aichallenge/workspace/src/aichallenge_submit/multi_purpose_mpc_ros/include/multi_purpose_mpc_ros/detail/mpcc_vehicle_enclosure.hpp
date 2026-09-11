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
#include <optional>
#include <type_traits>
#include <stdexcept>
#include <utility>
#include <vector>
#if defined(__SSE2__)
#include <emmintrin.h>
#endif
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
// Exactly the scalar down/up bit step, evaluated for both endpoints together.
// Keep the scalar fallback and the special/random-bit adjacency oracle.
inline I outward_pair(double lower, double upper) {
#if defined(__SSE2__)
  const __m128d value = _mm_set_pd(upper, lower);
  const __m128i bits = _mm_castpd_si128(value);
  const __m128i positive = _mm_castpd_si128(_mm_cmpgt_pd(value, _mm_setzero_pd()));
  const __m128i positive_step = _mm_set_epi64x(1, -1);
  const __m128i negative_step = _mm_set_epi64x(-1, 1);
  const __m128i step = _mm_or_si128(_mm_and_si128(positive, positive_step),
    _mm_andnot_si128(positive, negative_step));
  __m128i next = _mm_add_epi64(bits, step);
  const __m128i zero = _mm_castpd_si128(_mm_cmpeq_pd(value, _mm_setzero_pd()));
  const __m128i zero_next = _mm_set_epi64x(1, std::numeric_limits<std::int64_t>::min() + 1);
  next = _mm_or_si128(_mm_and_si128(zero, zero_next), _mm_andnot_si128(zero, next));
  const __m128i unchanged = _mm_castpd_si128(_mm_or_pd(_mm_cmpunord_pd(value, value),
    _mm_cmpeq_pd(value, _mm_set_pd(INFINITY, -INFINITY))));
  next = _mm_or_si128(_mm_and_si128(unchanged, bits), _mm_andnot_si128(unchanged, next));
  double result[2];
  _mm_storeu_pd(result, _mm_castsi128_pd(next));
  return {result[0], result[1]};
#else
  return {down(lower), up(upper)};
#endif
}

inline I operator+(I a, I b) {
  if (a.lo == 0 && a.hi == 0)
    return b;
  if (b.lo == 0 && b.hi == 0)
    return a;
  return outward_pair(a.lo + b.lo, a.hi + b.hi);
}
inline I operator-(I a, I b) { return outward_pair(a.lo - b.hi, a.hi - b.lo); }
inline I operator-(I a) { return {-a.hi, -a.lo}; }
inline I operator*(I a, I b) {
  if ((a.lo == 0 && a.hi == 0) || (b.lo == 0 && b.hi == 0))
    return I(0);
  const std::array<double, 4> p{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo,
                                a.hi * b.hi};
  return outward_pair(*std::min_element(p.begin(), p.end()), *std::max_element(p.begin(), p.end()));
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
// value equations below come from the production native model kernel. Force
// derivative coefficients are independently checked against that kernel.
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
// The wheel force is affine in [forward, left, yaw rate, drive + rolling]
// at a fixed tire angle. These interval matrices and their angle partials use
// outward arithmetic. Values still come from kernel::derivative verbatim.
// Keep the high-precision shared-kernel regression when changing either side.
struct ForceCoefficients {
  // Owned by one map call; never retained across steps or requests.
  std::array<I, 4> x{}, y{}, moment{}, angle_x{}, angle_y{}, angle_moment{};
  I tire;
  const model::Parameters *parameters{};
};
inline ForceCoefficients force_coefficients(I tire,
                                            const model::Parameters &p) {
  ForceCoefficients out;
  out.tire = tire;
  out.parameters = &p;
  for (const auto &w : p.wheels) {
    const I theta = w.steerable ? tire : I(0), c = cosine(theta),
            sn = sine(theta);
    const I K(w.cornering_per_sec), T(w.traction_fraction), x(w.com_forward_m),
        y(w.com_left_m);
    const I arm = sn * y + c * x;
    const std::array<I, 4> ax{-K * sn * sn, K * sn * c, K * sn * arm, c * T};
    const std::array<I, 4> ay{K * c * sn, -K * c * c, -K * c * arm, sn * T};
    std::array<I, 4> dx{}, dy{};
    if (w.steerable) {
      const I twice = I(2) * sn * c, difference = c * c - sn * sn;
      dx = {-K * twice, K * difference, K * (twice * y + difference * x),
            -T * sn};
      dy = {K * difference, K * twice, -K * (difference * y - twice * x),
            T * c};
    }
    for (size_t i = 0; i < 4; ++i) {
      out.x[i] = out.x[i] + ax[i];
      out.y[i] = out.y[i] + ay[i];
      out.moment[i] = out.moment[i] + x * ay[i] - y * ax[i];
      out.angle_x[i] = out.angle_x[i] + dx[i];
      out.angle_y[i] = out.angle_y[i] + dy[i];
      out.angle_moment[i] = out.angle_moment[i] + x * dy[i] - y * dx[i];
    }
  }
  return out;
}
template <class T>
inline model::kernel::BodyRates<T>
derivative_map(const model::kernel::StateValues<T> &s, const T &wire,
               const model::Parameters &p, double force_interval,
               const Arithmetic<T> &arithmetic, const ForceCoefficients *) {
  return model::kernel::derivative(s, wire, p, force_interval, arithmetic);
}
inline model::kernel::BodyRates<J>
derivative_map(const JS &s, const J &wire, const model::Parameters &p,
               double force_interval, const Arithmetic<J> &arithmetic,
               const ForceCoefficients *coefficients) {
  namespace k = model::kernel;
  if (!coefficients || coefficients->parameters != &p ||
      coefficients->tire.lo != s[k::Tire].v.lo ||
      coefficients->tire.hi != s[k::Tire].v.hi)
    throw std::runtime_error("force coefficient context mismatch");
  const auto &q = *coefficients;
  const auto &u = s[k::Forward];
  const auto &v = s[k::Left];
  const auto &r = s[k::YawRate];
  const double interval =
      std::max(force_interval, p.minimum_force_interval_sec);
  J acceleration = arithmetic.clamp(wire, -p.maximum_wire_deceleration_mps2,
                                    p.maximum_wire_acceleration_mps2);
  acceleration = arithmetic.reverse_correction(u, acceleration, interval);
  acceleration =
      arithmetic.clamp(acceleration, -p.maximum_wire_deceleration_mps2,
                       p.maximum_wire_acceleration_mps2);
  const J total =
      acceleration + arithmetic.rolling(u, interval, p.rolling_mps2);
  const std::array<I, 4> value{u.v, v.v, r.v, total.v};
  I angle_x, angle_y, angle_moment;
  for (size_t i = 0; i < 4; ++i) {
    angle_x = angle_x + q.angle_x[i] * value[i];
    angle_y = angle_y + q.angle_y[i] * value[i];
    angle_moment = angle_moment + q.angle_moment[i] * value[i];
  }
  model::kernel::BodyRates<J> result;
  const J reference_forward = u + r * J(p.com_left_m),
          reference_left = v - r * J(p.com_forward_m);
  const J c = cos(s[k::Yaw]), sn = sin(s[k::Yaw]);
  result[0] = c * reference_forward - sn * reference_left;
  result[1] = sn * reference_forward + c * reference_left;
  result[2] = r;
  for (size_t j = 0; j < N; ++j) {
    const std::array<I, 4> ds{u.d[j], v.d[j], r.d[j], total.d[j]};
    I dx, dy, dm;
    for (size_t i = 0; i < 4; ++i) {
      dx = dx + q.x[i] * ds[i];
      dy = dy + q.y[i] * ds[i];
      dm = dm + q.moment[i] * ds[i];
    }
    dx = dx + angle_x * s[k::Tire].d[j];
    dy = dy + angle_y * s[k::Tire].d[j];
    dm = dm + angle_moment * s[k::Tire].d[j];
    result[3].d[j] =
        dx - I(p.drag_per_sec) * u.d[j] + (r.d[j] * v.v + r.v * v.d[j]);
    result[4].d[j] =
        dy - I(p.drag_per_sec) * v.d[j] - (r.d[j] * u.v + r.v * u.d[j]);
    result[5].d[j] = dm * I(p.mass_kg) / p.yaw_inertia_kgm2 -
                     I(p.angular_drag_per_sec) * r.d[j];
  }
  // Preserve the natural interval extension and hybrid branch observations.
  k::StateValues<I> values;
  for (size_t i = 0; i < 8; ++i)
    values[i] = s[i].v;
  const auto original = k::derivative(values, wire.v, p, force_interval,
                                      Arithmetic<I>{arithmetic.discontinuous});
  for (size_t i = 0; i < result.size(); ++i)
    result[i].v = original[i];
  return result;
}

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
  // body_increment changes only coordinates X..YawRate. Both midpoint stages
  // therefore share exactly the updated tire interval and immutable model.
  std::optional<ForceCoefficients> coefficients;
  if constexpr (std::is_same_v<T, J>) {
    coefficients = force_coefficients(bounds(s[7]), p);
  }
  const auto d1 = derivative_map(s, a, p, dt, arithmetic,
                                 coefficients ? &*coefficients : nullptr);
  const auto mid = model::kernel::body_increment(s, d1, .5 * dt);
  const auto rates = derivative_map(mid, a, p, dt, arithmetic,
                                    coefficients ? &*coefficients : nullptr);
  if (yaw_increment)
    *yaw_increment = rates[model::kernel::Yaw] * T(dt);
  return model::kernel::body_increment(s, rates, dt);
}

inline CornerImage
advance_corners(const CornerProbe &probe, const Box &previous, const JS &range,
                const Box &point, const JS &inputs, const Box &center,
                const std::array<I, N> &offsets, const J &delta, I point_delta,
                bool discontinuous) {
  const auto enclose = [&](const J &value, I midpoint) {
    I image = midpoint;
    for (size_t j = 0; j < N; ++j)
      image = image + value.d[j] * offsets[j];
    if (discontinuous)
      image = value.v;
    else
      intersect(image, value.v);
    return image;
  };
  const I co = cosine(I(probe.origin_yaw)), so = sine(I(probe.origin_yaw));
  const J half_delta = delta / 2;
  const J mid_angle = J(I(probe.origin_yaw)) + inputs[2] + half_delta;
  // Preserve the common angle in both values and interval derivatives.
  const J half_sine = sin(half_delta);
  const J dc = J(-2) * sin(mid_angle) * half_sine;
  const J ds = J(2) * cos(mid_angle) * half_sine;
  const I point_mid = I(probe.origin_yaw) + center[2] + point_delta / 2;
  const I pdc = I(-2) * sine(point_mid) * sine(point_delta / 2);
  const I pds = I(2) * cosine(point_mid) * sine(point_delta / 2);
  const I turn = enclose(delta, point_delta);
  const double max_turn = std::max(std::abs(turn.lo), std::abs(turn.hi));
  const J world_x = J(co) * range[0] - J(so) * range[1];
  const J world_y = J(so) * range[0] + J(co) * range[1];
  const I point_x = co * point[0] - so * point[1];
  const I point_y = so * point[0] + co * point[1];
  CornerImage out;
  for (size_t k = 0; k < 8; k += 2) {
    const I x = probe.offsets[k], y = probe.offsets[k + 1];
    const J dx = world_x + J(x) * dc - J(y) * ds;
    const J dy = world_y + J(x) * ds + J(y) * dc;
    const I px = point_x + x * pdc - y * pds;
    const I py = point_y + x * pds + y * pdc;
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
                                            const std::vector<I> &inputs,
                                            const model::Parameters &p,
                                            double duration, Box *swept,
                                            CornerPopulation *corners = nullptr) {
  if (corners && corners->states.size() != states.size())
    throw std::runtime_error("corner population mismatch");
  const auto count = model::integration_steps(duration, p.maximum_step_sec);
  if (count == 0 || inputs.empty())
    throw std::runtime_error("invalid partitioned duration");
  if (swept) *swept = joined(states);
  if (corners) corners->swept = joined(corners->states);
  const auto merge = [](std::optional<Box> &to, const Box &from) {
    if (!to) to = from;
    else for (size_t i = 0; i < 8; ++i) (*to)[i] = hull((*to)[i], from[i]);
  };
  for (size_t step = 0; step < count; ++step) {
    std::vector<Box> parts;
    std::vector<CornerImage> images;
    parts.reserve(states.size() * inputs.size() * 4);
    if (corners) images.reserve(parts.capacity());
    double forward_low = INFINITY, forward_high = -INFINITY;
    // Evaluate every original input arm and native hybrid branch first.
    for (const auto &a : inputs)
      for (size_t j = 0; j < states.size(); ++j) {
        std::vector<CornerImage> branch_images;
        const auto next = step_parts(states[j], a, p, duration / count,
          corners ? &corners->probe : nullptr,
          corners ? &corners->states[j] : nullptr,
          corners ? &branch_images : nullptr);
        for (size_t i = 0; i < next.size(); ++i) {
          parts.push_back(next[i]);
          if (corners) images.push_back(branch_images[i]);
          if (!at_rest(next[i]) && next[i][3].hi > p.sleep_speed_mps) {
            forward_low = std::min(forward_low,
              std::max(p.sleep_speed_mps, next[i][3].lo));
            forward_high = std::max(forward_high, next[i][3].hi);
          }
        }
      }
    // Keeping rest alone is insufficient: merging all forward velocities
    // loses velocity/pose dependence before the next nonlinear map. Split
    // that interval at its midpoint. Both closed sides retain the boundary;
    // no speed, input, or physical model branch is excluded or changed.
    std::array<double, 6> cuts{-INFINITY, -p.sleep_speed_mps, 0,
                               p.sleep_speed_mps, INFINITY, INFINITY};
    size_t cut_count = 5;
    if (forward_high > forward_low) {
      const double midpoint = forward_low + (forward_high - forward_low) / 2;
      if (!std::isfinite(midpoint)) throw std::runtime_error("invalid forward partition");
      if (midpoint > forward_low && midpoint < forward_high) {
        cuts[4] = midpoint;
        cut_count = 6;
      }
    }
    std::array<std::optional<Box>, 6> bins, corner_bins;
    for (size_t i = 0; i < parts.size(); ++i) {
      const auto &next = parts[i];
      if (corners)
        for (size_t k = 0; k < 8; ++k)
          corners->swept[k] = hull(corners->swept[k], images[i].swept[k]);
      if (at_rest(next)) {
        merge(bins[0], next);
        if (corners) merge(corner_bins[0], images[i].endpoint);
        continue;
      }
      for (size_t j = 0; j + 1 < cut_count; ++j) {
        Box clipped = next;
        clipped[3] = {std::max(next[3].lo, cuts[j]),
                      std::min(next[3].hi, cuts[j + 1])};
        if (clipped[3].lo <= clipped[3].hi) {
          merge(bins[j + 1], clipped);
          if (corners) merge(corner_bins[j + 1], images[i].endpoint);
        }
      }
    }
    states.clear();
    if (corners) corners->states.clear();
    for (size_t i = 0; i < cut_count; ++i)
      if (bins[i]) {
        states.push_back(*bins[i]);
        if (corners) corners->states.push_back(*corner_bins[i]);
      }
    if (swept) {
      const auto all = joined(states);
      for (size_t i = 0; i < 8; ++i) (*swept)[i] = hull((*swept)[i], all[i]);
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
