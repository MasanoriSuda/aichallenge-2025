#ifndef MPCC_DIAGNOSTIC_PLANAR_BODY_MODEL_HPP
#define MPCC_DIAGNOSTIC_PLANAR_BODY_MODEL_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace mpcc_planar_body_model
{
// Body pose is the vehicle reference origin; velocity is at the COM, in body
// forward/left axes. Yaw and yaw rate are positive counterclockwise.
struct State
{
  double x{}, y{}, yaw{}, forward_velocity{}, lateral_velocity{}, yaw_rate{};
};

struct Wheel
{
  double com_forward_m{}, com_left_m{};
  double traction_fraction{};       // probability/contact weight divided by four
  double cornering_per_sec{};       // skid * supported mass / body mass / fixed dt
  bool steerable{};
};

struct Parameters
{
  double mass_kg{}, yaw_inertia_kgm2{}, com_forward_m{}, com_left_m{};
  double rolling_mps2{}, drag_per_sec{}, angular_drag_per_sec{};
  std::array<Wheel, 4> wheels;
};

struct Input
{
  double acceleration_wire_mps2{}, tire_steering_rad{};
};

inline bool finite(const State & s)
{
  return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.yaw) &&
         std::isfinite(s.forward_velocity) && std::isfinite(s.lateral_velocity) &&
         std::isfinite(s.yaw_rate);
}

inline bool valid(const Parameters & p)
{
  if (!(std::isfinite(p.mass_kg) && p.mass_kg > 0 &&
    std::isfinite(p.yaw_inertia_kgm2) && p.yaw_inertia_kgm2 > 0 &&
    std::isfinite(p.com_forward_m) && std::isfinite(p.com_left_m) &&
    std::isfinite(p.rolling_mps2) && p.rolling_mps2 >= 0 &&
    std::isfinite(p.drag_per_sec) && p.drag_per_sec >= 0 &&
    std::isfinite(p.angular_drag_per_sec) && p.angular_drag_per_sec >= 0))
  {
    return false;
  }
  for (const auto & w : p.wheels) {
    if (!(std::isfinite(w.com_forward_m) && std::isfinite(w.com_left_m) &&
      std::isfinite(w.traction_fraction) && w.traction_fraction >= 0 &&
      w.traction_fraction <= 0.25 && std::isfinite(w.cornering_per_sec) &&
      w.cornering_per_sec >= 0))
    {
      return false;
    }
  }
  return true;
}

// Moving-mode differential law. The simulator's discrete sleep/gear transition
// is a separate contract and is not approximated by an artificial v clamp here.
inline std::optional<State> derivative(
  const State & s, const Input & input, const Parameters & p)
{
  if (!valid(p) || !finite(s) || !std::isfinite(input.acceleration_wire_mps2) ||
    !std::isfinite(input.tire_steering_rad) ||
    std::abs(input.tire_steering_rad) >= 1.57079632679489661923)
  {
    return std::nullopt;
  }
  const double u = s.forward_velocity;
  const double v = s.lateral_velocity;
  const double r = s.yaw_rate;
  const double rolling = u == 0 ? 0 : -std::copysign(p.rolling_mps2, u);
  double fx{}, fy{}, moment{};
  for (const auto & w : p.wheels) {
    const double angle = w.steerable ? input.tire_steering_rad : 0;
    const double c = std::cos(angle), sn = std::sin(angle);
    const double point_forward = u - r * w.com_left_m;
    const double point_left = v + r * w.com_forward_m;
    const double side_velocity = -sn * point_forward + c * point_left;
    const double lateral = -w.cornering_per_sec * side_velocity;
    const double drive = w.traction_fraction * (input.acceleration_wire_mps2 + rolling);
    const double ax = c * drive - sn * lateral;
    const double ay = sn * drive + c * lateral;
    fx += ax;
    fy += ay;
    moment += w.com_forward_m * ay - w.com_left_m * ax;
  }
  // v_reference = v_COM + omega cross (reference - COM).
  const double reference_forward = u + r * p.com_left_m;
  const double reference_left = v - r * p.com_forward_m;
  const double c = std::cos(s.yaw), sn = std::sin(s.yaw);
  State result{
    c * reference_forward - sn * reference_left,
    sn * reference_forward + c * reference_left,
    r, fx - p.drag_per_sec * u + r * v,
    fy - p.drag_per_sec * v - r * u,
    moment * p.mass_kg / p.yaw_inertia_kgm2 - p.angular_drag_per_sec * r};
  return finite(result) ? std::optional<State>{result} : std::nullopt;
}

inline State add(const State & a, const State & b, double scale)
{
  return {a.x + scale * b.x, a.y + scale * b.y, a.yaw + scale * b.yaw,
    a.forward_velocity + scale * b.forward_velocity,
    a.lateral_velocity + scale * b.lateral_velocity,
    a.yaw_rate + scale * b.yaw_rate};
}

inline std::optional<State> transition(
  State state, const Input & input, const Parameters & parameters, double duration)
{
  if (!std::isfinite(duration) || duration < 0 || !valid(parameters) || !finite(state)) {
    return std::nullopt;
  }
  const std::size_t count = static_cast<std::size_t>(std::ceil(duration / 0.005));
  if (count == 0) {
    return state;
  }
  const double dt = duration / static_cast<double>(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto a = derivative(state, input, parameters);
    if (!a) {return std::nullopt;}
    const auto b = derivative(add(state, *a, dt / 2), input, parameters);
    if (!b) {return std::nullopt;}
    const auto c = derivative(add(state, *b, dt / 2), input, parameters);
    if (!c) {return std::nullopt;}
    const auto d = derivative(add(state, *c, dt), input, parameters);
    if (!d) {return std::nullopt;}
    state = add(add(add(add(state, *a, dt / 6), *b, dt / 3), *c, dt / 3), *d, dt / 6);
  }
  return finite(state) ? std::optional<State>{state} : std::nullopt;
}
}  // namespace mpcc_planar_body_model
#endif
