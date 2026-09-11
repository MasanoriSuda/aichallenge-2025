#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>

namespace multi_purpose_mpc_ros::mpcc_wire_command {
/// The raw ROS message first serializes physical steering to float32, then
/// calibration is applied, then the final ROS field serializes to float32.
/// All prediction/program producers and the publisher use this exact order.
inline double steering(double physical_rad, double gain) noexcept {
  if (!std::isfinite(physical_rad) || !std::isfinite(gain) || gain <= 0)
    return std::numeric_limits<double>::quiet_NaN();
  const float raw = static_cast<float>(physical_rad);
  const float final = static_cast<float>(static_cast<double>(raw) * gain);
  return std::isfinite(final) ? static_cast<double>(final)
                              : std::numeric_limits<double>::quiet_NaN();
}

/// Select a representable command before forecasting/certifying it. The
/// predicate is the actual wire-difference predicate used at publication;
/// double-valued endpoints alone do not survive the two float32 roundings.
inline std::optional<double> reachable_steering(
    double requested_rad, double previous_wire_rad, double gain,
    double maximum_abs_rad, double maximum_step_rad) noexcept {
  static_assert(sizeof(float) == sizeof(std::uint32_t) &&
    std::numeric_limits<float>::is_iec559 && std::numeric_limits<float>::digits == 24);
  const double largest = std::numeric_limits<float>::max();
  if (!std::isfinite(requested_rad) || !std::isfinite(previous_wire_rad) ||
      std::abs(previous_wire_rad) > largest ||
      static_cast<double>(static_cast<float>(previous_wire_rad)) != previous_wire_rad ||
      !std::isfinite(gain) || gain <= 0 || !std::isfinite(maximum_abs_rad) ||
      maximum_abs_rad <= 0 || maximum_abs_rad > largest ||
      !std::isfinite(maximum_abs_rad * gain) || maximum_abs_rad * gain > largest ||
      !std::isfinite(maximum_step_rad) || maximum_step_rad < 0)
    return std::nullopt;
  // Sign-folded IEEE keys order every finite raw float, including both zeros.
  const auto key = [](float value) {
    std::uint32_t bits{}; std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x80000000U) ? ~bits : (bits ^ 0x80000000U);
  };
  const auto raw = [](std::uint32_t ordered) {
    const std::uint32_t bits = (ordered & 0x80000000U) ?
      (ordered ^ 0x80000000U) : ~ordered;
    float value{}; std::memcpy(&value, &bits, sizeof(value)); return value;
  };
  const auto classify = [&](float value) {
    const double wire = steering(value, gain);
    const double physical = wire / gain;
    const double step = std::abs(wire - previous_wire_rad) / gain;
    if (physical < -maximum_abs_rad ||
        (wire < previous_wire_rad && step > maximum_step_rad)) return -1;
    if (physical > maximum_abs_rad ||
        (wire > previous_wire_rad && step > maximum_step_rad)) return 1;
    return 0;
  };
  float lower = static_cast<float>(-maximum_abs_rad);
  float upper = static_cast<float>(maximum_abs_rad);
  if (lower < -maximum_abs_rad) lower = std::nextafter(lower, 0.0F);
  if (upper > maximum_abs_rad) upper = std::nextafter(upper, 0.0F);
  if (std::abs(requested_rad) <= maximum_abs_rad) {
    const float original = static_cast<float>(requested_rad);
    if (original >= lower && original <= upper && classify(original) == 0)
      return requested_rad; // Existing valid certificate inputs stay identical.
  }
  // Find the exact nonempty interval of raw floats satisfying both constraints.
  std::uint32_t lo = key(lower), hi = key(upper);
  while (lo < hi) {
    const auto mid = lo + (hi - lo) / 2;
    if (classify(raw(mid)) < 0) lo = mid + 1; else hi = mid;
  }
  const auto first = lo;
  lo = key(lower); hi = key(upper);
  while (lo < hi) {
    const auto mid = lo + (hi - lo) / 2 + (hi - lo) % 2;
    if (classify(raw(mid)) > 0) hi = mid - 1; else lo = mid;
  }
  if (first > lo || classify(raw(first)) != 0 || classify(raw(lo)) != 0)
    return std::nullopt;
  const float desired = static_cast<float>(std::clamp(
    requested_rad, static_cast<double>(lower), static_cast<double>(upper)));
  return static_cast<double>(raw(std::clamp(key(desired), first, lo)));
}
} // namespace multi_purpose_mpc_ros::mpcc_wire_command
