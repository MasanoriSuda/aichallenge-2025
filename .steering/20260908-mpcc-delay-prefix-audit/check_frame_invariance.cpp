// Offline native counterexample. The reference is an exact arc-length circle;
// Cartesian straight motion is known analytically, without another integrator.
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
namespace r = multi_purpose_mpc_ros::mpcc_rate_resolved;
int main() {
  int failures = 0;
  std::cout << std::setprecision(17) << "[\n";
  bool first = true;
  for (double curvature : {0.0, 0.2, -0.2}) {
    for (double velocity : {0.0, 8.0}) {
      for (double virtual_speed : {0.0, 4.0, 8.0, 12.0}) {
        r::LinearizationRequest q;
        q.reference_lateral_m = velocity == 0.0 ? 0.0 : 0.2;
        q.reference_lag_m = -0.3;
        q.reference_velocity_mps = velocity;
        q.reference_virtual_progress_speed_mps = virtual_speed;
        q.reference_path_curvature_radpm = curvature;
        q.wheelbase_m = 1.087;
        q.yaw_response_gain = 0.75;
        q.yaw_response_time_constant_sec = 0.13;
        q.stage_dt_sec = 0.1;
        auto native = r::evaluate_temporal_frenet_transition(q);
        if (!native) return 2;
        const auto & s = native->next_state;
        const double angle = curvature * s[r::kProgressIndex];
        const double ref_x = curvature == 0.0 ? s[r::kProgressIndex] : std::sin(angle) / curvature;
        const double ref_y = curvature == 0.0 ? 0.0 : (1.0 - std::cos(angle)) / curvature;
        const double x = ref_x + s[r::kLagIndex] * std::cos(angle) - s[r::kLateralIndex] * std::sin(angle);
        const double y = ref_y + s[r::kLagIndex] * std::sin(angle) + s[r::kLateralIndex] * std::cos(angle);
        const double expected_x = q.reference_lag_m + velocity * q.stage_dt_sec;
        const double expected_y = q.reference_lateral_m;
        const double error = std::hypot(x - expected_x, y - expected_y);
        // 0.5 mm is far above roundoff for this exactly straight/stationary
        // physical motion, and far below the observed centimetre-scale defect.
        const bool passed = error < 0.0005;
        failures += !passed;
        if (!first) std::cout << ",\n";
        first = false;
        std::cout << "{\"curvature\":" << curvature << ",\"velocity\":" << velocity
          << ",\"virtual_speed\":" << virtual_speed << ",\"duration_sec\":" << q.stage_dt_sec
          << ",\"x\":" << x << ",\"y\":" << y
          << ",\"expected_x\":" << expected_x << ",\"expected_y\":" << expected_y
          << ",\"error_m\":" << error << ",\"passed\":" << (passed ? "true" : "false") << "}";
      }
    }
  }
  std::cout << "\n]\n";
  return failures ? 1 : 0;
}
