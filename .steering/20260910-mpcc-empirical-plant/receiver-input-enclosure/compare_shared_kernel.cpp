#define main preserved_native_driver_main
#include "../native_vehicle_model.cpp"
#undef main
#include "multi_purpose_mpc_ros/mpcc_vehicle_model_kernel.hpp"

#include <cstring>
#include <random>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model
{
std::optional<Transition> baseline_advance(
  State, const Input &, const Parameters &, double) noexcept;
std::optional<BodyDerivative> baseline_derivative(
  const State &, double, const Parameters &, double) noexcept;
std::uint64_t baseline_fingerprint(const Parameters &) noexcept;
}

int main()
{
  const auto original = read_parameters();
  std::mt19937_64 random(864210);
  std::uniform_real_distribution<double> unit(0, 1);
  std::size_t comparisons = 0, scalars = 0, rejects = 0, rest = 0;
  const auto equal = [&](const double a, const double b) {
      require(std::memcmp(&a, &b, sizeof(a)) == 0, "scalar bits differ from frozen baseline");
      ++scalars;
    };
  const auto compare = [&](const model::State & state, const model::Input & input,
      const model::Parameters & p, const double duration) {
      ++comparisons;
      require(model::fingerprint(p) == model::baseline_fingerprint(p), "profile identity differs");
      const auto old_rate = model::baseline_derivative(state, input.wire_acceleration_mps2, p, duration);
      const auto new_rate = model::derivative(state, input.wire_acceleration_mps2, p, duration);
      require(old_rate.has_value() == new_rate.has_value(), "derivative rejection differs");
      if (old_rate) {
        equal(old_rate->x_mps, new_rate->x_mps); equal(old_rate->y_mps, new_rate->y_mps);
        equal(old_rate->yaw_radps, new_rate->yaw_radps);
        equal(old_rate->forward_acceleration_mps2, new_rate->forward_acceleration_mps2);
        equal(old_rate->lateral_acceleration_mps2, new_rate->lateral_acceleration_mps2);
        equal(old_rate->yaw_acceleration_radps2, new_rate->yaw_acceleration_radps2);
      }
      const auto before = model::baseline_advance(state, input, p, duration);
      const auto after = model::advance(state, input, p, duration);
      require(before.has_value() == after.has_value(), "transition rejection differs");
      if (!before) {++rejects; return;}
      const auto a = model::kernel::values(before->state), b = model::kernel::values(after->state);
      for (std::size_t j = 0; j < a.size(); ++j) {equal(a[j], b[j]);}
      require(before->substeps == after->substeps, "substep count differs");
      require(before->nominal_rest_applied == after->nominal_rest_applied, "rest flag differs");
      rest += before->nominal_rest_applied;
    };

  for (std::size_t i = 0; i < 6000; ++i) {
    auto p = original;
    if (i % 2) {
      p.mass_kg *= .5 + unit(random); p.yaw_inertia_kgm2 *= .5 + unit(random);
      p.com_forward_m += unit(random) - .5; p.com_left_m = unit(random) - .5;
      p.rolling_mps2 *= .5 + unit(random); p.drag_per_sec *= .5 + unit(random);
      p.angular_drag_per_sec *= .5 + unit(random); p.nominal_settled_contact = i % 3;
      p.tire_grip *= .5 + .5 * unit(random); p.tire_lag_sec *= .5 + unit(random);
      for (auto & wheel : p.wheels) {
        wheel.com_forward_m += unit(random) - .5; wheel.com_left_m += unit(random) - .5;
        wheel.traction_fraction = .25 * unit(random);
        wheel.cornering_per_sec *= .5 + unit(random); wheel.steerable = unit(random) > .5;
      }
    }
    const model::State state{100 * unit(random), 100 * unit(random), 6 * unit(random) - 3,
      14 * unit(random) - 2, unit(random) - .5, 2 * unit(random) - 1,
      1.4 * unit(random) - .7, 1.4 * unit(random) - .7};
    const model::Input input{14 * unit(random) - 10, 4 * unit(random) - 2};
    const double stamp = i * .005;
    for (const double duration : {0., .001, (stamp + .005) - stamp, .025, .13, .5}) {
      compare(state, input, p, duration);
    }
  }
  for (const double u : {-1., -.021, -.020, -.019, -0., 0., .019, .020, .021, 8.}) {
    for (const double wire : {-10., -8., -3., -0., 0., 1e-12, 1.37, 2.}) {
      for (const double steer : {-.6, -.001, -0., 0., .001, .6}) {
        for (const double duration : {.005, .025, .25, 2.}) {
          compare({0, 0, .2, u, .03, .1, steer, steer}, {wire, .1}, original, duration);
        }
      }
    }
  }
  for (const double invalid : std::array<double, 3>{-1., NAN, INFINITY}) {
    compare({}, {1, 0}, original, invalid);
  }
  for (const double bad : {NAN, INFINITY, -INFINITY}) {
    compare({}, {bad, 0}, original, .025);
    compare({}, {0, bad}, original, .025);
    for (std::size_t j = 0; j < 8; ++j) {
      auto values = model::kernel::values(model::State{}); values[j] = bad;
      compare(model::kernel::state(values), {1, 0}, original, .025);
    }
  }
  compare({}, {1, 0}, {}, .025);
  auto invalid = original; invalid.wheels[0].traction_fraction = .3;
  compare({}, {1, 0}, invalid, .025);
  invalid = original; invalid.steering_wire_gain = std::numeric_limits<double>::max();
  compare({0, 0, 0, 2, 0, 0, 2, .1}, {1, 0}, invalid, .025);
  std::cout << "cases=" << comparisons << " bitwise_equal_scalars=" << scalars
            << " rejected=" << rejects << " rest_cases=" << rest << '\n';
  require(rejects > 0 && rest > 0, "comparison missed rejected/rest cases");
}
