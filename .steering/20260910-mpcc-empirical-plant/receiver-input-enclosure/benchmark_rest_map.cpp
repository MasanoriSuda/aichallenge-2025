#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_model_yaml.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>

namespace model = multi_purpose_mpc_ros::mpcc_vehicle_model;
namespace num = model::numerical;
num::Box exact_rest(const num::Box & b, const model::Parameters & p, double dt)
{
  auto result = b;
  result[3] = result[4] = result[5] = num::I(0);
  result[7] = num::tire_bounds(b[6], b[7], p, dt);
  return result;
}
int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  const auto loaded = model::decode_parameters(YAML::LoadFile(argv[1]));
  if (!loaded) throw std::runtime_error("invalid model config");
  const auto & p = *loaded;
  std::mt19937_64 engine(20260911);
  std::uniform_real_distribution<double> unit(0, 1);
  std::vector<num::Box> boxes;
  std::size_t checked = 0;
  for (int i = 0; i < 1000; ++i) {
    const model::State state{unit(engine), unit(engine), unit(engine),
      (unit(engine) - .5) * p.sleep_speed_mps, unit(engine), unit(engine),
      (unit(engine) - .5) * .5, (unit(engine) - .5) * .5};
    auto b = num::point(state);
    b[0] = {state.x_m - .2, state.x_m + .2};
    b[1] = {state.y_m - .2, state.y_m + .2};
    b[2] = {state.yaw_rad - .1, state.yaw_rad + .1};
    b[6] = {state.desired_steering_rad - .05, state.desired_steering_rad + .05};
    b[7] = {state.tire_steering_rad - .05, state.tire_steering_rad + .05};
    boxes.push_back(b);
    const auto candidate = exact_rest(b, p, .005);
    for (int arm = 0; arm < 32; ++arm) {
      auto value = num::values(state);
      for (std::size_t j = 0; j < 8; ++j) value[j] =
        arm < 2 ? (arm == 0 ? b[j].lo : b[j].hi) :
        b[j].lo + unit(engine) * (b[j].hi - b[j].lo);
      const auto native = model::advance(model::kernel::state(value), {-3 * unit(engine), 0}, p, .005);
      if (!native) return 2;
      const auto actual = num::values(native->state);
      for (std::size_t j = 0; j < 8; ++j) {
        if (actual[j] < candidate[j].lo || actual[j] > candidate[j].hi) {
          std::cout << "outside: case=" << i << " arm=" << arm << " state=" << j << '\n';
          return 1;
        }
        ++checked;
      }
    }
  }
  volatile double checksum = 0;
  for (const bool candidate : {false, true}) {
    const auto start = std::chrono::steady_clock::now();
    for (int repeat = 0; repeat < 30; ++repeat) for (const auto & b : boxes) {
      const auto result = candidate ? exact_rest(b, p, .005) :
        num::centered_step(b, num::I(-3, 0), p, .005, true);
      checksum += result[7].hi;
    }
    std::cout << (candidate ? "analytic-rest" : "current-centered-rest") << " ms=" <<
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() << '\n';
  }
  std::cout << "checked_native_values=" << checked << " checksum=" << checksum << '\n';
}
