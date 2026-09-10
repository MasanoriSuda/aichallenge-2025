#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"
#include <iostream>

namespace model = multi_purpose_mpc_ros::mpcc_vehicle_model;
int main()
{
  const model::InputApplicationProfile profile{"test-gap", .125, .25, .1};
  const model::PublishedInputProgram next{.025, {{1.5, -3, .125}}, true};
  const auto missing_at_start = model::applied_input_bounds(
    {{0, 1, 0}, {1.35, 1, 0}}, next, profile, 1.495, 1.505);
  std::cout << "future packet fills preceding uncovered time=" << bool(missing_at_start) << '\n';
  const model::PublishedInputProgram later{.025, {{1.525, -3, .125}}, true};
  const auto interior_gap = model::applied_input_bounds(
    {{0, 1, 0}, {1.377, 1, 0}}, later, profile, 1.5, 1.53);
  std::cout << "separated packets hide an interior coverage gap=" << bool(interior_gap) << '\n';
  const auto contiguous = model::applied_input_bounds(
    {{0, 1, 0}, {1.375, 1, 0}}, next, profile, 1.495, 1.505);
  std::cout << "contiguous causal coverage=" << bool(contiguous) << '\n';
  return !missing_at_start && !interior_gap && contiguous ? 0 : 1;
}
