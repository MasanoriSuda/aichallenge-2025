#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace model = multi_purpose_mpc_ros::mpcc_vehicle_model;

void require(const bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

model::Parameters read_parameters()
{
  model::Parameters p;
  std::cin >> p.profile_id >> p.mass_kg >> p.yaw_inertia_kgm2 >> p.com_forward_m >>
    p.com_left_m >> p.rolling_mps2 >> p.drag_per_sec >> p.angular_drag_per_sec >>
    p.maximum_wire_acceleration_mps2 >> p.maximum_wire_deceleration_mps2 >>
    p.sleep_speed_mps >> p.minimum_force_interval_sec >> p.tire_lag_sec >>
    p.tire_slew_radps >> p.tire_grip >> p.steering_wire_gain >>
    p.maximum_wire_steering_rad >> p.maximum_step_sec >> p.nominal_settled_contact;
  for (auto & w : p.wheels) {
    std::cin >> w.com_forward_m >> w.com_left_m >> w.traction_fraction >>
      w.cornering_per_sec >> w.steerable;
  }
  require(std::cin.good() && model::valid(p), "invalid parameter fixture");
  return p;
}

double difference(const model::State & a, const model::State & b)
{
  return std::abs(a.x_m-b.x_m)+std::abs(a.y_m-b.y_m)+std::abs(a.yaw_rad-b.yaw_rad)+
         std::abs(a.forward_velocity_mps-b.forward_velocity_mps)+
         std::abs(a.lateral_velocity_mps-b.lateral_velocity_mps)+
         std::abs(a.yaw_rate_radps-b.yaw_rate_radps)+
         std::abs(a.desired_steering_rad-b.desired_steering_rad)+
         std::abs(a.tire_steering_rad-b.tire_steering_rad);
}

void self_test(const model::Parameters & p)
{
  model::State moving;
  moving.forward_velocity_mps = 2;
  const auto accelerated = model::advance(moving, {1, 0}, p, .2);
  require(accelerated && accelerated->state.forward_velocity_mps > 2.05 &&
    accelerated->state.forward_velocity_mps < 2.1, "wire/net force counterexample lost");
  const auto unforced = model::advance({}, {0, 0}, p, 2);
  const auto brake_rest = model::advance({}, {-3, 0}, p, 2);
  require(unforced && brake_rest && difference(unforced->state, {}) == 0 &&
    difference(brake_rest->state, {}) == 0, "unforced/braked rest moves");
  const auto launched = model::advance(brake_rest->state, {1, 0}, p, .2);
  require(launched && launched->state.forward_velocity_mps > .05 &&
    launched->state.x_m > 0 && !launched->nominal_rest_applied, "rest prevents launch");
  const auto stopped = model::advance(moving, {-3, 0}, p, 2);
  require(stopped && stopped->nominal_rest_applied &&
    stopped->state.forward_velocity_mps == 0 && stopped->state.x_m > 0,
    "nominal continuous braking does not reach rest");
  auto no_contact_assumption = p;
  no_contact_assumption.nominal_settled_contact = false;
  model::State negative;
  negative.forward_velocity_mps = -.001;
  const auto restorative = model::derivative(negative, -3, no_contact_assumption, .005);
  require(restorative && restorative->forward_acceleration_mps2 > 0,
    "Drive reverse-drift restorative law missing");
  model::State left{0, 0, .1, 5, .03, .2, .2, .1};
  auto right = left;
  right.yaw_rad *= -1; right.lateral_velocity_mps *= -1; right.yaw_rate_radps *= -1;
  right.desired_steering_rad *= -1; right.tire_steering_rad *= -1;
  const auto l = model::advance(left, {1, .1}, p, .5);
  const auto r = model::advance(right, {1, -.1}, p, .5);
  require(l && r && std::abs(l->state.x_m-r->state.x_m) < 1e-12 &&
    std::abs(l->state.y_m+r->state.y_m) < 1e-12 &&
    std::abs(l->state.yaw_rad+r->state.yaw_rad) < 1e-12, "turn reflection broken");
  const auto first = model::advance(left, {1, .1}, p, .025);
  const auto second = model::advance(first->state, {1, .1}, p, .025);
  const auto whole = model::advance(left, {1, .1}, p, .05);
  require(second && whole && difference(second->state, whole->state) < 1e-12,
    "publisher interval composition changes model");
  model::State com_reference{0, 0, 0, 3, p.com_forward_m, 1, 0, 0};
  const auto pose_rate = model::derivative(com_reference, 0, p, .005);
  require(pose_rate && std::abs(pose_rate->y_mps) < 1e-12 && pose_rate->x_mps == 3,
    "COM/base_link reference velocity mixed");
  for (const double duration : {.001, .009, .01, .123, 1.0}) {
    const auto varied = model::advance(left, {1, .1}, p, duration);
    require(varied && duration/varied->substeps <= p.maximum_step_sec+1e-12,
      "variable-duration integration exceeds declared step");
  }
  auto invalid = p;
  invalid.wheels[0].traction_fraction = .3;
  require(!model::advance(left, {1, 0}, invalid, .1), "invalid force profile accepted");
  require(!model::advance(left, {1, 0}, p, -1), "negative duration accepted");
  require(!model::advance(left, {std::numeric_limits<double>::quiet_NaN(), 0}, p, .1),
    "nonfinite wire accepted");
  left.yaw_rad = std::numeric_limits<double>::infinity();
  require(!model::advance(left, {1, 0}, p, .1), "nonfinite state accepted");
  std::cout << "Passed wire/net, rest/brake/launch, restorative Drive, mirror turns, "
    "COM/base_link, publisher composition, variable duration and invalid requests. "
    "wire1_v0=2_after0.2s=" << accelerated->state.forward_velocity_mps <<
    " nominal_stop_distance=" << stopped->state.x_m << '\n';
}

int main(int argc, char ** argv)
{
  std::cout << std::setprecision(17);
  const auto parameters = read_parameters();
  if (argc == 2 && std::string(argv[1]) == "self-test") {
    self_test(parameters);
    return 0;
  }
  // Each sequence has one public initial state and prescribed future input
  // samples. No subsequent body/contact observations enter this executable.
  int id{}, count{};
  while (std::cin >> id >> count) {
    require(count > 0 && count <= 2000, "invalid prescribed sequence length");
    model::State state;
    std::cin >> state.x_m >> state.y_m >> state.yaw_rad >> state.forward_velocity_mps >>
      state.lateral_velocity_mps >> state.yaw_rate_radps >>
      state.desired_steering_rad >> state.tire_steering_rad;
    for (int k = 0; k < count; ++k) {
      double dt{}, wire{}, desired{};
      std::cin >> dt >> wire >> desired;
      require(std::cin.good(), "incomplete prescribed sequence");
      state.desired_steering_rad = desired;
      const auto next = model::advance(state, {wire, 0}, parameters, dt);
      require(next.has_value(), "native prescribed transition rejected");
      state = next->state;
      if (k+1 == 20 || k+1 == 50 || k+1 == 100 || k+1 == 200 || k+1 == 400 || k+1 == count) {
        std::cout << id << ' ' << k+1 << ' ' << state.x_m << ' ' << state.y_m << ' ' <<
          state.yaw_rad << ' ' << state.forward_velocity_mps << ' ' <<
          state.lateral_velocity_mps << ' ' << state.yaw_rate_radps << ' ' <<
          state.desired_steering_rad << ' ' << state.tire_steering_rad << '\n';
      }
    }
  }
  require(std::cin.eof(), "malformed request stream");
  return 0;
}
