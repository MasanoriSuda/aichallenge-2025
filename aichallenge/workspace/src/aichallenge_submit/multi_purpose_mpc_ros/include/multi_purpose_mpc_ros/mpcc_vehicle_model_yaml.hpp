#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_YAML_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_YAML_HPP_

#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"

#include <yaml-cpp/yaml.h>

#include <optional>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model
{

inline YAML::Node encode_parameters(const Parameters & p)
{
  YAML::Node node;
  node["profile_id"] = p.profile_id;
  node["mass_kg"] = p.mass_kg;
  node["yaw_inertia_kgm2"] = p.yaw_inertia_kgm2;
  node["com_forward_m"] = p.com_forward_m;
  node["com_left_m"] = p.com_left_m;
  node["rolling_mps2"] = p.rolling_mps2;
  node["drag_per_sec"] = p.drag_per_sec;
  node["angular_drag_per_sec"] = p.angular_drag_per_sec;
  node["maximum_wire_acceleration_mps2"] = p.maximum_wire_acceleration_mps2;
  node["maximum_wire_deceleration_mps2"] = p.maximum_wire_deceleration_mps2;
  node["sleep_speed_mps"] = p.sleep_speed_mps;
  node["minimum_force_interval_sec"] = p.minimum_force_interval_sec;
  node["tire_lag_sec"] = p.tire_lag_sec;
  node["tire_slew_radps"] = p.tire_slew_radps;
  node["tire_grip"] = p.tire_grip;
  node["steering_wire_gain"] = p.steering_wire_gain;
  node["maximum_wire_steering_rad"] = p.maximum_wire_steering_rad;
  node["maximum_step_sec"] = p.maximum_step_sec;
  node["nominal_settled_contact"] = p.nominal_settled_contact;
  for (const auto & w : p.wheels) {
    YAML::Node wheel;
    wheel["com_forward_m"] = w.com_forward_m;
    wheel["com_left_m"] = w.com_left_m;
    wheel["traction_fraction"] = w.traction_fraction;
    wheel["cornering_per_sec"] = w.cornering_per_sec;
    wheel["steerable"] = w.steerable;
    node["wheels"].push_back(wheel);
  }
  return node;
}

inline std::optional<Parameters> decode_parameters(const YAML::Node & node)
{
  try {
    if (!node.IsMap() || !node["wheels"].IsSequence() || node["wheels"].size() != 4) {
      return std::nullopt;
    }
    Parameters p;
    p.profile_id = node["profile_id"].as<std::string>();
    p.mass_kg = node["mass_kg"].as<double>();
    p.yaw_inertia_kgm2 = node["yaw_inertia_kgm2"].as<double>();
    p.com_forward_m = node["com_forward_m"].as<double>();
    p.com_left_m = node["com_left_m"].as<double>();
    p.rolling_mps2 = node["rolling_mps2"].as<double>();
    p.drag_per_sec = node["drag_per_sec"].as<double>();
    p.angular_drag_per_sec = node["angular_drag_per_sec"].as<double>();
    p.maximum_wire_acceleration_mps2 = node["maximum_wire_acceleration_mps2"].as<double>();
    p.maximum_wire_deceleration_mps2 = node["maximum_wire_deceleration_mps2"].as<double>();
    p.sleep_speed_mps = node["sleep_speed_mps"].as<double>();
    p.minimum_force_interval_sec = node["minimum_force_interval_sec"].as<double>();
    p.tire_lag_sec = node["tire_lag_sec"].as<double>();
    p.tire_slew_radps = node["tire_slew_radps"].as<double>();
    p.tire_grip = node["tire_grip"].as<double>();
    p.steering_wire_gain = node["steering_wire_gain"].as<double>();
    p.maximum_wire_steering_rad = node["maximum_wire_steering_rad"].as<double>();
    p.maximum_step_sec = node["maximum_step_sec"].as<double>();
    p.nominal_settled_contact = node["nominal_settled_contact"].as<bool>();
    for (std::size_t i = 0; i < p.wheels.size(); ++i) {
      const auto wheel = node["wheels"][i];
      p.wheels[i] = Wheel{
        wheel["com_forward_m"].as<double>(), wheel["com_left_m"].as<double>(),
        wheel["traction_fraction"].as<double>(), wheel["cornering_per_sec"].as<double>(),
        wheel["steerable"].as<bool>()};
    }
    return valid(p) ? std::optional<Parameters>{p} : std::nullopt;
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
}

}  // namespace multi_purpose_mpc_ros::mpcc_vehicle_model

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_VEHICLE_MODEL_YAML_HPP_
