#pragma once

#include "multi_purpose_mpc_ros/mpcc_vehicle_model_yaml.hpp"

#include <filesystem>
#include <stdexcept>

namespace multi_purpose_mpc_ros::test
{
inline mpcc_vehicle_model::Parameters vehicle_model()
{
  const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
    "config/mpcc_plant_awsim_2025.yaml";
  const auto parameters = mpcc_vehicle_model::decode_parameters(YAML::LoadFile(path.string()));
  if (!parameters) throw std::runtime_error("invalid shared test vehicle profile");
  return *parameters;
}
}  // namespace multi_purpose_mpc_ros::test
