#pragma once
// Diagnostic adapters share the production numerical implementation.
#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"
namespace enclosure {
namespace model=multi_purpose_mpc_ros::mpcc_vehicle_model;
using namespace model::numerical;
}
