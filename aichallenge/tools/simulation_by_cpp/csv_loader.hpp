#pragma once

#include <string>
#include <vector>
#include <memory>
#include "waypoint.hpp"
#include "trajectory_point.hpp"

std::vector<std::shared_ptr<Waypoint>> load_csv(const std::string& filename);
std::vector<TrajectoryPoint> load_autoware_csv(const std::string& filename);  // ←★名前変更！

