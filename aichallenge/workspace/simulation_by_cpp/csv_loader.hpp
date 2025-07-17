#pragma once

#include <string>
#include <vector>
#include <memory>
#include "waypoint.hpp"

std::vector<std::shared_ptr<Waypoint>> load_csv(const std::string& filename);
