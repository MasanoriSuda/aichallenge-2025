#pragma once

#include <tuple>
#include <vector>
#include <string>

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
load_csv(const std::string& filename);
