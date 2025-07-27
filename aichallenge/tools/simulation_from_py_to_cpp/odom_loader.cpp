// odom_loader.cpp
#include "OdometryInput.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

OdometryInput load_odom_csv(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;

    // Skip header
    std::getline(file, line);

    // Read one line
    std::getline(file, line);
    std::stringstream ss(line);

    std::string val;
    std::vector<double> values;
    while (std::getline(ss, val, ',')) {
        values.push_back(std::stod(val));
    }

    OdometryInput odom;
    odom.x = values[0];
    odom.y = values[1];
    odom.yaw = values[2];
    odom.v = values[3];
    return odom;
}