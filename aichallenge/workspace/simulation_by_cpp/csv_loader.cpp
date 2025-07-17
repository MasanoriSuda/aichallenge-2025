#include "csv_loader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>

std::vector<std::shared_ptr<Waypoint>> load_csv(const std::string& filename) {
    std::vector<std::shared_ptr<Waypoint>> waypoints;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Could not open CSV file: " << filename << std::endl;
        return waypoints;
    }

    std::string line;
    bool is_header = true;

    while (std::getline(file, line)) {
        if (is_header) {
            is_header = false;
            continue;  // ヘッダー行は読み飛ばす
        }

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() < 5) {
            continue;  // x, y, yaw, speed, kappa が必要
        }

        double x = std::stod(tokens[0]);
        double y = std::stod(tokens[1]);
        double yaw = std::stod(tokens[2]);
        double speed = std::stod(tokens[3]);
        double kappa = std::stod(tokens[4]);  // ← 追加

        auto wp = std::make_shared<Waypoint>(x, y, yaw, speed, kappa);
        waypoints.push_back(wp);
    }

    std::cout << "Loaded " << waypoints.size() << " waypoints from CSV." << std::endl;
    return waypoints;
}
