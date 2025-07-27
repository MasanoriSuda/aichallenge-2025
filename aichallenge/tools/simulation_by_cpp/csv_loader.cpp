#include "csv_loader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>

#include <cmath>

// クォータニオンからyaw（偏角）を計算
double quaternion_to_yaw(double x, double y, double z, double w) {
    // オイラー角yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    return std::atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z));
}

// Autoware形式CSVの読み込み
std::vector<TrajectoryPoint> load_autoware_csv(const std::string& filename) {
    std::vector<TrajectoryPoint> traj;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return traj;
    }

    std::string line;
    std::getline(file, line);  // ヘッダー読み飛ばし

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string field;
        TrajectoryPoint p;

        std::getline(ss, field, ','); p.x = std::stod(field);
        std::getline(ss, field, ','); p.y = std::stod(field);
        std::getline(ss, field, ','); p.z = std::stod(field);

        double qx, qy, qz, qw;
        std::getline(ss, field, ','); qx = std::stod(field);
        std::getline(ss, field, ','); qy = std::stod(field);
        std::getline(ss, field, ','); qz = std::stod(field);
        std::getline(ss, field, ','); qw = std::stod(field);

        std::getline(ss, field, ','); p.v = std::stod(field);  // speed

        p.yaw = quaternion_to_yaw(qx, qy, qz, qw);
        p.kappa = 0.0;  // 初期化、あとで補完 or 差分計算しても良い

        traj.push_back(p);
    }

    return traj;
}


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
