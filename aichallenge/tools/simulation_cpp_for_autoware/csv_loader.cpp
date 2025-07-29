#include "csv_loader.hpp"  // ← 必須！CSVLoaderのクラス定義を含むヘッダ
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>  // std::sqrt 用（Waypoint::distanceTo を使うなら）

bool CSVLoader::load(const std::string & filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "CSVファイルを開けませんでした: " << filename << std::endl;
        return false;
    }

    std::string line;
    bool is_first = true;
    while (std::getline(file, line)) {
        if (is_first) {
            is_first = false;
            if (line.find_first_not_of("0123456789.-,") != std::string::npos) continue;  // ヘッダー
        }

        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;
        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (...) {
                row.clear();
                break;
            }
        }

        if (row.size() >= 6) {  // x, y, yaw, speed, kappa, s
            waypoints_.emplace_back(row[0], row[1], row[2], row[4], row[3], row[5]);
        }
    }
    return true;
}
