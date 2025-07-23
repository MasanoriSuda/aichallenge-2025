#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <iostream>

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
load_csv(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<double> x_list, y_list, v_list;

    if (!file.is_open()) {
        std::cerr << "Failed to open CSV: " << filename << std::endl;
        return {x_list, y_list, v_list};
    }

    std::string line;
    bool is_first_line = true;
    while (std::getline(file, line)) {
        // 最初の行がヘッダーかどうかをチェック
        if (is_first_line) {
            is_first_line = false;
            if (line.find_first_not_of("0123456789.-,") != std::string::npos) {
                continue;  // ヘッダーならスキップ
            }
        }

        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid value in CSV: '" << value << "' in line: " << line << std::endl;
                row.clear();  // その行は使わない
                break;
            }
        }

        if (row.size() >= 4) {  // x, y, yaw, speed の想定
            x_list.push_back(row[0]);
            y_list.push_back(row[1]);
            v_list.push_back(row[3]);
        }
    }

    return {x_list, y_list, v_list};
}
