#include "reference_path.hpp"   // ← ReferencePath のクラス定義
#include "waypoint.hpp"         // ← Waypoint クラス
#include <vector>               // std::vector
#include <memory>               // std::make_shared
#include <cmath>                // std::sqrt, std::atan2
#include <algorithm>            // std::max, std::min
#include <iostream>

void ReferencePath::construct_path(const std::vector<double>& x_list, const std::vector<double>& y_list) {
    double s = 0.0;
    const double wheelbase = 1.087;  // ← 明示的に定義

    std::cout << "x_list.size() = " << x_list.size() << std::endl;

    for (size_t i = 0; i < x_list.size(); ++i) {
        double x = x_list[i];
        double y = y_list[i];
        double yaw = 0.0;
        double kappa = 0.0;

        // yawの計算
        if (i < x_list.size() - 1) {
            double dx = x_list[i + 1] - x;
            double dy = y_list[i + 1] - y;
            yaw = std::atan2(dy, dx);
        } else if (i > 0) {
            double dx = x - x_list[i - 1];
            double dy = y - y_list[i - 1];
            yaw = std::atan2(dy, dx);
        }

        // 曲率の推定：3点円弧近似
        if (i > 0 && i < x_list.size() - 1) {
            double x0 = x_list[i - 1], y0 = y_list[i - 1];
            double x1 = x_list[i],     y1 = y_list[i];
            double x2 = x_list[i + 1], y2 = y_list[i + 1];

            double a = std::hypot(x1 - x0, y1 - y0);
            double b = std::hypot(x2 - x1, y2 - y1);
            double c = std::hypot(x2 - x0, y2 - y0);

            double s_tri = (a + b + c) / 2.0;
            double area = std::sqrt(std::max(s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c), 0.0));

            if (area > 1e-6) {
                double R = (a * b * c) / (4.0 * area);
                kappa = 1.0 / R;
            } else {
                kappa = 0.0;
            }
        }

        // 距離の更新（Waypoint作成前に！）
        if (i > 0) {
            double dx = x - x_list[i - 1];
            double dy = y - y_list[i - 1];
            s += std::sqrt(dx * dx + dy * dy);
        }

        // Waypoint 生成
        double speed = 0.0;
        double delta_ref = std::atan(wheelbase * kappa);

        Waypoint wp(x, y, yaw, speed, kappa);
        wp.s = s;  // ✅ これが重要！
        wp.t = s;  // tも一応セット（t = sと同義運用なら）
        wp.delta_ref_ = delta_ref;

        std::cout << "[construct] i=" << i << ", s=" << s << ", delta_ref=" << delta_ref << std::endl;

        waypoints.push_back(std::make_shared<Waypoint>(wp));
    }
}


//void ReferencePath::set_speed_profile(double default_speed) {
//    for (auto& wp : waypoints) {
//        wp->v_ref = default_speed;
//    }
//}

//void ReferencePath::set_speed_profile(double default_speed) {
void ReferencePath::set_speed_profile(double default_speed) {
    for (auto& wp : waypoints) {
        double k = std::abs(wp->kappa);  // 曲率（絶対値）
        double v = default_speed / (1.0 + 80.0 * k);  // ← 係数40.0は暫定（調整OK）

        // 速度の下限と上限を制限（安全対策）
        v = std::clamp(v, 5.0, default_speed);  // 最低5km/h、最大default

        wp->v_ref = v;
    }
}

