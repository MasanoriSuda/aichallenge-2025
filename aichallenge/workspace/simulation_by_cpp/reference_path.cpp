#include "reference_path.hpp"
#include <cmath>
#include <iostream>

void ReferencePath::construct_path(const std::vector<double>& xs,
                                   const std::vector<double>& ys) {
    // 簡易な線形補間とs計算（必要に応じてスプラインに置き換え）
    waypoints.clear();

    double s = 0.0;
    for (size_t i = 0; i < xs.size(); ++i) {
        auto wp = std::make_shared<Waypoint>();
        wp->x = xs[i];
        wp->y = ys[i];
        if (i > 0) {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            s += std::sqrt(dx * dx + dy * dy);
        }
        wp->s = s;
        waypoints.push_back(wp);
    }

    // yaw, kappa を補間（簡易：前後差分）
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i]->x - waypoints[i - 1]->x;
        double dy = waypoints[i]->y - waypoints[i - 1]->y;
        waypoints[i - 1]->yaw = std::atan2(dy, dx);
    }
    waypoints.back()->yaw = waypoints[waypoints.size() - 2]->yaw;  // 最後にも補間

    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        double dx1 = waypoints[i]->x - waypoints[i - 1]->x;
        double dy1 = waypoints[i]->y - waypoints[i - 1]->y;
        double dx2 = waypoints[i + 1]->x - waypoints[i]->x;
        double dy2 = waypoints[i + 1]->y - waypoints[i]->y;

        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        double angle = std::atan2(dy2, dx2) - std::atan2(dy1, dx1);
        angle = std::atan2(std::sin(angle), std::cos(angle));  // wrap to [-pi, pi]

        waypoints[i]->kappa = angle / ((len1 + len2) * 0.5);
    }

    waypoints.front()->kappa = 0.0;
    waypoints.back()->kappa = 0.0;

    const double L = 1.087;
    for (auto& wp : waypoints) {
        wp->delta_ref_ = std::atan(L * wp->kappa);  // ← リファレンスステア角の追加
        std::cout << wp->delta_ref_ << std::endl;
    }
}

void ReferencePath::set_speed_profile(double default_speed) {
    for (auto& wp : waypoints) {
        wp->v_ref = default_speed;  // 固定値
    }
}

double ReferencePath::get_speed(double s) const {
    // s に最も近い waypoint を探す
    if (waypoints.empty()) return 0.0;

    double min_dist = std::numeric_limits<double>::max();
    double v = 0.0;
    for (const auto& wp : waypoints) {
        double dist = std::abs(wp->s - s);
        if (dist < min_dist) {
            min_dist = dist;
            v = wp->v_ref;
        }
    }
    return v;
}

std::shared_ptr<Waypoint> ReferencePath::get_waypoint(double s) const {
    if (waypoints.empty()) return nullptr;

    double min_dist = std::numeric_limits<double>::max();
    std::shared_ptr<Waypoint> nearest_wp = nullptr;

    for (const auto& wp : waypoints) {
        double dist = std::abs(wp->s - s);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_wp = wp;
        }
    }
    return nearest_wp;
}

