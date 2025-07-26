// reference_path.cpp
#include "reference_path.hpp"
#include "interp_utils.hpp"
#include <algorithm>
#include <numeric>
#include <iostream>

#include <unsupported/Eigen/Splines>

ReferencePath::ReferencePath(const std::vector<double>& wp_x, const std::vector<double>& wp_y,
                               double path_resolution, double smoothing_distance, double max_width,
                               bool circular)
    : resolution(path_resolution),
      smoothing_distance(smoothing_distance),
      max_width(max_width),
      circular(circular) {

    waypoints = construct_path(wp_x, wp_y);
    segment_lengths = compute_segment_lengths();
}

std::vector<std::shared_ptr<Waypoint>> ReferencePath::construct_path(const std::vector<double>& wp_x,
                                                                      const std::vector<double>& wp_y) {
    std::vector<double> distance(wp_x.size(), 0.0);
    for (size_t i = 1; i < wp_x.size(); ++i) {
        double dx = wp_x[i] - wp_x[i - 1];
        double dy = wp_y[i] - wp_y[i - 1];
        distance[i] = distance[i - 1] + std::sqrt(dx * dx + dy * dy);
    }

    double total_dist = distance.back();
    int n_interp = static_cast<int>(total_dist / resolution);
    std::vector<double> s_interp(n_interp);
    for (int i = 0; i < n_interp; ++i) {
        s_interp[i] = distance.front() + i * resolution;
    }

    Eigen::VectorXd dist_eig = Eigen::Map<const Eigen::VectorXd>(distance.data(), distance.size());
    Eigen::VectorXd x_eig = Eigen::Map<const Eigen::VectorXd>(wp_x.data(), wp_x.size());
    Eigen::VectorXd y_eig = Eigen::Map<const Eigen::VectorXd>(wp_y.data(), wp_y.size());

    Eigen::Spline<double, 1> spline_x = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(x_eig.transpose(), 1, dist_eig);
    Eigen::Spline<double, 1> spline_y = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y_eig.transpose(), 1, dist_eig);

    std::vector<std::shared_ptr<Waypoint>> interp_points;
    for (int i = 0; i < n_interp; ++i) {
        double s = s_interp[i];
        double x = spline_x(s)(0);
        double y = spline_y(s)(0);
        double yaw;
        if (i == n_interp - 1) {
            double x_prev = spline_x(s_interp[i - 1])(0);
            double y_prev = spline_y(s_interp[i - 1])(0);
            yaw = std::atan2(y - y_prev, x - x_prev);
        } else {
            double x_next = spline_x(s_interp[i + 1])(0);
            double y_next = spline_y(s_interp[i + 1])(0);
            yaw = std::atan2(y_next - y, x_next - x);
        }
        interp_points.push_back(std::make_shared<Waypoint>(x, y, yaw));
    }
    return interp_points;
}

std::vector<double> ReferencePath::compute_segment_lengths() {
    std::vector<double> lengths = {0.0};
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i]->x - waypoints[i - 1]->x;
        double dy = waypoints[i]->y - waypoints[i - 1]->y;
        lengths.push_back(std::sqrt(dx * dx + dy * dy));
    }
    return lengths;
}

Waypoint ReferencePath::get_waypoint(int idx) const {
    int idx_clamped = std::max(0, std::min(static_cast<int>(waypoints.size()) - 1, idx));
    return *waypoints[idx_clamped];
}

double ReferencePath::get_speed(double s) const {
    if (v_profile.empty()) return 0.0;
    std::vector<double> idx(v_profile.size());
    std::iota(idx.begin(), idx.end(), 0);
    return linear_interp(s, idx, v_profile);
}

double ReferencePath::get_curvature(double s) const {
    if (kappa_profile.empty()) return 0.0;
    std::vector<double> idx(kappa_profile.size());
    std::iota(idx.begin(), idx.end(), 0);
    return linear_interp(s, idx, kappa_profile);
}


void ReferencePath::set_speed_profile(const std::vector<double>& v_profile_in) {
    v_profile = v_profile_in;
    if (v_profile.size() == waypoints.size()) {
        for (size_t i = 0; i < waypoints.size(); ++i) {
            waypoints[i]->v_ref = v_profile[i];
        }
    }
}

void ReferencePath::update_path_constraints(int waypoint_idx, int horizon,
                                            double upper_bound, double lower_bound,
                                            std::vector<double>& ub,
                                            std::vector<double>& lb,
                                            std::vector<double>& width) {
    int H = std::max(1, horizon);
    if (waypoint_idx >= static_cast<int>(waypoints.size())) {
        waypoint_idx = static_cast<int>(waypoints.size()) - 1;
    }
    ub.assign(H, upper_bound);
    lb.assign(H, -lower_bound);
    width.assign(H, upper_bound + lower_bound);
}

int ReferencePath::get_closest_index(double s) const {
    double min_diff = std::numeric_limits<double>::max();
    int closest_index = 0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        double diff = std::abs(waypoints[i]->s - s);
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = static_cast<int>(i);
        }
    }
    return closest_index;
}

std::vector<Waypoint> ReferencePath::extract_subpath(double s, int N) const {
    std::vector<Waypoint> result;

    int center_idx = get_closest_index(s);
    int start_idx = std::max(0, center_idx - N);
    int end_idx = std::min(static_cast<int>(waypoints.size()) - 1, center_idx + N);

    for (int i = start_idx; i <= end_idx; ++i) {
        result.push_back(*waypoints[i]);  // shared_ptr → 実体コピー
    }

    return result;
}

double ReferencePath::get_closest_s(double x, double y) const {
    double min_dist = 1e9;
    double closest_s = 0.0;
    for (const auto& wp : waypoints) {
        double dx = wp->x - x;
        double dy = wp->y - y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_s = wp->s;
        }
    }
    return closest_s;
}

// reference_path.cpp
Waypoint ReferencePath::get_waypoint_from_s(double s) const {
    // 境界チェック（最後尾より後ろ）
    if (s >= waypoints.back()->s) return *waypoints.back();
    if (s <= waypoints.front()->s) return *waypoints.front();

    // s に一番近い区間を見つけて線形補間
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const auto& wp0 = waypoints[i];
        const auto& wp1 = waypoints[i + 1];
        if (wp0->s <= s && s < wp1->s) {
            double ratio = (s - wp0->s) / (wp1->s - wp0->s);
            Waypoint interp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);  // ✅ 初期値は上書き前提でOK
            interp.s = s;
            interp.x = wp0->x + ratio * (wp1->x - wp0->x);
            interp.y = wp0->y + ratio * (wp1->y - wp0->y);
            interp.psi = wp0->psi + ratio * (wp1->psi - wp0->psi);
            interp.kappa = wp0->kappa + ratio * (wp1->kappa - wp0->kappa);
            interp.v_ref = wp0->v_ref + ratio * (wp1->v_ref - wp0->v_ref);
            interp.delta_ref = wp0->delta_ref + ratio * (wp1->delta_ref - wp0->delta_ref);
            return interp;
        }
    }

    // fallback
    return *waypoints.back();
}
