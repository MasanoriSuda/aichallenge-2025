#include "reference_path.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>

ReferencePath::ReferencePath(const std::vector<double>& wp_x,
                             const std::vector<double>& wp_y,
                             double path_resolution,
                             double smoothing_distance,
                             double max_width,
                             bool circular)
    : wp_x_(wp_x), wp_y_(wp_y), resolution_(path_resolution),
      smoothing_distance_(smoothing_distance), max_width_(max_width), circular_(circular) {
    construct_path(wp_x_, wp_y_);
    compute_segment_lengths();
}

void ReferencePath::construct_path(const std::vector<double>& wp_x,
                                   const std::vector<double>& wp_y) {
    // Compute cumulative distance
    std::vector<double> distance(wp_x.size(), 0.0);
    for (size_t i = 1; i < wp_x.size(); ++i) {
        double dx = wp_x[i] - wp_x[i - 1];
        double dy = wp_y[i] - wp_y[i - 1];
        distance[i] = distance[i - 1] + std::sqrt(dx * dx + dy * dy);
    }

    int n_points = static_cast<int>((distance.back() - distance.front()) / resolution_);
    std::vector<double> s_interp(n_points);
    for (int i = 0; i < n_points; ++i) {
        s_interp[i] = distance.front() + i * resolution_;
    }

    std::vector<double> x_interp(n_points);
    std::vector<double> y_interp(n_points);

    for (int i = 0; i < n_points; ++i) {
        x_interp[i] = interpolate(distance, wp_x, s_interp[i]);
        y_interp[i] = interpolate(distance, wp_y, s_interp[i]);
    }

    waypoints_.clear();
    for (int i = 0; i < n_points; ++i) {
        double yaw;
        if (i == n_points - 1) {
            yaw = std::atan2(y_interp[i] - y_interp[i - 1], x_interp[i] - x_interp[i - 1]);
        } else {
            yaw = std::atan2(y_interp[i + 1] - y_interp[i], x_interp[i + 1] - x_interp[i]);
        }
        waypoints_.emplace_back(std::make_shared<Waypoint>(x_interp[i], y_interp[i], yaw));
    }
}

double ReferencePath::interpolate(const std::vector<double>& s,
                                  const std::vector<double>& y,
                                  double query) const{
    if (query <= s.front()) return y.front();
    if (query >= s.back()) return y.back();

    auto upper = std::upper_bound(s.begin(), s.end(), query);
    auto lower = upper - 1;
    size_t idx = std::distance(s.begin(), lower);

    double ratio = (query - s[idx]) / (s[idx + 1] - s[idx]);
    return y[idx] * (1 - ratio) + y[idx + 1] * ratio;
}

void ReferencePath::compute_segment_lengths() {
    segment_lengths_.resize(waypoints_.size(), 0.0);
    for (size_t i = 1; i < waypoints_.size(); ++i) {
        double dx = waypoints_[i]->x - waypoints_[i - 1]->x;
        double dy = waypoints_[i]->y - waypoints_[i - 1]->y;
        segment_lengths_[i] = std::sqrt(dx * dx + dy * dy);
    }
}

double ReferencePath::get_speed(double s) const {
    if (v_profile_.empty()) return 0.0;
    if (s < 0) return v_profile_.front();
    if (s > length()) return v_profile_.back();

    std::vector<double> idx(v_profile_.size());
    std::iota(idx.begin(), idx.end(), 0);
    return interpolate(idx, v_profile_, s);
}

double ReferencePath::get_curvature(double s) const {
    if (kappa_profile_.empty()) return 0.0;
    if (s < 0) return kappa_profile_.front();
    if (s > length()) return kappa_profile_.back();

    std::vector<double> idx(kappa_profile_.size());
    std::iota(idx.begin(), idx.end(), 0);
    return interpolate(idx, kappa_profile_, s);
}

std::shared_ptr<Waypoint> ReferencePath::get_waypoint(int idx) const {
    int safe_idx = std::max(0, std::min(static_cast<int>(waypoints_.size()) - 1, idx));
    return waypoints_[safe_idx];
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
ReferencePath::update_path_constraints(int waypoint_idx, int horizon,
                                       double upper_bound, double lower_bound) const {
    int h = std::max(1, horizon);
    std::vector<double> ub(h, upper_bound);
    std::vector<double> lb(h, -lower_bound);
    std::vector<double> width(h, upper_bound + lower_bound);
    return {ub, lb, width};
}

void ReferencePath::set_speed_profile(const std::vector<double>& v_profile) {
    v_profile_ = v_profile;
    if (waypoints_.size() == v_profile.size()) {
        for (size_t i = 0; i < v_profile.size(); ++i) {
            waypoints_[i]->v_ref = v_profile[i];
        }
    }
}

double ReferencePath::length() const {
    return static_cast<double>(waypoints_.size());
}