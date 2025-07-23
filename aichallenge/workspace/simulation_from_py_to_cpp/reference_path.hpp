// reference_path.hpp
#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>

class Waypoint {
public:
    double x;
    double y;
    double psi;
    double kappa;
    double v_ref;

    Waypoint(double x_, double y_, double yaw = 0.0, double kappa_ = 0.0, double v_ref_ = 0.0)
        : x(x_), y(y_), psi(yaw), kappa(kappa_), v_ref(v_ref_) {}

    double distanceTo(const Waypoint& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

class ReferencePath {
public:
    ReferencePath(const std::vector<double>& wp_x, const std::vector<double>& wp_y,
                  double path_resolution, double smoothing_distance, double max_width,
                  bool circular = false);

    Waypoint get_waypoint(int idx) const;
    double get_speed(double s) const;
    double get_curvature(double s) const;
    void set_speed_profile(const std::vector<double>& v_profile);
    void update_path_constraints(int waypoint_idx, int horizon,
                                 double upper_bound, double lower_bound,
                                 std::vector<double>& ub, std::vector<double>& lb,
                                 std::vector<double>& width);
    const std::vector<std::shared_ptr<Waypoint>>& get_all_waypoints() const {
        return waypoints;
    }


    std::vector<std::shared_ptr<Waypoint>> waypoints;
    std::vector<double> segment_lengths;
    std::vector<double> v_profile;
    std::vector<double> kappa_profile;
    double resolution;
    double smoothing_distance;
    double max_width;
    bool circular;

private:
    std::vector<std::shared_ptr<Waypoint>> construct_path(const std::vector<double>& wp_x,
                                                           const std::vector<double>& wp_y);
    std::vector<double> compute_segment_lengths();
};
