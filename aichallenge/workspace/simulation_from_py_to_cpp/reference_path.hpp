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
    double s;           // ✅ ← 必ず追加！
    double delta_ref = 0.0;  // ✅ ← これを追加！
    // 完全コンストラクタ
    Waypoint(double x_, double y_, double psi_, double kappa_, double v_ref_, double s_)
        : x(x_), y(y_), psi(psi_), kappa(kappa_), v_ref(v_ref_), s(s_), delta_ref(0.0) {}

    Waypoint(double x_, double y_, double psi_)
        : x(x_), y(y_), psi(psi_), kappa(0.0), v_ref(0.0), s(0.0), delta_ref(0.0) {}
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

    int get_closest_index(double s) const;

    // ReferencePath extension (ReferencePath.hpp or where appropriate)
    std::vector<Waypoint> extract_subpath(double s, int N) const;  // Declaration

    double get_closest_s(double x, double y) const;

    Waypoint get_waypoint_from_s(double s) const;

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
