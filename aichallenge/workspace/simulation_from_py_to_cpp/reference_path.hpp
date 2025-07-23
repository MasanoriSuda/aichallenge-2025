// reference_path.hpp
#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

struct Waypoint {
    double x;
    double y;
    double psi = 0.0;
    double speed;
    double kappa = 0.0;
    double v_ref = 0.0;
    double s = 0.0;  // ✅ 経路長（ReferencePathで使う）

    Waypoint() = default;

    // ✅ 追加（3引数用）
    Waypoint(double x_, double y_, double psi_)
        : x(x_), y(y_), psi(psi_) {}

    Waypoint(double x_, double y_, double psi_, double speed_, double kappa_, double s_)
            : x(x_), y(y_), psi(psi_), speed(speed_), kappa(kappa_), s(s_) {}

    double distanceTo(const Waypoint& other) const {
        return std::sqrt((x - other.x)*(x - other.x) + (y - other.y)*(y - other.y));
    }
};


class ReferencePath {
public:
    ReferencePath(const std::vector<double>& wp_x, const std::vector<double>& wp_y,
                  double resolution, double smoothing_distance, double max_width,
                  bool circular = false);

    std::shared_ptr<Waypoint> get_waypoint(int idx) const;
    double get_speed(double s) const;
    double get_curvature(double s) const;
    void set_speed_profile(const std::vector<double>& v_profile);
        std::tuple<std::vector<double>,
                std::vector<double>,
                std::vector<double>>
        update_path_constraints(int waypoint_idx, int horizon,
                                double ds_lookahead, double ds_behind)  const;

    std::vector<std::shared_ptr<Waypoint>> waypoints;
    //int length;

    const std::vector<double>& get_segment_lengths() const {
        return segment_lengths_;
    }

    const std::vector<double>& segment_lengths() const { return segment_lengths_; }
    size_t size() const {
        return waypoints_.size();
    }

    double length() const;  // ★追加
    double interpolate(const std::vector<double>& s,
                    const std::vector<double>& values,
                    double query_s) const;
    void construct_path(const std::vector<double>& wp_x,
                                   const std::vector<double>& wp_y);

private:
    std::vector<double> wp_x_, wp_y_;
    double resolution_;
    double smoothing_distance_;
    double max_width_;
    bool circular_;
    std::vector<std::shared_ptr<Waypoint>> waypoints_;  // ★追加

    std::vector<double> segment_lengths_;
    std::vector<double> v_profile_;
    std::vector<double> kappa_profile_;

    void compute_segment_lengths();
};
