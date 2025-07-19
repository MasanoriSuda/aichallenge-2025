#pragma once
#include <algorithm>
#include <Eigen/Dense>
#include "waypoint.hpp"
#include "spatial_bicycle_models.hpp"

// ステア角 saturate
inline double saturate_delta(double delta, double min = -1.396, double max = 1.396) {
    return std::clamp(delta, min, max);
}

// s_dot 計算（曲率補正）
inline double compute_s_dot(const SimpleSpatialState& ss, const Waypoint& wp, double v) {
    return v * std::cos(ss.e_yaw) / (1.0 - ss.e_y * wp.kappa);
}

// 重み初期化（チューニング用）
inline void set_default_weights(Eigen::MatrixXd& Q, Eigen::MatrixXd& R) {
    Q = Eigen::MatrixXd::Identity(3, 3);
    Q(0, 0) = 10.0;  // e_y
    Q(1, 1) = 15.0;   // e_psi
    Q(2, 2) = 1.0;   // t（s）

    R = Eigen::MatrixXd::Identity(1, 1);
    R(0, 0) = 0.5;  // delta
}
