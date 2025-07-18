#include "MPC.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>  // for std::min, std::max
#include <cmath>

// MPC.cpp

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>  // std::min, std::max

Eigen::VectorXd MPC::solve(const Eigen::VectorXd& x,
                           const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B) {
    Eigen::VectorXd u(1);  // 出力：ステア角のみ

    // 状態ベクトル x = [e_y, e_psi, t]
    double e_y   = x(0);
    double e_psi = x(1);
    double v_ref = 10.0;

    if (ref_waypoint_) {
        v_ref = ref_waypoint_->v_ref;
    }

    // --- 弱めの差分制御ゲイン ---
    double base_k_y   = 0.5;   // ← 安定性重視
    double base_k_psi = 1.0;
    double scale = 1.0;
    double k_y   = base_k_y * scale;
    double k_psi = base_k_psi * scale;

    // --- フィードバック制御（delta_refなし） ---
    double delta = - (k_y * e_y + k_psi * e_psi);

    // --- ステア角制限 ---
    double max_steer = 1.396;
    delta = std::max(-max_steer, std::min(max_steer, delta));

    u(0) = delta;

#if 1
    std::cout << "[MPC::solve] e_y=" << e_y
              << ", e_psi=" << e_psi
              << ", delta_out=" << delta
              << ", v_ref=" << v_ref
              << ", k_y=" << k_y
              << ", k_psi=" << k_psi
              << std::endl;
#endif

    return u;
}




















