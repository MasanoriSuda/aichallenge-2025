#include "MPC.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>  // for std::min, std::max
#include <cmath>

Eigen::VectorXd MPC::solve(const Eigen::VectorXd& x,
                           const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B) {
    Eigen::VectorXd u(1);

    // ゲイン調整
    double k_psi = 3.0;  // e_psi
    double k_y   = 1.5;  // e_y

    // 目標操舵角（経路から）
    double delta_ref = 0.0;
    if (ref_waypoint_) {
        delta_ref = ref_waypoint_->delta_ref_;
        std::cout << "not null" << std::endl;
    } else {
        std::cout << "nullprtr" << std::endl;
    }
#if 1    
    std::cout << "solve 出力 delta_ref = " << delta_ref << std::endl;
#endif
    // 状態ベクトル x = [e_y, e_psi, t]
    double e_y   = x(0);
    double e_psi = x(1);

    // 差分制御（delta_refからの補正）
    double delta = delta_ref - (k_y * e_y + k_psi * e_psi);
    // ステア角制限（保険）
    double max_steer = 1.396;  // 80度 = 約1.396rad
    delta = std::max(-max_steer, std::min(max_steer, delta));

    u(0) = delta;
#if 0
    // デバッグログ
    std::cout << "[MPC::solve] e_y=" << e_y
              << ", e_psi=" << e_psi
              << ", delta_ref=" << delta_ref
              << ", delta_out=" << delta << std::endl;
#endif

#if 0
        std::cout << "solve 出力 delta = " << delta << std::endl;
#endif

#if 0
    std::cout << "solve 出力 delta = " << delta << std::endl;
#endif

    return u;
}
