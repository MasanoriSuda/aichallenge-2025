#include "MPC.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <vector>

void linearize(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double v, double delta, double dt) {
    const double L = 1.087;
    const double eps = 1e-6;

    double beta = std::atan(std::tan(delta));  // small delta 近似

    A = Eigen::MatrixXd::Identity(3, 3);
    B = Eigen::MatrixXd::Zero(3, 1);

    A(0, 1) = v * dt;
    A(2, 1) = dt * std::tan(delta) / (v * std::pow(std::cos(delta), 2.0) + eps);

    B(1, 0) = v * dt / (L * std::pow(std::cos(delta), 2.0) + eps);
    B(2, 0) = std::tan(delta) / (v * std::cos(delta) + eps);
}


Eigen::VectorXd MPC::solve(const Eigen::VectorXd& x0, const ReferencePath& ref_path) {
    const int nx = 3;  // 状態次元 [e_y, e_yaw, t]
    const int nu = 1;  // 入力次元 [delta]
    const int N = N_;  // ホライズン長

    // === QP設定 ===
    Eigen::MatrixXd Q(nx, nx); Q.setZero();
    Q(0, 0) = Q_e_y_;
    Q(1, 1) = Q_e_yaw_;
    Q(2, 2) = Q_t_;

    Eigen::MatrixXd R(nu, nu); R.setIdentity();
    R *= R_delta_;

    const double v_ref = ref_path.get_speed(x0(2));  // s=tに対応した速度
    const double dt = 0.1;  // 時間ステップ（必要ならパラメータ化）

    // === 各ステップのリファレンス生成 ===
    std::vector<Eigen::MatrixXd> A_list, B_list;
    std::vector<Eigen::VectorXd> u_ref_list;
    std::vector<Eigen::VectorXd> x_ref_list;

    for (int i = 0; i < N; ++i) {
        double s = x0(2) + i * v_ref * dt;

        // 参照点
        auto wp = ref_path.get_waypoint(s);
        double kappa = wp->kappa;
        //std::cout << "kappa =" << kappa << std::endl; 
        double delta_ref = wp->delta_ref_;
        double v = wp->v_ref;

        // 線形化
        Eigen::MatrixXd A(nx, nx), B(nx, nu);
        linearize(A, B, v, kappa, dt);
        A_list.push_back(A);
        B_list.push_back(B);

        // u_ref（delta）
        Eigen::VectorXd u_ref(nu);
        u_ref << delta_ref;
        u_ref_list.push_back(u_ref);

        // x_ref（理想状態：e_y = e_yaw = 0, t = s）
        Eigen::VectorXd x_ref(nx);
        x_ref << 0.0, 0.0, s;
        x_ref_list.push_back(x_ref);
    }

    // === QP行列生成 ===
    // → ここでQ, R, A_list, B_list, x_ref_list, u_ref_listを使って
    //    H, f, A_qp, l, u を構築してOsqpEigenに渡す
    // （これは今のsolve()をベースに書き換え）

    // 最終的に解を得て、u0を返す
    Eigen::VectorXd u_opt(nu);
    //u_opt << 0.0;  // TODO: QP解から取り出して返す
    u_opt << u_ref_list[0];  // TODO: QP解から取り出して返す

    return u_opt;
}





