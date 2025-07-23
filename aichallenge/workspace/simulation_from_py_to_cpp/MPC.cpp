#include "MPC.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <iostream>
#include "constraints.hpp"

// MPC.cpp
MPC::MPC(std::shared_ptr<BicycleModel> model_,
         int N_,
         const Eigen::MatrixXd& Q_,
         const Eigen::MatrixXd& R_,
         const Eigen::MatrixXd& QN_,
         const StateConstraints& state_constraints_,
         const InputConstraints& input_constraints_,
         double dt_)
    : model(model_), N(N_), Q(Q_), R(R_), QN(QN_),
      state_constraints(state_constraints_), input_constraints(input_constraints_),
      dt(dt_) {}

Eigen::Vector2d MPC::get_control() {
    const int nx = model->n_states;
    const int nu = 2;

    // 線形化とリファレンス抽出（仮）
    std::vector<Eigen::MatrixXd> A_list, B_list;
    std::vector<Eigen::VectorXd> f_list;
    std::vector<Eigen::Vector2d> u_ref;
    std::vector<Eigen::VectorXd> x_ref;

    for (int i = 0; i < N; ++i) {
        const auto& wp = model->reference_path->get_waypoint(model->wp_id + i);
        const auto& next_wp = model->reference_path->get_waypoint(model->wp_id + i + 1);

        double delta_s = next_wp->s - wp->s;
        double kappa_ref = wp->kappa;
        double v_ref = wp->v_ref;

        Eigen::VectorXd f;
        Eigen::MatrixXd A, B;
        std::tie(f, A, B) = model->linearize(v_ref, kappa_ref, delta_s);

        f_list.push_back(f);
        A_list.push_back(A);
        B_list.push_back(B);
        u_ref.emplace_back((Eigen::Vector2d() << v_ref, kappa_ref).finished());
    }

    // 制約境界設定
    Eigen::VectorXd umin = input_constraints.umin;
    Eigen::VectorXd umax = input_constraints.umax;
    Eigen::VectorXd xmin = state_constraints.xmin;
    Eigen::VectorXd xmax = state_constraints.xmax;

    // OSQP用のP, q, A, l, uの構築（簡易版、要改良）
    int n_vars = N * nx + N * nu;
    int n_constraints = N * nx;

    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);
    Eigen::SparseMatrix<double> A(n_constraints, n_vars);
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_constraints);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_constraints);

    // P行列の構築
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < nx; ++j) P.insert(i * nx + j, i * nx + j) = Q.coeff(j, j);
        for (int j = 0; j < nu; ++j) P.insert(N * nx + i * nu + j, N * nx + i * nu + j) = R.coeff(j, j);
    }

    std::vector<Eigen::Triplet<double>> A_triplets;

    for (int i = 0; i < N; ++i) {
        const auto& A_i = A_list[i];
        const auto& B_i = B_list[i];
        const auto& f_i = f_list[i];

        // A_i の要素を triplet に追加
        for (int r = 0; r < A_i.rows(); ++r) {
            for (int c = 0; c < A_i.cols(); ++c) {
                double val = A_i(r, c);
                if (std::abs(val) > 1e-10) {
                    A_triplets.emplace_back(i * nx + r, i * nx + c, val);
                }
            }
        }

        // B_i の要素を triplet に追加
        for (int r = 0; r < B_i.rows(); ++r) {
            for (int c = 0; c < B_i.cols(); ++c) {
                double val = B_i(r, c);
                if (std::abs(val) > 1e-10) {
                    A_triplets.emplace_back(i * nx + r, N * nx + i * nu + c, val);
                }
            }
        }

        // l, u の更新
        l.segment(i * nx, nx) = -f_i;
        u.segment(i * nx, nx) = -f_i;
    }

    // Tripletから一括構築
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());


    // OSQPで解く
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    if (!solver.data()->setHessianMatrix(P) ||
        !solver.data()->setGradient(q) ||
        !solver.data()->setLinearConstraintsMatrix(A) ||
        !solver.data()->setLowerBound(l) ||
        !solver.data()->setUpperBound(u)) {
        std::cerr << "OSQP設定失敗" << std::endl;
        return Eigen::Vector2d::Zero();
    }

    if (!solver.initSolver()) {
        std::cerr << "OSQP初期化失敗" << std::endl;
        return Eigen::Vector2d::Zero();
    }

    auto status = solver.solveProblem();
    if (status != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "OSQP solve failed" << std::endl;
        return Eigen::Vector2d(0.0, 0.0);
    }

    Eigen::VectorXd sol = solver.getSolution();
    Eigen::Vector2d u0 = sol.segment(N * nx, 2);  // 最初の制御入力
    return u0;
}
