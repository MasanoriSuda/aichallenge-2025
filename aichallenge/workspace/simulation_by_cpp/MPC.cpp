#include "MPC.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <vector>

Eigen::VectorXd MPC::solve(const Eigen::VectorXd& x0,
                           const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B) {
    const int N = 10;
    const int nx = x0.size();  // 3
    const int nu = B.cols();   // 1

    const int n_vars = N * nx + N * nu;
    const int n_constraints = N * nx;

    // H行列（2次コスト項）
    Eigen::SparseMatrix<double> H(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> H_triplets;

    for (int i = 0; i < N; ++i) {
        for (int r = 0; r < nx; ++r) {
            int idx = i * nx + r;
            H_triplets.emplace_back(idx, idx, Q_(r, r));
        }
        for (int r = 0; r < nu; ++r) {
            int idx = N * nx + i * nu + r;
            H_triplets.emplace_back(idx, idx, R_(r, r));
        }
    }
    H.setFromTriplets(H_triplets.begin(), H_triplets.end());

    // fベクトル（1次コスト項） ← (x - x_ref)^T Q, (u - u_ref)^T R
    Eigen::VectorXd f = Eigen::VectorXd::Zero(n_vars);

    for (int i = 0; i < N; ++i) {
        if (!ref_path_ || !ref_waypoint_) break;

        double s_ref = x0(2) + i * ref_waypoint_->v_ref * 0.1;  // dt = 0.1
        auto wp = ref_path_->get_waypoint(s_ref);

        Eigen::VectorXd x_ref(nx);
        x_ref << 0.0, 0.0, s_ref;

        Eigen::VectorXd u_ref(nu);
        u_ref << wp->delta_ref_;

        for (int r = 0; r < nx; ++r) {
            int idx = i * nx + r;
            f(idx) = -Q_(r, r) * x_ref(r);
        }

        for (int r = 0; r < nu; ++r) {
            int idx = N * nx + i * nu + r;
            f(idx) = -R_(r, r) * u_ref(r);
        }
    }

    // 制約 Aeq * z = beq
    Eigen::SparseMatrix<double> Aeq(n_constraints, n_vars);
    std::vector<Eigen::Triplet<double>> Aeq_triplets;
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_constraints);

    for (int i = 0; i < N; ++i) {
        // x[i+1]
        for (int r = 0; r < nx; ++r) {
            int row = i * nx + r;
            int col = i * nx + r;
            Aeq_triplets.emplace_back(row, col, 1.0);
        }

        // x[i]
        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nx; ++c) {
                int row = i * nx + r;
                int col = (i - 1) * nx + c;
                if (i == 0) {
                    beq(row) -= A(r, c) * x0(c);
                } else {
                    Aeq_triplets.emplace_back(row, col, -A(r, c));
                }
            }
        }

        // u[i]
        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nu; ++c) {
                int row = i * nx + r;
                int col = N * nx + i * nu + c;
                Aeq_triplets.emplace_back(row, col, -B(r, c));
            }
        }
    }
    Aeq.setFromTriplets(Aeq_triplets.begin(), Aeq_triplets.end());

    Eigen::VectorXd leq = beq;
    Eigen::VectorXd ueq = beq;

    // 入力制約
    double delta_min = -1.396;
    double delta_max =  1.396;
    Eigen::VectorXd l_var = Eigen::VectorXd::Constant(n_vars, -1e6);
    Eigen::VectorXd u_var = Eigen::VectorXd::Constant(n_vars,  1e6);
    for (int i = 0; i < N; ++i) {
        int idx = N * nx + i * nu;
        l_var(idx) = delta_min;
        u_var(idx) = delta_max;
    }

    // OSQP solver設定
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    if (!solver.data()->setHessianMatrix(H)) return Eigen::VectorXd::Zero(nu);
    if (!solver.data()->setGradient(f)) return Eigen::VectorXd::Zero(nu);
    if (!solver.data()->setLinearConstraintsMatrix(Aeq)) return Eigen::VectorXd::Zero(nu);
    if (!solver.data()->setLowerBound(leq)) return Eigen::VectorXd::Zero(nu);
    if (!solver.data()->setUpperBound(ueq)) return Eigen::VectorXd::Zero(nu);
    if (!solver.initSolver()) return Eigen::VectorXd::Zero(nu);
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "[MPC] OSQP solve failed" << std::endl;
        return Eigen::VectorXd::Zero(nu);
    }

    Eigen::VectorXd sol = solver.getSolution();
    Eigen::VectorXd u0 = sol.segment(N * nx, nu);

#if 1
    std::cout << "[MPC::solve-MPC] u0=" << u0(0)
              << ", from x=[" << x0.transpose() << "]" << std::endl;
#endif

    return u0;
}


