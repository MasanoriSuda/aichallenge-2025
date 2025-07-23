#include "MPC.hpp"
#include "spatial_bicycle_models.hpp"
#include "reference_path.hpp"

#include <iostream>
#include <numeric>
#include <cmath>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/SparseCore>
#include <Eigen/Sparse>

using namespace Eigen;

MPC::MPC(std::shared_ptr<SpatialBicycleModel> model_,
         int N_,
         const MatrixXd& Q_,
         const MatrixXd& R_,
         const MatrixXd& QN_,
         const std::map<std::string, VectorXd>& state_constraints_,
         const std::map<std::string, VectorXd>& input_constraints_,
         double ay_max_)
    : model(model_), N(N_), Q(Q_), R(R_), QN(QN_),
      state_constraints(state_constraints_),
      input_constraints(input_constraints_),
      ay_max(ay_max_), infeasibility_counter(0) {
    nx = model->n_states;
    nu = 2;
    current_control = VectorXd::Zero(nu * N);
    solver.settings()->setVerbosity(false);
}

void MPC::init_problem() {
    // もし前回のデータが残っていたら、クリアする
    solver.data()->clearHessianMatrix();  // Hessian行列をクリア
    solver.data()->clearLinearConstraintsMatrix();  // 線形制約行列をクリア
    solver.clearSolver();  // ソルバーの状態をクリア

    const auto& umin = input_constraints.at("umin");
    const auto& umax = input_constraints.at("umax");
    const auto& xmin = state_constraints.at("xmin");
    const auto& xmax = state_constraints.at("xmax");

    MatrixXd A_dense = MatrixXd::Zero(nx * (N + 1), nx * (N + 1));
    MatrixXd B_dense = MatrixXd::Zero(nx * (N + 1), nu * N);
    VectorXd ur = VectorXd::Zero(nu * N);
    VectorXd xr = VectorXd::Zero(nx * (N + 1));
    VectorXd uq = VectorXd::Zero(N * nx);
    VectorXd xmin_dyn = VectorXd::Zero((N + 1) * nx);
    VectorXd xmax_dyn = VectorXd::Zero((N + 1) * nx);
    VectorXd umax_dyn = VectorXd::Zero(N * nu);

    for (int i = 0; i <= N; ++i) {
        xmin_dyn.segment(i * nx, nx) = xmin;
        xmax_dyn.segment(i * nx, nx) = xmax;
    }
    for (int i = 0; i < N; ++i) {
        umax_dyn.segment(i * nu, nu) = umax;
    }

    VectorXd kappa_pred(N);
    for (int i = 0; i < N; ++i) {
        double delta_i = (i + 1 < N) ? current_control[2 * (i + 1) + 1] : current_control[2 * (N - 1) + 1];
        kappa_pred[i] = std::tan(delta_i) / model->length;
    }
#if 0
    for (int i = 0; i < N; ++i) {
        std::cout << "kappa_pred[" << i << "] = " << kappa_pred[i] << std::endl;
    }
#endif
    for (int n = 0; n < N; ++n) {
        auto current_wp = model->reference_path->get_waypoint(model->wp_id + n);
        auto next_wp = model->reference_path->get_waypoint(model->wp_id + n + 1);
        double delta_s = current_wp.distanceTo(next_wp);
        double kappa_ref = current_wp.kappa;
        double v_ref = current_wp.v_ref;

        Vector3d f;
        Matrix3d A_lin;
        Matrix<double, 3, 2> B_lin;
        model->linearize(v_ref, kappa_ref, delta_s, f, A_lin, B_lin);

        A_dense.block((n + 1) * nx, n * nx, nx, nx) = A_lin;
        B_dense.block((n + 1) * nx, n * nu, nx, nu) = B_lin;

        ur.segment(n * nu, nu) << v_ref, kappa_ref;
        uq.segment(n * nx, nx) = B_lin * Vector2d(v_ref, kappa_ref) - f;

        double vmax_dyn = std::sqrt(ay_max / (std::abs(kappa_pred[n]) + 1e-12));
        if (vmax_dyn < umax_dyn[n * nu]) {
            umax_dyn[n * nu] = vmax_dyn;
        }
    }

    std::vector<double> ub, lb, width;
    model->reference_path->update_path_constraints(
        model->wp_id + 1, N, 2.0 * model->safety_margin, model->safety_margin,
        ub, lb, width);

    xmin_dyn.head(nx)[0] = model->spatial_state->e_y;
    xmax_dyn.head(nx)[0] = model->spatial_state->e_y;
    for (int i = 1; i <= N; ++i) {
        xmin_dyn.segment(i * nx, 1)[0] = lb[i - 1];
        xmax_dyn.segment(i * nx, 1)[0] = ub[i - 1];
        xr.segment(i * nx, 1)[0] = 0.5 * (lb[i - 1] + ub[i - 1]);
    }

    SparseMatrix<double> Ax = kroneckerProduct(
        MatrixXd::Identity(N + 1, N + 1).sparseView(),
        -MatrixXd::Identity(nx, nx)).eval();
    Ax += A_dense.sparseView();

    SparseMatrix<double> Bu = B_dense.sparseView();

    SparseMatrix<double> Aeq(Ax.rows(), Ax.cols() + Bu.cols());
    std::vector<Triplet<double>> triplets;
    for (int k = 0; k < Ax.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(Ax, k); it; ++it)
            triplets.emplace_back(it.row(), it.col(), it.value());
    for (int k = 0; k < Bu.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(Bu, k); it; ++it)
            triplets.emplace_back(it.row(), it.col() + Ax.cols(), it.value());
    Aeq.setFromTriplets(triplets.begin(), triplets.end());

    SparseMatrix<double> Aineq((N + 1) * nx + N * nu, (N + 1) * nx + N * nu);
    Aineq.setIdentity();

    SparseMatrix<double> A_all(Aeq.rows() + Aineq.rows(), Aeq.cols());
    triplets.clear();
    for (int k = 0; k < Aeq.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(Aeq, k); it; ++it)
            triplets.emplace_back(it.row(), it.col(), it.value());
    for (int k = 0; k < Aineq.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(Aineq, k); it; ++it)
            triplets.emplace_back(it.row() + Aeq.rows(), it.col(), it.value());
    A_all.setFromTriplets(triplets.begin(), triplets.end());

    VectorXd x0(nx);
    for (int i = 0; i < nx; ++i) x0[i] = model->spatial_state->get(i);

    VectorXd leq(uq.size() + x0.size());
    leq.head(x0.size()) = -x0;
    leq.tail(uq.size()) = uq;
    VectorXd ueq = leq;

    VectorXd lineq((N + 1) * nx + N * nu);
    VectorXd uineq((N + 1) * nx + N * nu);
    lineq << xmin_dyn, umin.replicate(N, 1);
    uineq << xmax_dyn, umax_dyn;

    VectorXd l(leq.size() + lineq.size());
    l << leq, lineq;
    VectorXd u(ueq.size() + uineq.size());
    u << ueq, uineq;

    // ★ P行列（Hessian）の構築
    SparseMatrix<double> P(N * nx + nx + N * nu, N * nx + nx + N * nu);
    std::vector<Triplet<double>> P_triplets;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < nx; ++j)
            P_triplets.emplace_back(i * nx + j, i * nx + j, Q(j, j));
    for (int j = 0; j < nx; ++j)
        P_triplets.emplace_back(N * nx + j, N * nx + j, QN(j, j));
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < nu; ++j)
            P_triplets.emplace_back(N * nx + nx + i * nu + j, N * nx + nx + i * nu + j, R(j, j));
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());

    VectorXd q(N * nx + nx + N * nu);
    q.head(N * nx) = -Q.diagonal().replicate(N, 1).cwiseProduct(xr.head(N * nx));
    q.segment(N * nx, nx) = -QN * xr.tail(nx);
    q.tail(N * nu) = -R.diagonal().replicate(N, 1).cwiseProduct(ur);

    solver.data()->setNumberOfVariables(P.cols());
    solver.data()->setNumberOfConstraints(A_all.rows());
    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q);
    solver.data()->setLinearConstraintsMatrix(A_all);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);
    solver.initSolver();
}

Eigen::MatrixXd MPC::update_prediction(const Eigen::MatrixXd& x_reshaped) {
    return x_reshaped;
}


Eigen::Vector2d MPC::get_control() {
    model->get_current_waypoint();
    model->spatial_state = std::make_shared<SimpleSpatialState>(
        model->t2s(*model->current_waypoint, model->temporal_state->to_vector())
    );

    init_problem();
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "OSQP failed" << std::endl;
        return Eigen::Vector2d::Zero();  // or some fallback
    }

    Eigen::VectorXd solution = solver.getSolution();

    Eigen::VectorXd control_signals = solution.tail(N * nu);
    for (int i = 1; i < control_signals.size(); i += 2) {
        control_signals[i] = std::atan(control_signals[i] * model->length);
    }

    double v = control_signals[0];
    double delta = control_signals[1];
    current_control = control_signals;

    Eigen::VectorXd x_flat = solution.head((N + 1) * nx);
    Eigen::MatrixXd x_reshaped = Eigen::Map<const MatrixXd>(x_flat.data(), nx, N + 1).transpose();
    current_prediction = update_prediction(x_reshaped);

    infeasibility_counter = 0;
    return Eigen::Vector2d(v, delta);
}
