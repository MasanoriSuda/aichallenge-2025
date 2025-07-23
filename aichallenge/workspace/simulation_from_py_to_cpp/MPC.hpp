// MPC.hpp
#pragma once

#include "spatial_bicycle_models.hpp"
#include "reference_path.hpp"
#include "constraints.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

class MPC {
public:
    // MPC.hpp
    MPC(std::shared_ptr<BicycleModel> model,
        int N,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& QN,
        const StateConstraints& state_constraints,
        const InputConstraints& input_constraints,
        double dt);


    Eigen::Vector2d get_control();

    std::vector<Eigen::Vector2d> current_prediction;

private:
    void init_problem();
    std::vector<Eigen::Vector2d> update_prediction(const Eigen::MatrixXd& spatial_state_prediction);

    std::shared_ptr<BicycleModel> model;
    int N;
    int nx;
    int nu;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd QN;
    StateConstraints state_constraints;
    InputConstraints input_constraints;
    double dt;

    Eigen::VectorXd current_control;
    int infeasibility_counter = 0;

    OsqpEigen::Solver solver;
};
