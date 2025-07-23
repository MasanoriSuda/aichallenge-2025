// MPC.hpp
#pragma once

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <memory>
#include <vector>
#include <utility>

class SpatialBicycleModel;
class ReferencePath;

class MPC {
public:
    MPC(std::shared_ptr<SpatialBicycleModel> model,
        int N,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& QN,
        const std::map<std::string, Eigen::VectorXd>& state_constraints,
        const std::map<std::string, Eigen::VectorXd>& input_constraints,
        double ay_max);

    Eigen::Vector2d get_control();
    Eigen::MatrixXd update_prediction(const Eigen::MatrixXd& spatial_state_prediction);

    Eigen::MatrixXd current_prediction;

private:
    void init_problem();

    int N;
    int nx;
    int nu;

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd QN;

    std::map<std::string, Eigen::VectorXd> state_constraints;
    std::map<std::string, Eigen::VectorXd> input_constraints;

    double ay_max;

    std::shared_ptr<SpatialBicycleModel> model;

    std::vector<double> current_prediction_x;
    std::vector<double> current_prediction_y;
    Eigen::VectorXd current_control;
    int infeasibility_counter;

    OsqpEigen::Solver solver;
};