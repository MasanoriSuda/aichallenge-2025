#pragma once

// 標準ライブラリ
#include <memory>
#include <vector>
#include <map>
#include <string>

// EigenとOSQP
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

// 自作ヘッダー
#include "reference_path.hpp"
#include "OdometryInput.hpp"

// 前方宣言
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
    Eigen::Vector2d get_control(const OdometryInput& odom, const std::vector<std::shared_ptr<Waypoint>>& trajectory);
    Eigen::Vector2d get_control_org(const OdometryInput& odom, const std::vector<std::shared_ptr<Waypoint>>& trajectory);
    Eigen::MatrixXd update_prediction(const Eigen::MatrixXd& spatial_state_prediction);

    std::vector<std::shared_ptr<Waypoint>> raw_waypoints_;
    std::vector<std::shared_ptr<Waypoint>> extract_raw_subpath(double s, int N) const;

private:
    void init_problem();
    void init_problem(const Eigen::VectorXd& x0,
                      const Eigen::VectorXd& x_ref,
                      const Eigen::VectorXd& u_ref);  // ←★コレがなかった

    int nx;
    int nu;

    std::shared_ptr<SpatialBicycleModel> model;  // ←順番変更済み
    int N;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd QN;

    Eigen::VectorXd x_ref_;
    Eigen::VectorXd u_ref_;

    std::map<std::string, Eigen::VectorXd> state_constraints;
    std::map<std::string, Eigen::VectorXd> input_constraints;

    double ay_max;

    std::vector<double> current_prediction_x;
    std::vector<double> current_prediction_y;
    Eigen::VectorXd current_control;
    int infeasibility_counter;

    OsqpEigen::Solver solver;
};