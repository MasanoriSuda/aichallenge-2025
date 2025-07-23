// constraints.hpp
#pragma once
#include <Eigen/Dense>

struct StateConstraints {
    Eigen::VectorXd xmin;
    Eigen::VectorXd xmax;

    StateConstraints() = default;
    StateConstraints(const Eigen::VectorXd& xmin_, const Eigen::VectorXd& xmax_)
        : xmin(xmin_), xmax(xmax_) {}
};

struct InputConstraints {
    Eigen::VectorXd umin;
    Eigen::VectorXd umax;

    InputConstraints() = default;
    InputConstraints(const Eigen::VectorXd& umin_, const Eigen::VectorXd& umax_)
        : umin(umin_), umax(umax_) {}
};
