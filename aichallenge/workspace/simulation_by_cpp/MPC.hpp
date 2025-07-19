#ifndef MPC_HPP
#define MPC_HPP

#include <Eigen/Dense>
#include <iostream>
#include <memory>             // ← 忘れずに追加！
#include "waypoint.hpp"       // ← Waypoint定義が必要
#include "reference_path.hpp"

class MPC {
public:
    MPC() {
        N_ = 10;  // ホライズン長
        n_ = 3;   // 状態次元
        m_ = 1;   // 入力次元

        Q_ = Eigen::MatrixXd::Identity(n_, n_) * 1.0;
        R_ = Eigen::MatrixXd::Identity(m_, m_) * 0.1;
    }
    Eigen::VectorXd solve(const Eigen::VectorXd& x0,
                          const ReferencePath& ref_path);
    // MPC.hpp のクラスに追加
    std::shared_ptr<Waypoint> ref_waypoint_;
    void set_reference_waypoint(std::shared_ptr<Waypoint> wp) {
        ref_waypoint_ = wp;
    }

    std::shared_ptr<ReferencePath> ref_path_;  // ★追加これ！
    // ★追加：ReferencePathセット用（今回必要）
    void set_reference_path(std::shared_ptr<ReferencePath> path) {
        ref_path_ = path;
    }    
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    void set_horizon(int N) {
        N_ = N;
    }
        double Q_e_y_;
        double Q_e_yaw_;
        double Q_t_;
        double R_delta_;        


private:
    double max_steer_;
    int N_;  // ホライズン長
    int n_;  // 状態次元
    int m_;  // 入力次元
};

#endif
