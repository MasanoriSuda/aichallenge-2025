// mpc_node.cpp（ROS2ノード化・initとspin分割のベース）
#include "reference_path.hpp"
#include "spatial_bicycle_models.hpp"
#include "MPC.hpp"
#include "csv_loader.hpp"
#include "OdometryInput.hpp"
#include "odom_loader.hpp"

#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <chrono>

class MPCSimulator {
public:
    // メンバ変数
    std::shared_ptr<ReferencePath> reference_path;
    std::shared_ptr<BicycleModel> car;
    std::shared_ptr<MPC> mpc;
    std::vector<std::vector<double>> log_;  // ⬅️ これが必要
    CSVLoader loader;
    OdometryInput odom;
    std::vector<double> wp_speed;
    int N = 30;
    int WAYPOINT_NUM = 10;
    double get_dt() const {
        return car->get_Ts();
    }

    void save_log(const std::string& filename) const {
        std::ofstream file(filename);
        file << "time,x,y,s,e_y,e_psi,v,delta,ref_x,ref_y\n";
        for (const auto& row : log_) {
            for (size_t j = 0; j < row.size(); ++j) {
                file << row[j];
                if (j < row.size() - 1) file << ",";
            }
            file << "\n";
        }
        file.close();
    }
 

    // 初期化関数
    bool init(const std::string& csv_path, const std::string& odom_path) {
        if (!loader.load(csv_path)) {
            std::cerr << "CSVのロードに失敗しました" << std::endl;
            return false;
        }

        // 最初の適当なパスでリファレンス初期化
        std::vector<double> x = {0, 1}, y = {0, 1};
        reference_path = std::make_shared<ReferencePath>(x, y, 0.2, 5.0, 1.5, false);

        car = std::make_shared<BicycleModel>(reference_path, 1.2, 0.8, 0.01);

        odom = load_odom_csv(odom_path);
        car->set_pose_from_odom(odom);

        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
        Q(1, 1) = 0.0;
        Q(2, 2) = 0.0;

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
        R(0, 0) = 0.5;
        R(1, 1) = 0.0;

        Eigen::MatrixXd QN = Q;

        double v_max = 35.0 / 3.6;
        double delta_max = 0.66;
        double ay_max = 5.0;

        std::map<std::string, Eigen::VectorXd> input_constraints = {
            {"umin", (Eigen::Vector2d() << 0.0, -std::tan(delta_max) / car->get_length()).finished()},
            {"umax", (Eigen::Vector2d() << v_max, std::tan(delta_max) / car->get_length()).finished()}
        };

        std::map<std::string, Eigen::VectorXd> state_constraints = {
            {"xmin", (Eigen::Vector3d() << -1e6, -1e6, -1e6).finished()},
            {"xmax", (Eigen::Vector3d() << 1e6, 1e6, 1e6).finished()}
        };

        mpc = std::make_shared<MPC>(car, N, Q, R, QN, state_constraints, input_constraints, ay_max);

        wp_speed = std::vector<double>(10000, 9.7);  // 固定 or 別途補間に置き換え
        return true;
    }

    // ステップ実行
    void spin_once(double t) {
        odom.x = car->get_temporal_state()->x;
        odom.y = car->get_temporal_state()->y;
        odom.yaw = car->get_temporal_state()->psi;
        odom.v = car->get_temporal_state()->v;
        car->set_pose_from_odom(odom);

        double s = car->get_s();
        auto subpath_loop = loader.extractForwardSubpath(s, WAYPOINT_NUM);
        std::vector<double> xs, ys;
        for (const auto& wp : subpath_loop) {
            xs.push_back(wp.x);
            ys.push_back(wp.y);
        }
        reference_path->update_hoge(xs, ys);

        std::vector<double> wp_speed_interp(reference_path->get_all_waypoints().size(), 9.7);
        reference_path->set_speed_profile(wp_speed_interp);

        Eigen::Vector2d u = mpc->get_control(odom, reference_path->get_all_waypoints());
        car->drive(u);

        Waypoint wp = reference_path->get_waypoint(car->get_wp_id());
#if 0
        return {
            t,
            car->get_temporal_state()->x,
            car->get_temporal_state()->y,
            car->get_s(),
            car->get_spatial_state()->e_y,
            car->get_spatial_state()->e_psi,
            u(0), u(1),
            wp.x, wp.y
        };
#else
        log_.push_back({
            t,
            car->get_temporal_state()->x,
            car->get_temporal_state()->y,
            car->get_s(),
            car->get_spatial_state()->e_y,
            car->get_spatial_state()->e_psi,
            u(0),  // v
            u(1),  // delta
            wp.x,
            wp.y
        });
#endif
    }
};
