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

int main() {
    std::string filename = "raceline_awsim_15km_py.csv";
    auto [wp_x, wp_y, wp_speed] = load_csv(filename);

    // リファレンスパス構築
    std::shared_ptr<ReferencePath> reference_path = std::make_shared<ReferencePath>(
        wp_x, wp_y,
        0.2,   // resolution
        5.0,   // smoothing_distance
        1.5,   // max_width
        false  // circular
    );

    // 補間された速度プロファイル設定
    std::vector<double> wp_speed_interp(reference_path->get_all_waypoints().size());
    for (size_t i = 0; i < wp_speed_interp.size(); ++i) {
        double ratio = static_cast<double>(i) * (wp_speed.size() - 1) / (wp_speed_interp.size() - 1);
        size_t idx = static_cast<size_t>(ratio);
        double frac = ratio - idx;
        double v = (1.0 - frac) * wp_speed[idx] + frac * wp_speed[std::min(idx + 1, wp_speed.size() - 1)];
        wp_speed_interp[i] = v;
    }
    reference_path->set_speed_profile(wp_speed_interp);

    // 車両モデル初期化
    std::shared_ptr<BicycleModel> car = std::make_shared<BicycleModel>(
        reference_path, 1.2, 0.8, 0.01
    );

    OdometryInput odom = load_odom_csv("odom.csv");
    car->set_pose_from_odom(odom);
    
    // MPC 設定
    int N = 30;
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

    std::shared_ptr<MPC> mpc = std::make_shared<MPC>(
        car, N, Q, R, QN,
        state_constraints,
        input_constraints,
        ay_max
    );

    // シミュレーションループ
    double t = 0.0;
    int steps = 3000;
    std::vector<std::vector<double>> log;

    for (int i = 0; i < steps; ++i) {

        auto start = std::chrono::steady_clock::now();
        Eigen::Vector2d u = mpc->get_control();
        //std::cout << "Control vector u: [" << u[0] << ", " << u[1] << "]" << std::endl;
        car->drive(u);

        // wpの中身を表示
        Waypoint wp = reference_path->get_waypoint(car->get_wp_id());
#if 0
        std::cout << "wp.x = " << wp.x << std::endl;
        std::cout << "wp.y = " << wp.y << std::endl;
        std::cout << "wp.psi = " << wp.psi << std::endl; // yaw angle
        std::cout << "wp.kappa = " << wp.kappa << std::endl; // curvature
        std::cout << "wp.v_ref = " << wp.v_ref << std::endl; // reference velocity
#endif

        // 処理の終了時間
        auto end = std::chrono::steady_clock::now();  // endが未定義だったので追加

        // 経過時間をミリ秒で計算
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        // ミリ秒を小数点付きで表示
        double milliseconds = duration.count() / 1000.0; // ミリ秒 -> 秒に変換
        std::cout << "処理時間: " << milliseconds * 1000 << "ミリ秒" << std::endl;  // ミリ秒に戻す

        log.push_back({
            t,
            car->get_temporal_state()->x,
            car->get_temporal_state()->y,
            car->get_s(),
            car->get_spatial_state()->e_y,
            car->get_spatial_state()->e_psi,
            u(0),
            u(1),
            wp.x,
            wp.y
        });

        t += car->get_Ts();
    }

    // CSV出力
    std::ofstream file("mpc_log.csv");
    file << "time,x,y,s,e_y,e_psi,v,delta,ref_x,ref_y\n";
    for (const auto& row : log) {
        for (size_t j = 0; j < row.size(); ++j) {
            file << row[j];
            if (j < row.size() - 1) file << ",";
        }
        file << "\n";
    }
    file.close();

    std::cout << "✅ mpc_log.csv にログを保存しました。" << std::endl;
    return 0;
}
