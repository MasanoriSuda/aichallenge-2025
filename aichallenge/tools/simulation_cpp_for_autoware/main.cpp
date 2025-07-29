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
    // MPC 設定
    int WAYPOINT_NUM = 10;
    int N = 30;
    CSVLoader loader;
    std::vector<double> wp_x, wp_y, wp_speed;
    const std::string csv_file = "raceline_awsim_15km_py.csv";
    if (!loader.load(csv_file)) {
        std::cerr << "CSVのロードに失敗しました" << std::endl;
        return 1;
    }


    // Waypointのリストを取得
    auto subpath = loader.extractForwardSubpath(0, 2);
    for (int i=0;i<2;++i) {
        wp_x.push_back(i);
        wp_y.push_back(i);
        wp_speed.push_back(i);
    }



    // リファレンスパス構築
    std::shared_ptr<ReferencePath> reference_path = std::make_shared<ReferencePath>(
        wp_x, wp_y,
        0.2,   // resolution
        5.0,   // smoothing_distance
        1.5,   // max_width
        false  // circular
    );

    // 車両モデル初期化
    std::shared_ptr<BicycleModel> car = std::make_shared<BicycleModel>(
        reference_path, 1.2, 0.8, 0.01
    );

    OdometryInput odom = load_odom_csv("odom.csv");
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

    std::shared_ptr<MPC> mpc = std::make_shared<MPC>(
        car, N, Q, R, QN,
        state_constraints,
        input_constraints,
        ay_max
    );


    // シミュレーションループ
    double t = 0.0;
    int steps = 6000;
    std::vector<std::vector<double>> log;

    for (int i = 0; i < steps; ++i) {

        auto start = std::chrono::steady_clock::now();

        odom.x = car->get_temporal_state()->x;
        odom.y = car->get_temporal_state()->y;
        odom.yaw = car->get_temporal_state()->psi;
        odom.v = car->get_temporal_state()->v;
        car->set_pose_from_odom(odom);

        // 仮の s から trajectory を切り出し（例：odom.x + y から推定）
        double s = car->get_s();
        std::cout << "s =" << s << std::endl; 
        // 1. 現在のs値を元に raw waypoint から周囲を抽出
        auto subpath_loop = loader.extractForwardSubpath(s, WAYPOINT_NUM);

        // 2. x, y に分解してスプライン補間 → ローカルReferencePath生成
        std::vector<double> xs, ys;
        for (const auto& wp : subpath_loop) {
            xs.push_back(wp.x);
            ys.push_back(wp.y);
        }
        reference_path->update_hoge(xs,ys);

        // 補間された速度プロファイル設定
        std::vector<double> wp_speed_interp_local(reference_path->get_all_waypoints().size());
        for (size_t i = 0; i < wp_speed_interp_local.size(); ++i) {
            double ratio = static_cast<double>(i) * (wp_speed.size() - 1) / (wp_speed_interp_local.size() - 1);
            size_t idx = static_cast<size_t>(ratio);
            double frac = ratio - idx;
            double v = (1.0 - frac) * wp_speed[idx] + frac * wp_speed[std::min(idx + 1, wp_speed.size() - 1)];
            wp_speed_interp_local[i] = 9.722222222222221;//
        }
        reference_path->set_speed_profile(wp_speed_interp_local);

        Eigen::Vector2d u = mpc->get_control(odom, reference_path->get_all_waypoints());

        car->drive(u);

        // wpの中身を表示
        Waypoint wp = reference_path->get_waypoint(car->get_wp_id());

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
