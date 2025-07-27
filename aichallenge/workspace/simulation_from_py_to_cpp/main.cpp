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
    int steps = 1000;
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
        
        // 1. 現在のs値を元に raw waypoint から周囲を抽出
        auto raw_subpath = reference_path->extract_raw_subpath(s, N);

        // 2. x, y に分解してスプライン補間 → ローカルReferencePath生成
        std::vector<double> xs, ys;
        for (const auto& wp : raw_subpath) {
            xs.push_back(wp->x);
            ys.push_back(wp->y);
        }

        //kappaを入れる
        std::vector<double> kappa_list(xs.size(), 0.0);  // 端は0にしておく

        for (size_t i = 1; i < xs.size() - 1; ++i) {
            double x_prev = xs[i - 1];
            double x_curr = xs[i];
            double x_next = xs[i + 1];
            double y_prev = ys[i - 1];
            double y_curr = ys[i];
            double y_next = ys[i + 1];

            double dx1 = x_curr - x_prev;
            double dy1 = y_curr - y_prev;
            double dx2 = x_next - x_curr;
            double dy2 = y_next - y_curr;

            double num = dx2 * dy1 - dy2 * dx1;
            double denom = std::pow(std::pow(x_next - x_prev, 2) + std::pow(y_next - y_prev, 2), 1.5);

            if (std::abs(denom) > 1e-6) {
                kappa_list[i] = num / denom;
            } else {
                kappa_list[i] = 0.0;  // 曲率が極端なときは0で安定化
            }
        }
        kappa_list[0] = 2 * kappa_list[1] - kappa_list[2];
        kappa_list.back() = 2 * kappa_list[kappa_list.size() - 2] - kappa_list[kappa_list.size() - 3];


        for(int i = 0; i< kappa_list.size();++i){
            std::cout << "kapa_list[" <<i <<"]" << kappa_list[i] << std::endl;
        }


        std::shared_ptr<ReferencePath> local_ref = std::make_shared<ReferencePath>(
            xs, ys,
            0.2,   // resolution
            5.0,   // smoothing_distance
            1.5,   // max_width
            false  // circular
        );


        // 補間された速度プロファイル設定
        std::vector<double> wp_speed_interp_local(local_ref->get_all_waypoints().size());
        for (size_t i = 0; i < wp_speed_interp_local.size(); ++i) {
            double ratio = static_cast<double>(i) * (wp_speed.size() - 1) / (wp_speed_interp_local.size() - 1);
            size_t idx = static_cast<size_t>(ratio);
            double frac = ratio - idx;
            double v = (1.0 - frac) * wp_speed[idx] + frac * wp_speed[std::min(idx + 1, wp_speed.size() - 1)];
            wp_speed_interp_local[i] = 9.722222222222221;//
        }
        local_ref->set_speed_profile(wp_speed_interp_local);

        std::vector<double> kappa_interp_local(local_ref->get_all_waypoints().size());
        for (size_t i = 0; i < kappa_interp_local.size(); ++i) {
            double ratio = static_cast<double>(i) * (kappa_list.size() - 1) / (kappa_interp_local.size() - 1);
            size_t idx = static_cast<size_t>(ratio);
            double frac = ratio - idx;
            double kappa_val = (1.0 - frac) * kappa_list[idx] + frac * kappa_list[std::min(idx + 1, kappa_list.size() - 1)];
            kappa_interp_local[i] = kappa_val;
        }

        // Waypoint に反映
        auto waypoints = local_ref->get_all_waypoints();
        for (size_t i = 0; i < waypoints.size(); ++i) {
            waypoints[i]->kappa = kappa_interp_local[i];
            waypoints[i]->delta_ref = std::atan(kappa_interp_local[i] * 1.087);  // 必要なら
        }

#if 1
        Eigen::Vector2d u = mpc->get_control(odom, local_ref->get_all_waypoints());
#else
        Eigen::Vector2d u = mpc->get_control();
#endif
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
