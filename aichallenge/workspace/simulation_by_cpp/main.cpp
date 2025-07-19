#include "spatial_bicycle_models.hpp"
#include "reference_path.hpp"
#include "waypoint.hpp"
#include "csv_loader.hpp"
#include "MPC.hpp"  // ← MPC追加！
#include "mpc_utils.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>  // ← これも必要
#include <Eigen/Dense>

// std::vector<double> → Eigen::DiagonalMatrix
//Eigen::MatrixXd diag(const std::vector<double>& vec) {
//    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(vec.size(), vec.size());
//    for (size_t i = 0; i < vec.size(); ++i) {
//        mat(i, i) = vec[i];
//    }
//    return mat;
//}


int main() {
    //std::string filename = "../raceline_awsim_15km.csv";
    //auto waypoints = load_csv(filename);
    std::vector<TrajectoryPoint> traj_points = load_autoware_csv("../raceline_awsim_15km.csv");

    std::shared_ptr<ReferencePath> ref_path = std::make_shared<ReferencePath>();  // ← ★追加

    std::vector<std::shared_ptr<Waypoint>> waypoints;
    for (const auto& tp : traj_points) {
        auto wp = std::make_shared<Waypoint>();
        wp->x = tp.x;
        wp->y = tp.y;
        wp->yaw = tp.yaw;
        wp->v = tp.v;
        wp->kappa = tp.kappa;
        waypoints.push_back(wp);
    }

    std::vector<double> xs, ys;
    for (const auto& wp : waypoints) {
        xs.push_back(wp->x);
        ys.push_back(wp->y);
    }
    std::cout << "xs.size() = " << xs.size() << std::endl;

    ref_path->construct_path(xs, ys);           // ← スプライン補間で ref_path->waypoints 作成
    ref_path->set_speed_profile(3.0);          // ← 曲率に応じて速度セット
    ref_path->set_points(ref_path->waypoints);  // ← それを元に delta_ref_ 平滑化 ← 🔥これが重要！！



    BicycleModel model(ref_path, 2.0, 1.0, 0.1);  // length=2.0m, width=1.0m, Ts=0.1s
    MPC mpc;  // ← MPCインスタンス作成

    // ✅ ホライズン・重みセット（最初にやっておく）

#if 0
    mpc.set_horizon(15);
    set_default_weights(mpc.Q_, mpc.R_);
#else

    std::ifstream ifs("config.json");
    if (!ifs) {
        std::cerr << "config.json が開けませんでした。" << std::endl;
        return 1;
    }

    nlohmann::json config;
    ifs >> config;

    // 重みとホライズンを設定
    mpc.set_horizon(config["N"].get<int>());
    Eigen::Vector3d Q_vec = Eigen::Vector3d::Map(config["Q"].get<std::vector<double>>().data());
    Eigen::MatrixXd Q = Q_vec.asDiagonal();

    Eigen::VectorXd R_vec = Eigen::VectorXd::Map(config["R"].get<std::vector<double>>().data(), 1);
    Eigen::MatrixXd R = R_vec.asDiagonal();
    mpc.Q_ = Q;
    mpc.R_ = R;
#endif


    double dt = 0.1;

    int start_index = 0;
    if (start_index >= waypoints.size()) {
        std::cerr << "Error: start_index is out of range!" << std::endl;
        return 1;
    }

    auto wp_start = ref_path->waypoints[start_index];

    // TemporalState（時系列状態）初期化
    model.temporal_state->x = wp_start->x;
    model.temporal_state->y = wp_start->y;
    model.temporal_state->yaw = wp_start->yaw;  // ←これは維持（走行方向にセット）

    // 初期状態ログ出力（デバッグ用）
    std::cout << "Initial State:" << std::endl;
    std::cout << "  x = " << model.temporal_state->x << std::endl;
    std::cout << "  y = " << model.temporal_state->y << std::endl;
    std::cout << "  yaw = " << model.temporal_state->yaw << std::endl;


    // SpatialState（空間状態）初期化
    auto ss_start = dynamic_cast<SimpleSpatialState*>(model.spatial_state.get());
    if (!ss_start) {
        std::cerr << "Error: SpatialState is not a SimpleSpatialState." << std::endl;
        return 1;
    }
    ss_start->t = wp_start->s;  // 経路上の位置を設定
    ss_start->update(*model.temporal_state, *wp_start);  // ← e_y, e_psi更新

        // ログファイルを開く（必要なら）
    std::ofstream log_file("mpc_log.csv");
    log_file << "step,x,y,yaw,e_y,e_yaw,t,delta,v_ref,delta_ref\n";


    for (int i = 0; i <  350; ++i) {
        // 空間状態を取得
        auto ss = dynamic_cast<SimpleSpatialState*>(model.spatial_state.get());
        if (!ss) {
            std::cerr << "Error: SpatialState is not a SimpleSpatialState." << std::endl;
            break;
        }

        if (ss->t >= ref_path->waypoints.back()->s) {
            std::cout << "✅ Reached end of path. Breaking simulation." << std::endl;
            break;
        }

        double s = ss->t;
        auto wp = ref_path->get_waypoint(s);

        // 線形化（A, B行列取得）
        Eigen::MatrixXd A, B;
        model.linearize(*ss, wp->v_ref, wp->kappa, dt, A, B);

        Eigen::VectorXd x(3);
        x << ss->e_y, ss->e_yaw, ss->t;


        // main.cpp 側でsolve前に追加
        mpc.set_reference_path(ref_path);  // ← solve前にこれが必要！
        mpc.set_reference_waypoint(wp);
        Eigen::VectorXd u_mpc = mpc.solve(x, *ref_path);

        // ✅ ステア角 saturate
        double delta = saturate_delta(u_mpc(0));  // ← ⬅️ ここが重要！

        // ✅ v: 安全速度（min 1.0）
        double safe_v = std::max(wp->v_ref, 1.0);
        Eigen::Vector2d u(safe_v, delta);

        model.drive(u);

        auto ts = model.temporal_state.get();
        ss->update(*ts, *wp);

        // ✅ 曲率補正付き s方向更新
        double s_dot = compute_s_dot(*ss, *wp, safe_v);
        ss->t += dt * s_dot;

        model.update_current_waypoint();

#if 0        
        // ✔️ 統一ログ出力（cout）
        std::cout << i
                << ", x=" << ts->x
                << ", y=" << ts->y
                << ", yaw=" << ts->yaw
                << ", e_y=" << ss->e_y
                << ", e_yaw=" << ss->e_yaw
                << ", t=" << ss->t
                << ", delta=" << delta
                << ", v_ref=" << wp->v_ref
                << ", delta_ref=" << wp->delta_ref_  // ← これ追加！
                << std::endl;
#endif

        // ✔️ CSVログ出力
        log_file << i << "," << ts->x << "," << ts->y << "," << ts->yaw << ","
                << ss->e_y << "," << ss->e_yaw << "," << ss->t << ","
                << delta << "," << wp->v_ref << "," << wp->delta_ref_ << "\n";


    }


    return 0;
}
