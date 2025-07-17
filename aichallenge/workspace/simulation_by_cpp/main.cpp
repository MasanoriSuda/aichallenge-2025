#include "spatial_bicycle_models.hpp"
#include "reference_path.hpp"
#include "waypoint.hpp"
#include "csv_loader.hpp"
#include "MPC.hpp"  // ← MPC追加！

#include <iostream>
#include <fstream>

int main() {
    std::string filename = "../raceline_awsim_15km_py.csv";
    auto waypoints = load_csv(filename);
    std::cout << "Loaded " << waypoints.size() << " waypoints from CSV." << std::endl;

    auto ref_path = std::make_shared<ReferencePath>();
    ref_path->set_points(waypoints);     // CSVから読み込んだwaypoints
    std::vector<double> xs, ys;
    for (const auto& wp : waypoints) {
        xs.push_back(wp->x);
        ys.push_back(wp->y);
    }
    std::cout << "xs.size() = " << xs.size() << std::endl;

    ref_path->construct_path(xs, ys);

    ref_path->set_speed_profile(10.0);  // ← ★ここを変更！

    BicycleModel model(ref_path, 2.0, 1.0, 0.1);  // length=2.0m, width=1.0m, Ts=0.1s
    MPC mpc;  // ← MPCインスタンス作成

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
    log_file << "step,x,y,yaw,e_y,e_yaw,t,delta,v_ref\n";

    for (int i = 0; i <  200; ++i) {
        // 空間状態を取得
        auto ss = dynamic_cast<SimpleSpatialState*>(model.spatial_state.get());
        if (!ss) {
            std::cerr << "Error: SpatialState is not a SimpleSpatialState." << std::endl;
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
        mpc.set_reference_waypoint(wp);
        Eigen::VectorXd u_mpc = mpc.solve(x, A, B);

        double delta = u_mpc(0);

        // ステア角制限：±1.396 rad
        double max_steer = 1.396;
        delta = std::max(-max_steer, std::min(max_steer, delta));

        double safe_v = std::max(wp->v_ref, 1.0);
        Eigen::Vector2d u(safe_v, delta);

        model.drive(u);

        auto ts = model.temporal_state.get();
        ss->update(*ts, *wp);
        ss->t += dt * safe_v;  // ← ★追加！s方向に進む
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
                << delta << "," << wp->v_ref << "\n";
        std::cout << "[step " << i << "] s=" << ss->t << ", delta_ref=" << wp->delta_ref_ << std::endl;

    }


    return 0;
}
