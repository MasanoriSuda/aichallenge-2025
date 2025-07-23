#include "reference_path.hpp"
#include "spatial_bicycle_models.hpp"
#include "MPC.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

int main() {
    // CSV読み込み
    std::ifstream file("raceline_awsim_15km_py.csv");
    std::string line;
    std::vector<double> wp_x, wp_y, wp_speed;

    std::getline(file, line); // ヘッダー読み飛ばし
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> tokens;
        while (std::getline(ss, val, ',')) tokens.push_back(val);
        wp_x.push_back(std::stod(tokens[0]));
        wp_y.push_back(std::stod(tokens[1]));
        wp_speed.push_back(std::stod(tokens[3])); // "speed"列
    }

    // リファレンスパス生成
    ReferencePath ref_path(wp_x, wp_y, 0.2, 5, 1.5, false);

    // 速度プロファイル補間
    std::vector<double> wp_speed_interp(ref_path.size());
    for (size_t i = 0; i < ref_path.size(); ++i) {
        double ratio = static_cast<double>(i) * (wp_speed.size() - 1) / (ref_path.size() - 1);
        size_t idx = static_cast<size_t>(std::floor(ratio));
        double frac = ratio - idx;
        if (idx + 1 < wp_speed.size()) {
            wp_speed_interp[i] = (1 - frac) * wp_speed[idx] + frac * wp_speed[idx + 1];
        } else {
            wp_speed_interp[i] = wp_speed.back();
        }
    }
    ref_path.set_speed_profile(wp_speed_interp);

    // 車両モデル初期化
    BicycleModel car(std::make_shared<ReferencePath>(ref_path), 1.2, 0.8, 0.05);

    // コントローラ設定
    int N = 30;
    Eigen::SparseMatrix<double> Q(3, 3), R(2, 2), QN(3, 3);
    Q.setIdentity(); Q.coeffRef(0, 0) = 1.0;
    R.setIdentity(); R.coeffRef(0, 0) = 0.5;
    QN.setIdentity(); QN.coeffRef(0, 0) = 1.0;

    double v_max = 35.0 / 3.6;
    double delta_max = 0.66;
    double ay_max = 5.0;
    Eigen::VectorXd umin(2), umax(2);
    umin << 0.0, -std::tan(delta_max) / car.length;
    umax << v_max, std::tan(delta_max) / car.length;
    Eigen::VectorXd xmin = Eigen::VectorXd::Constant(3, -INFINITY);
    Eigen::VectorXd xmax = Eigen::VectorXd::Constant(3, INFINITY);

    StateConstraints state_constraints(xmin, xmax);
    InputConstraints input_constraints(umin, umax);

    MPC mpc(std::make_shared<BicycleModel>(car),
            N, Q, R, QN,
            state_constraints, input_constraints,
            ay_max);




    // シミュレーションループ
    double t = 0.0;
    const int steps = 300;
    std::ofstream log("mpc_log.csv");
    log << "time,x,y,s,e_y,e_psi,v,delta,ref_x,ref_y\n";

    for (int i = 0; i < steps; ++i) {
        Eigen::Vector2d u = mpc.get_control();
        car.drive(u);
        auto wp = ref_path.get_waypoint(car.s);

        log << t << "," << car.temporal_state.x << "," << car.temporal_state.y << ","
            << car.s << "," << car.spatial_state.e_y << "," << car.spatial_state.e_psi << ","
            << u[0] << "," << u[1] << "," << wp->x << "," << wp->y << "\n";

        t += car.Ts;
    }

    std::cout << "✅ mpc_log.csv にログを保存しました。" << std::endl;
    return 0;
}