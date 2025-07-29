#include "mpc_simulator.hpp"  // クラス定義（MPCSimulator）を含むヘッダを想定
#include <iostream>
#include <fstream>

int main() {
    MPCSimulator sim;

    const std::string csv_file = "raceline_awsim_15km_py.csv";
    const std::string odom_file = "odom.csv";

    if (!sim.init(csv_file, odom_file)) {
        std::cerr << "❌ 初期化に失敗しました。" << std::endl;
        return 1;
    }

    const int steps = 6000;
    double t = 0.0;
    const double dt = sim.get_dt();  // 車両モデルの Ts を取得する関数（例）

    for (int i = 0; i < steps; ++i) {
        sim.spin_once(t);
        t += dt;
    }

    sim.save_log("mpc_log.csv");

    std::cout << "✅ シミュレーション終了。ログを mpc_log.csv に保存しました。" << std::endl;
    return 0;
}
