from map import Map
from reference_path import ReferencePath
from spatial_bicycle_models import BicycleModel
from MPC import MPC
from scipy import sparse
import numpy as np
import pandas as pd
import csv

if __name__ == '__main__':
    # CSV読み込み
    df = pd.read_csv("raceline_awsim_15km_py.csv")
    wp_x = df["x"].values
    wp_y = df["y"].values
    wp_speed = df["speed"].values

    # ダミーマップ（画像不要）
    dummy_map = Map(file_path=None, origin=[wp_x.min(), wp_y.min()], resolution=0.1)

    # リファレンスパス作成
    reference_path = ReferencePath(
        dummy_map,
        wp_x,
        wp_y,
        0.2,
        5,
        1.5,
        False
    )

    # 速度プロファイル補間
    wp_speed_interp = np.interp(
        np.linspace(0, len(wp_speed) - 1, len(reference_path.waypoints)),
        np.arange(len(wp_speed)),
        wp_speed
    )
    reference_path.set_speed_profile(wp_speed_interp)

    # 車両モデル初期化
    car = BicycleModel(length=1.2, width=0.8,
                       reference_path=reference_path, Ts=0.05)

    # コントローラ設定
    N = 30
    Q = sparse.diags([1.0, 0.0, 0.0])
    R = sparse.diags([0.5, 0.0])
    QN = sparse.diags([1.0, 0.0, 0.0])

    v_max = 35.0 / 3.6
    delta_max = 0.66
    ay_max = 5.0
    InputConstraints = {
        'umin': np.array([0.0, -np.tan(delta_max) / car.length]),
        'umax': np.array([v_max, np.tan(delta_max) / car.length])
    }
    StateConstraints = {
        'xmin': np.array([-np.inf, -np.inf, -np.inf]),
        'xmax': np.array([np.inf, np.inf, np.inf])
    }

    mpc = MPC(car, N, Q, R, QN, StateConstraints, InputConstraints, ay_max)

    # シミュレーションループ（固定ステップ数）
    t = 0.0
    steps = 300
    log = []

    for _ in range(steps):
        u = mpc.get_control()
        car.drive(u)
        wp = reference_path.get_waypoint(car.s)

        log.append([
            t,
            car.temporal_state.x,
            car.temporal_state.y,
            car.s,
            car.spatial_state.e_y,
            car.spatial_state.e_psi,
            u[0],
            u[1],
            wp.x,
            wp.y
        ])
        t += car.Ts

    # CSV出力
    with open("mpc_log.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "x", "y", "s", "e_y", "e_psi", "v", "delta", "ref_x", "ref_y"])
        writer.writerows(log)

    print("✅ mpc_log.csv にログを保存しました。")
