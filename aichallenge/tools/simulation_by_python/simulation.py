from map import Map
from reference_path import ReferencePath
from spatial_bicycle_models import BicycleModel
import matplotlib.pyplot as plt
from MPC import MPC
from scipy import sparse
import numpy as np
import pandas as pd

if __name__ == '__main__':
    # CSV読み込み（ChatGPT変換済みのwaypoints_converted.csv）
    df = pd.read_csv("raceline_awsim_15km_py.csv")
    wp_x = df["x"].values
    wp_y = df["y"].values
    wp_speed = df["speed"].values

    # マップはダミー（画像使わない）
    dummy_map = Map(file_path=None, origin=[wp_x.min(), wp_y.min()], resolution=0.1)

    # リファレンスパス作成
    reference_path = ReferencePath(
        dummy_map,
        wp_x,
        wp_y,
        0.2,            # path_resolution
        5,              # smoothing_distance
        1.5,            # max_width
        False           # circular
    )

    # 補間してリファレンス速度にセット
    wp_speed_interp = np.interp(
        np.linspace(0, len(wp_speed) - 1, len(reference_path.waypoints)),
        np.arange(len(wp_speed)),
        wp_speed
    )
    reference_path.set_speed_profile(wp_speed_interp)



    # デバッグ確認
    print("wp_x len:", len(wp_x))
    print("wp_speed len:", len(wp_speed))
    print("reference_path.waypoints len:", len(reference_path.waypoints))
    print("v_profile length:", len(reference_path.v_profile))
    print("v_profile sample:", reference_path.v_profile[:5])
    print("v_profile contains NaN:", np.any(pd.isnull(reference_path.v_profile)))

    # モーションモデル初期化（実車パラメータに合わせて調整可）
    car = BicycleModel(length=1.2, width=0.8,
                       reference_path=reference_path, Ts=0.05)

    #################
    # コントローラ設定
    #################
    N = 30
    Q = sparse.diags([1.0, 0.0, 0.0])
    R = sparse.diags([0.5, 0.0])
    QN = sparse.diags([1.0, 0.0, 0.0])

    v_max = 35.0 / 3.6  # [km/h] → [m/s]
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

    #################
    # シミュレーション
    #################
    t = 0.0
    x_log = [car.temporal_state.x]
    y_log = [car.temporal_state.y]
    v_log = [0.0]

    while car.s < reference_path.length:
        u = mpc.get_control()
        car.drive(u)

        x_log.append(car.temporal_state.x)
        y_log.append(car.temporal_state.y)
        v_log.append(u[0])
        t += car.Ts

        reference_path.show()
        car.show()
        mpc.show_prediction()
        plt.title(f'MPC Simulation: v={u[0]:.2f}, delta={u[1]:.2f}, time={t:.2f}s')
        plt.axis('off')
        plt.pause(0.001)