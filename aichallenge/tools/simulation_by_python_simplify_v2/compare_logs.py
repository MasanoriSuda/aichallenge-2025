import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# ファイル読み込み
ref = pd.read_csv("raceline_awsim_15km_py.csv")
mpc = pd.read_csv("mpc_log.csv")

# 期待軌道からKDTreeを作る
tree = KDTree(ref[['x', 'y']].values)

# 各MPC点について最近傍のリファレンス点を検索
distances, indices = tree.query(mpc[['x', 'y']].values)

# 対応する参照データ取得
ref_matched = ref.iloc[indices].reset_index(drop=True)

# 誤差計算
mpc['ref_x'] = ref_matched['x']
mpc['ref_y'] = ref_matched['y']
mpc['ref_yaw'] = ref_matched['yaw']
mpc['ref_speed'] = ref_matched['speed']
mpc['ref_kappa'] = ref_matched['kappa']

# 各誤差
mpc['xy_error'] = distances
mpc['yaw_error'] = np.abs(np.unwrap(mpc['ref_yaw']) - np.unwrap(np.arctan2(np.sin(mpc['e_psi']), np.cos(mpc['e_psi'])) + mpc['ref_yaw']))
mpc['speed_error'] = np.abs(mpc['v'] - mpc['ref_speed'])

# RMSE計算
rmse_xy = np.sqrt(np.mean(mpc['xy_error'] ** 2))
rmse_yaw = np.sqrt(np.mean(mpc['yaw_error'] ** 2))
rmse_speed = np.sqrt(np.mean(mpc['speed_error'] ** 2))

print(f"✅ RMSE (xy): {rmse_xy:.3f} m")
print(f"✅ RMSE (yaw): {rmse_yaw:.3f} rad")
print(f"✅ RMSE (speed): {rmse_speed:.3f} m/s")

# グラフ表示
plt.figure(figsize=(10, 6))
plt.subplot(3,1,1)
plt.plot(mpc['s'], mpc['xy_error'], label='xy error')
plt.ylabel("xy [m]")
plt.grid()

plt.subplot(3,1,2)
plt.plot(mpc['s'], mpc['yaw_error'], label='yaw error')
plt.ylabel("yaw [rad]")
plt.grid()

plt.subplot(3,1,3)
plt.plot(mpc['s'], mpc['speed_error'], label='speed error')
plt.ylabel("speed [m/s]")
plt.xlabel("s [m]")
plt.grid()

plt.tight_layout()
plt.show()
