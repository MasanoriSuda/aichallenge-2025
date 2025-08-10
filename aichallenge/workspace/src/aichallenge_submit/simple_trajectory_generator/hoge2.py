import pandas as pd
import numpy as np

# 入力ファイル名
input_file = "your_input.csv"
output_file = "straight_line_from_input.csv"

# 読み込み
df = pd.read_csv(input_file)

# --- 📐 平均方向ベクトル（先頭20点の差分ベクトルの平均） ---
# x[i+1] - x[i] のような差分ベクトルを 0〜18 番までとって平均
diff_x = df['x'].values[1:20] - df['x'].values[0:19]
diff_y = df['y'].values[1:20] - df['y'].values[0:19]
avg_dx = np.mean(diff_x)
avg_dy = np.mean(diff_y)
direction = np.array([avg_dx, avg_dy])
direction_unit = direction / np.linalg.norm(direction)  # 単位ベクトル

# ステップ数と平均ステップ長
N = 1000
ds = np.mean(np.sqrt(diff_x**2 + diff_y**2))  # 平均ステップ長（≒1m未満）

# 延長された x, y 座標（起点は点0）
x0, y0 = df.loc[0, ['x', 'y']]
x_new = x0 + direction_unit[0] * ds * np.arange(N)
y_new = y0 + direction_unit[1] * ds * np.arange(N)

# その他の列（クォータニオンなど）は1行目をコピー
z_new = np.zeros(N)
speed_new = np.full(N, 9.72)
x_quat = np.full(N, df.loc[0, 'x_quat'])
y_quat = np.full(N, df.loc[0, 'y_quat'])
z_quat = np.full(N, df.loc[0, 'z_quat'])
w_quat = np.full(N, df.loc[0, 'w_quat'])

# DataFrame化
df_new = pd.DataFrame({
    'x': x_new,
    'y': y_new,
    'z': z_new,
    'x_quat': x_quat,
    'y_quat': y_quat,
    'z_quat': z_quat,
    'w_quat': w_quat,
    'speed': speed_new
})

# 保存
df_new.to_csv(output_file, index=False)
print(f"✅ 書き出し完了: {output_file}")
