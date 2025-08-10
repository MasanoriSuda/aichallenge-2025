import pandas as pd
import numpy as np

# CSVファイルを読み込む
input_file = 'raceline_awsim_15km copy.csv'  # 実際のCSVファイルのパスを指定
df = pd.read_csv(input_file)

# 曲率計算関数
def calculate_curvature(x1, y1, x2, y2, x3, y3):
    # 曲率を計算するために3点を使用
    return 2 * (x2 - x1) * (y3 - y2) - (x3 - x2) * (y2 - y1) / ((x2 - x1)**2 + (y2 - y1)**2)**1.5

# 曲率が0.1以下の部分を直線化
threshold = 0.1
straight_path = []

# 最初の2点はそのまま追加（曲率計算のために前後3点が必要）
straight_path.append([df['x'][0], df['y'][0], df['z'][0], df['x_quat'][0], df['y_quat'][0], df['z_quat'][0], df['w_quat'][0], df['speed'][0]])

for i in range(1, len(df)-1):
    # 3点を使って曲率を計算
    x1, y1 = df['x'][i-1], df['y'][i-1]
    x2, y2 = df['x'][i], df['y'][i]
    x3, y3 = df['x'][i+1], df['y'][i+1]
    
    kappa = calculate_curvature(x1, y1, x2, y2, x3, y3)
    
    if abs(kappa) <= threshold:
        # 曲率が小さい場合は直線化（2点を結ぶ）
        straight_path.append([df['x'][i], df['y'][i], df['z'][i], df['x_quat'][i], df['y_quat'][i], df['z_quat'][i], df['w_quat'][i], df['speed'][i]])
    else:
        # 曲率が大きい場合はそのまま
        straight_path.append([df['x'][i], df['y'][i], df['z'][i], df['x_quat'][i], df['y_quat'][i], df['z_quat'][i], df['w_quat'][i], df['speed'][i]])

# 最後の行を取得
straight_path.append([df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], 
                      df['x_quat'].iloc[-1], df['y_quat'].iloc[-1], 
                      df['z_quat'].iloc[-1], df['w_quat'].iloc[-1], 
                      df['speed'].iloc[-1]])

# 結果をDataFrameに変換
straight_path_df = pd.DataFrame(straight_path, columns=['x', 'y', 'z', 'x_quat', 'y_quat', 'z_quat', 'w_quat', 'speed'])

# 新しいCSVとして保存
output_file = 'raceline_awsim_15km.csv'  # 出力するファイル名
straight_path_df.to_csv(output_file, index=False)

# 結果を表示
print(straight_path_df)
