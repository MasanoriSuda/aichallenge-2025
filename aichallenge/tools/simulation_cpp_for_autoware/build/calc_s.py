import pandas as pd
import numpy as np

def compute_cumulative_s(csv_input_path: str, csv_output_path: str):
    # CSV読み込み
    df = pd.read_csv(csv_input_path)

    # 差分距離を計算
    dx = df['x'].diff().fillna(0)
    dy = df['y'].diff().fillna(0)
    ds = np.sqrt(dx**2 + dy**2)

    # 累積距離を計算
    df['s'] = ds.cumsum()

    # カラムの順番変更（お好みで）
    df = df[['x', 'y', 'yaw', 'speed', 'kappa', 's']]

    # CSV出力
    df.to_csv(csv_output_path, index=False)
    print(f"✅ 書き出し完了: {csv_output_path}")

# 使い方
if __name__ == "__main__":
    input_csv = "raceline_awsim_15km_py_org.csv"    # ←元の軌道CSV（x,y,yaw,speed,kappa）
    output_csv = "raceline_awsim_15km_py.csv"  # ←累積s付き出力
    compute_cumulative_s(input_csv, output_csv)
