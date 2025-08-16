import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込み（ファイル名を適宜変更してください）
df = pd.read_csv("raceline_awsim_30km_from_garage.csv")

# xとyの値を抽出
x = df['x']
y = df['y']

# インデックスを取得
index = df.index

# プロット（x軸: index, y軸: xとy）
plt.figure(figsize=(12, 6))

# xプロット
plt.plot(index, x, label='x', marker='o', linestyle='-', alpha=0.7)

# yプロット
plt.plot(index, y, label='y', marker='s', linestyle='-', alpha=0.7)

plt.title("Index vs x and y values")
plt.xlabel("Index")
plt.ylabel("Position Values")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
