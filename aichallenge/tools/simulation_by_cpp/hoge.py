import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("mpc_log.csv")

plt.figure(figsize=(12, 8))

# XY軌跡
plt.subplot(2, 2, 1)
plt.plot(df["x"], df["y"])
plt.title("Trajectory (x, y)")
plt.axis("equal")

# 横ズレ
plt.subplot(2, 2, 2)
plt.plot(df["step"], df["e_y"])
plt.title("Lateral Error e_y")

# 姿勢誤差
plt.subplot(2, 2, 3)
plt.plot(df["step"], df["e_yaw"])
plt.title("Yaw Error e_yaw")

# ステア角
plt.subplot(2, 2, 4)
plt.plot(df["step"], df["delta"])
plt.title("Steering delta")

plt.tight_layout()
plt.show()
