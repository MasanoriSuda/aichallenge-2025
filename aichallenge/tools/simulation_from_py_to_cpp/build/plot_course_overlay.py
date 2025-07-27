import pandas as pd
import matplotlib.pyplot as plt

# CSV読み込み
ref = pd.read_csv("raceline_awsim_15km_py.csv")
mpc = pd.read_csv("mpc_log.csv")

# プロット準備
plt.figure(figsize=(10, 10))
plt.plot(ref["x"], ref["y"], label="Reference Raceline", color='green', linewidth=2)
plt.plot(mpc["x"], mpc["y"], label="MPC Trajectory", color='red', linestyle='--', linewidth=1)

# 描画調整
plt.axis("equal")
plt.title("Raceline vs MPC Trajectory")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
