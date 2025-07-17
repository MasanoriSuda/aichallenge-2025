import pandas as pd
import matplotlib.pyplot as plt

# 元のracelineとMPCログを読み込み
df_ref = pd.read_csv("raceline_awsim_15km_py.csv")
df_mpc = pd.read_csv("mpc_log.csv")

# グラフ描画
plt.figure()
plt.plot(df_ref["x"], df_ref["y"], label="Reference Trajectory (raceline)", linewidth=2)
plt.plot(df_mpc["x"], df_mpc["y"], label="MPC Output", linestyle='--')
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.legend()
plt.title("MPC Output vs Reference Trajectory")
plt.grid()
plt.show()
