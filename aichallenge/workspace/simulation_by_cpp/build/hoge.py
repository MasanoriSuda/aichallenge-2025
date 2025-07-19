import pandas as pd
import matplotlib.pyplot as plt

# CSV読み込み
df_ref = pd.read_csv("raceline_awsim_15km.csv")
df_mpc = pd.read_csv("mpc_log.csv")

# プロット準備
plt.figure(figsize=(12, 10))

# --- 1. 軌道比較 (x vs y) ---
plt.subplot(2, 2, 1)
plt.plot(df_ref["x"], df_ref["y"], label="Reference Trajectory", linewidth=2)
plt.plot(df_mpc["x"], df_mpc["y"], label="MPC Output", linestyle='--')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory Comparison")
plt.legend()
plt.axis("equal")
plt.grid()

# --- 2. ステア角比較 (delta vs delta_ref) ---
plt.subplot(2, 2, 2)
plt.plot(df_mpc["step"], df_mpc["delta"], label="MPC delta")
plt.plot(df_mpc["step"], df_mpc["delta_ref"], label="Reference delta", linestyle='--')
plt.xlabel("Step")
plt.ylabel("Steering Angle [rad]")
plt.title("Steering Angle vs Reference")
plt.legend()
plt.grid()

# --- 3. 横ずれ誤差 (e_y) ---
plt.subplot(2, 2, 3)
plt.plot(df_mpc["step"], df_mpc["e_y"])
plt.xlabel("Step")
plt.ylabel("Lateral Error e_y [m]")
plt.title("Lateral Error (e_y)")
plt.grid()

# --- 4. ヨー誤差 (e_yaw) ---
plt.subplot(2, 2, 4)
plt.plot(df_mpc["step"], df_mpc["e_yaw"])
plt.xlabel("Step")
plt.ylabel("Heading Error e_yaw [rad]")
plt.title("Heading Error (e_yaw)")
plt.grid()

# 描画
plt.tight_layout()
plt.suptitle("MPC Tracking Report", fontsize=16, y=1.02)
plt.show()

