
import optuna
import subprocess
import pandas as pd
import os

def objective(trial):
    # チューニング対象
    Q_e_y = trial.suggest_float("Q_e_y", 0.1, 50.0)
    Q_e_yaw = trial.suggest_float("Q_e_yaw", 0.1, 100.0)
    Q_t = trial.suggest_float("Q_t", 0.01, 5.0)
    R_delta = trial.suggest_float("R_delta", 0.01, 5.0)
    N = trial.suggest_int("N", 5, 25)

    # JSONファイルに書き出し
    config = {
        "Q": [Q_e_y, Q_e_yaw, Q_t],
        "R": [R_delta],
        "N": N
    }

    with open("config.json", "w") as f:
        import json
        json.dump(config, f)

    # 実行
    result = subprocess.run(["./test_exec"], capture_output=True)

    if result.returncode != 0:
        return 1e6  # 実行失敗は大きなペナルティ

    # mpc_log.csv 読み込み
    try:
        df = pd.read_csv("mpc_log.csv")
    except:
        return 1e6

    if len(df) < 100:
        return 1e5  # 破綻（短すぎ）

    # 指標
    e_y_mean = df["e_y"].abs().mean()
    e_yaw_mean = df["e_yaw"].abs().mean()
    t_final = df["t"].iloc[-1]

    # スコア（小さいほど良い）
    score = (
        e_y_mean * 5.0 +
        e_yaw_mean * 3.0 -
        t_final * 1.0
    )

    return score

if __name__ == "__main__":
    study = optuna.create_study(direction="minimize")
    study.optimize(objective, n_trials=50)

    print("Best Params:", study.best_params)