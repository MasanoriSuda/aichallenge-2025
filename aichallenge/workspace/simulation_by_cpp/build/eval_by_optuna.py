import optuna
import subprocess
import json

# Studyの保存先（同じ名前で再開できる！）
storage = "sqlite:///tune.db"
study_name = "mpc_tuning_1000_trials"

def objective(trial):
    Q_e_y   = trial.suggest_float("Q_e_y",    1.0,  50.0)
    Q_e_yaw = trial.suggest_float("Q_e_yaw",  1.0, 100.0)
    Q_t     = trial.suggest_float("Q_t",      0.1,  10.0)
    R_delta = trial.suggest_float("R_delta",  0.01,  5.0)
    N       = trial.suggest_int("N",          5,    30)

    # config.json書き出し
    config = {
        "Q": [Q_e_y, Q_e_yaw, Q_t],
        "R": [R_delta],
        "N": N
    }
    with open("config.json", "w") as f:
        json.dump(config, f)

    # シミュレーション実行
    result = subprocess.run(["./test_exec"], capture_output=True, text=True)

    # 評価指標の取得（標準出力の最後の行にスコアを書くなど）
    try:
        last_line = result.stdout.strip().split("\n")[-1]
        score = float(last_line)  # ← 終端にスコアを出力する形式にしておく
    except Exception as e:
        print("Error:", e)
        print(result.stdout)
        return float("inf")  # 失敗したら最大コスト

    return score  # 例：コスト（小さいほど良い）

# Study作成
study = optuna.create_study(
    study_name=study_name,
    storage=storage,
    direction="minimize",
    load_if_exists=True
)

# 💥ここで1000回ぶん回す！
study.optimize(objective, n_trials=1000)

# ベスト結果保存
with open("best_config.json", "w") as f:
    best = study.best_params
    best_json = {
        "Q": [best["Q_e_y"], best["Q_e_yaw"], best["Q_t"]],
        "R": [best["R_delta"]],
        "N": best["N"]
    }
    json.dump(best_json, f, indent=2)

print("Best Score:", study.best_value)
print("Best Params:", best)
