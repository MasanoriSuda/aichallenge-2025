import optuna
import subprocess
import json

def objective(trial):
    params = {
        "Q_e_y": trial.suggest_float("Q_e_y",  5.0, 50.0),
        "Q_e_yaw": trial.suggest_float("Q_e_yaw", 10.0, 100.0),
        "Q_t": trial.suggest_float("Q_t", 0.01, 5.0),
        "R_delta": trial.suggest_float("R_delta", 0.01, 5.0),
        "N": trial.suggest_int("N", 5, 30)
    }

    # JSONファイルに保存
    with open("params.json", "w") as f:
        json.dump({
            "Q": [params["Q_e_y"], params["Q_e_yaw"], params["Q_t"]],
            "R": [params["R_delta"]],
            "N": params["N"]
        }, f)

    # C++のmain.cpp（test_exec）を実行
    result = subprocess.run(["./test_exec"], capture_output=True, text=True)

    try:
        # 最終行にスコアが出力されている想定（例: score: 123.45）
        for line in result.stdout.strip().split("\n"):
            if line.startswith("score:"):
                score = float(line.split(":")[1])
                return score
    except Exception as e:
        print("Failed to parse output:", e)
    
    return float("inf")  # 破綻時や出力ミス時

# --- Optuna 実行 ---
study = optuna.create_study(direction="minimize")
study.optimize(objective, n_trials=1000)

print("Best Score:", study.best_value)
print("Best Params:", study.best_params)
