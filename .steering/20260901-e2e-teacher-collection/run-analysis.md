# Teacher Run Analysis

## Run

- output: `output/20260901-024545`
- scenario: 1 vehicle / 0 NPC / fixed start / 3 laps
- controller: MPC (`control_method: mpc`をsystem/submitの両launch logで確認)
- AWSIM terminal state: `Finish`

## Runtime

MPCは起動直後、odometry受信前に1回だけmissing/stale failsafeを記録した。その後は
production commandを継続し、controller log上の2つの完全lapは58.713秒、57.688秒だった。
configured 3 lapsの最終crossingでAWSIMはFinishへ移ったが、controller内部lap loggerは
Lap 3行を出力しなかった。

## Bag and Dataset

- bag duration: 172.140868189 s
- `/sensing/lidar/scan`: 3,439 messages
- `/control/command/control_cmd`: 6,887 messages
- extracted samples: 3,439
- sync reject: 0
- sync delta: mean 0.008082 s / p95 0.012174 s / max 0.016504 s

このrunは教師収集経路の成立証拠であり、単独runだけで再学習を開始する根拠ではない。
run-level validationとclosed-loop perturbation coverageのため、次Sliceで複数teacher runと
dataset manifestを収集する。
