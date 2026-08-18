# Results

## Static validation

- `make autoware-build`: 成功（25 packages）
- `test_mpcc_progress`: 19 / 19 成功
- `test_v2x_overtake_core`: 707 / 707 成功
- `git diff --check`: 成功

実装した負荷制御:

- pre-arm 中は 1 cycle あたり既定 4 stage の物理壁 envelope を予熱する。
- active Mission 開始後 0.30 秒、または当該 problem 構築中に物理壁 cache miss が 1 件以上ある cycle は、RTI 第 2 solve を省略する。
- いずれも第 1 feasible QP を採用し、通常時の receding-horizon 更新周期、安全制約、戦術選択は変更しない。

## Dynamic validation

ユーザー試走待ち。次を比較する。

- MPC callback `overrun` 回数と最大時間
- `MPCC RTI-SQP` の `skip_cold_load` 回数
- `Overtake wall prewarm` の request / miss / stage 数
- Idle -> ShiftOut 直後 0.5 秒の callback 時間
- ShiftOut -> Pass -> Return -> Idle 完遂数
