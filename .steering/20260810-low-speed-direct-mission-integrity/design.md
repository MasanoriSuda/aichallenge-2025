# Design

## 問題

`plan_stopped_vehicle_local_path()` は自車より前方 (`s > 0`) の車両だけを経路生成対象にする。
そのため対象車が横並びへ移ると planner が非 active になり、既存実装は保持した横目標と
静的壁だけを確認して Pass を継続していた。また、live planner が反対側を返すと Pass 中でも
sideを変更していた。

## 方針

### 1. Pass side のcommit

純粋関数 `can_update_low_speed_direct_pass_side()` でside更新可否を決める。

- Shift: 反対側への変更可
- Pass/Rejoin: 現在sideと同じ更新だけ可
- 未設定side: 有効候補を採用可

Pass中に反対候補が現れた場合、全幅横断はせず現在sideを保持する。

### 2. target-aware retained Pass

開始時の `low_speed_stopped_candidate_id` を `low_speed_shift_target_vehicle_id_` に保存する。
V2X評価ループでそのIDだけについて、course frame上の以下を算出する。

- 現在 longitudinal / relative lateral
- 現在車体 footprint separation
- 観測横速度を用いた短時間予測
- 予測 footprint sweep separation
- pass side ordering

純粋関数 `resolve_low_speed_retained_pass_validation()` がreject理由を一意に返す。
静的経路が成立していても、対象車のidentity/continuity/dynamic footprintが不成立なら
retained Passとして扱わない。

### 3. ライフサイクル

race session reset、external recovery reset、MPC handoff完了で対象IDをclearする。
一時的に別車をnearestとして検出しても、保持対象IDをすり替えない。

## 影響範囲

- `multi_purpose_mpc_ros/v2x_overtake_core.hpp`
- `src/v2x_overtake_core.cpp`
- `src/mpc_controller_cpp.cpp`
- `test/test_v2x_overtake_core.cpp`

設定・launch・ROSインターフェースは変更しない。

## 実走確認項目

- `Low-speed direct control retained side` がPass中に反対側へ変わらない。
- `Low-speed retained Pass rejected` の理由別回数。
- front→side移行後にPassを維持し、rear-clear後にRejoinする割合。
- 接触、壁stop、Reverse、競争停止が増えていないこと。

