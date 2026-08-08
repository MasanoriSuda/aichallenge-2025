# Tasklist

- [x] 最新runのPass遷移・速度・終了理由を集計する
- [x] requirements/designを記録する
- [x] forward completionの速度参照解放を実装する
- [x] Mission整合SafeSeparation budgetを実装する
- [x] recoverable side contact resolverとMission保持を実装する
- [x] Contact Continuationの横分離biasとログを追加する
- [x] core unit testを追加する
- [x] `docs/spec/mpc-integration.md`を更新する
- [x] package testとbuildを実行する
- [x] `git diff --check`とinterface境界を確認する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 396 tests passed
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（894 tests、0 failures）
- ROS 2 topic / service / message、Domain、評価結果schema、提出物構造の変更なし
- `make dev2`による動的効果確認は未実施

## 動的確認項目

- SafeSeparation中の`v_ref`が前車速度+2.0 m/sへ固定されず、通常コース参照まで伸びる
- `Pass -> Return -> Idle`が0/13から増える
- `short horizon unsafe`と`local distance limit`が減る
- 接触時に`contact_continue=1`となり、同じtarget/side/Mission generationを保持する
- 接触が0.8秒を超える、前進しない、壁へ接触する場合は従来どおり離脱する
- SafetyBrake、Recovery、Reverse、壁接触、競争停止が増えない
