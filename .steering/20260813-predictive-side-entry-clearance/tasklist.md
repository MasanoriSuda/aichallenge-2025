# Tasklist

- [x] 最新走行の失敗経路と現行候補順位を照合する
- [x] 動的 body-clear margin resolver を追加する
- [x] rollout に Pass 中の最小対車 surface clearance を追加する
- [x] Mission 候補の interaction reserve 順位を追加する
- [x] controller と 2 種類の config を更新する
- [x] startup / candidate debug log を更新する
- [x] core 単体テストを追加する
- [x] package build/test を実行する
- [x] 実施結果と実走確認項目を記録する

## Verification

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets成功
- `colcon test-result --verbose`: 1066 tests、0 errors、0 failures
- `git diff --check`: 成功
- ROS topic/service、評価JSON、提出物構造の変更なし

## 実走で見る項目

- 左右候補の `pass_target_clear` と選択 side
- `deadline_margin` と `target_intrusion_v`
- `opponent side opportunity pending/replaced` が actual overlap 前に出るか
- ContactContinuation / SafetyBrake / wall Recovery 回数
- ShiftOut -> Pass -> rear-clear の完遂率
