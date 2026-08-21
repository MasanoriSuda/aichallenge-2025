# Tasklist

- [x] `DynamicEscapeAttemptTracker`と終了理由をpure C++ coreへ追加
- [x] planner request欠落時のattempt即時破棄をtrackerへ置換
- [x] target loss grace設定をlocal/cloud configへ追加
- [x] race session resetとtactical snapshotをtrackerへ接続
- [x] lifecycleログとformat testを追加
- [x] request gap、target loss、retargetの単体テストを追加
- [x] package build/testを実行
- [x] 差分とインターフェース契約をレビュー
- [x] コミット

## Definition of Done

- 同一targetがrelevantな間、requestがfalse/trueを往復してもattempt IDが変わらない。
- target欠落はgrace後に明示的な`target-lost`で終了する。
- stale solutionのlease値は変更されていない。
- `/control/command/control_cmd`等のROS契約に差分がない。
- 対象packageのbuild/testが成功する。

## Verification

- `make autoware-build`: 25 packages successful。
- `colcon test --packages-select multi_purpose_mpc_ros`: 1481 tests、failure 0。
- `git diff --check`: errorなし。
- `pre-commit run --files ...`: hostに`pre-commit`がなく未実行。
- ROS topic、message、service、launch、result schemaの変更なし。
