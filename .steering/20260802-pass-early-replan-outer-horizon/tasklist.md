# Tasklist

## Steering

- [x] 最新runのPass遷移とextension失敗理由を機械集計する
- [x] 計画時と実行時の曲率・横加速度上限不整合を特定する
- [x] requirements/design/tasklistを作成する

## Core

- [x] rear-clear距離によるearly replanトリガーを追加する
- [x] outer strategyの全区間連続性評価を追加する
- [x] 純粋関数の単体テストを追加する

## ROS adapter

- [x] candidate rolloutへoffset曲率speed capsを適用する
- [x] outer strategyをmission stateへ保存する
- [x] same-side extensionをrear-clear必要距離まで拡張する
- [x] extension時もouter continuityとfull pathを再検証する
- [x] 状態変化ログへearly/outer判定を追加する

## Verification

- [x] `test_v2x_overtake_core`
- [x] `git diff --check`
- [x] Release build
- [x] `make autoware-build`
- [x] ROS interface差分なしを確認する

## Dynamic verification

- [ ] `make dev2` 6周以上
- [ ] early rear-clear replan要求数
- [ ] same-side extension成功/失敗数と必要距離
- [ ] outer role reversal候補の棄却数と地点
- [ ] `Pass -> Return -> Idle` 完遂率
- [ ] lateral-accel / wall / horizon Recovery数
- [ ] SafetyBrakeとReverse要求数

## Verification results

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25 test targets、845 tests、失敗0
- 最終build後 `ctest -R '^test_v2x_overtake_core$'`: 成功
- topic/service/message/launch/result schema差分なし
- `aichallenge/result-summary.json` の既存変更は未編集
