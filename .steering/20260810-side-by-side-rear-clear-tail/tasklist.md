# Tasklist

- [x] 最新ログとshort-horizon判定を照合する
- [x] 要求・設計を文書化する
- [x] rearward physical completion guardを実装する
- [x] local budget到達後のbounded rear-clear tailを実装する
- [x] 状態ログを追加する
- [x] core回帰テストを追加する
- [x] 対象package testを実行する
- [x] `make autoware-build`を実行する

## Definition of Done

- SideBySide、target_s <= 0、body/prediction/corridor clearでは、rollout budget missだけで
  `ShortHorizonUnsafe`にならない。
- 同条件ではlocal time／distance limit後もabsolute limit内で同じsideを維持する。
- absolute limit、physical wall fault、confirmed overlap、prediction/corridor不成立では緩和しない。
- Selectable / ShiftCommittedの既存挙動を維持する。
- 対象単体テストとpackage buildが成功する。

## 実走確認項目

- `SideBySide rear-clear tail active`から`Pass -> Return -> Idle`へ進む。
- 同イベントで`short horizon unsafe`、RecoverBehind、Recoveryが発生しない。
- `target_s <= 0`後の最低速度とrear-clear所要時間を確認する。
- wall contact、confirmed overlap、solver failure、Reverseが増えていない。

## 静的検証結果

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  942 tests、0 errors、0 failures、0 skipped
