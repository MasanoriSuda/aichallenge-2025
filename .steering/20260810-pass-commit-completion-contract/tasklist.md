# Tasklist

- [x] 直近ログの早期 Return と prediction-loss Recovery をコードへ照合する
- [x] rear-clear 前の margin-only Pass hold を実装する
- [x] SafeSeparation を commit-stage aware にする
- [x] SideBySide current-overlap debounce を同側継続へ配線する
- [x] 回帰単体テストを追加する
- [x] `test_v2x_overtake_core` を含む package test を実行する
- [x] `make autoware-build` を実行する

## Definition of Done

- `rear_clear=false` の margin-only event で Return しない。
- 同じ条件で physical wall contact があれば Recovery する。
- `SideBySideCommitted` の SafeSeparation は RecoverBehind を返さない。
- `ShiftCommitted` の従来 RecoverBehind は維持する。
- current overlap 未確認期間のみ同側 SafeSeparationへ移行可能で、confirmed overlapは
  緩和しない。
- 対象テストと package build が成功する。

## 実走確認項目

- `ReturnBeforeWallMarginRecovery` のログは `rear_clear=1` の場合だけ出る。
- `HoldPassForRearClearBeforeWallMarginRecovery` 後に Pass を維持し、rear-clear後Returnする。
- `opp_stage=side_by_side_committed` 後の RecoverBehind / prediction-loss Recovery が0回。
- wall physical contact、solver failure、Reverseが増えていない。

## 静的検証結果

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  940 tests、0 errors、0 failures、0 skipped
