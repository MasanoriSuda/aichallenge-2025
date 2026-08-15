# Tasklist

- [x] 検証済みMission即時entryのpure functionと単体テスト
- [x] 新規設定を通常・cloud configへ追加
- [x] Behavior entry admissionへ統合
- [x] 検証済みentry reserveによる初動closing ownershipを統合
- [x] デバッグログへ即時entry理由を追加
- [x] `make autoware-build`
- [x] package test / test-result
- [x] 差分確認・コミット

## Definition of Done

- 実相対速度未成立でも、現在の検証済みMissionとhard guardが成立すれば15 m以内でShiftOutできる。
- 未検証Mission、距離予算不足、body-clear不能、hard guard不成立では即時entryしない。
- 検証済みentry reserveを持つShiftOutはMission closingを一段目のadaptive capで縮めない。
- 既存テストを壊さず、追加テストが通る。

## Verification

- `make autoware-build`: 25 packages successful
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets passed
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1158 tests, 0 errors, 0 failures, 0 skipped
