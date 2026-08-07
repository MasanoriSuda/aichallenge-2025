# Tasklist

- [x] 最新ログと現行コードを照合する
- [x] requirements/design を作成する
- [x] latch 済み forward completion を予測重複確認対象へ追加する
- [x] 未確認中だけ forward completion を保持する
- [x] pure policy test を追加・更新する
- [x] format/build/test を実行する
- [x] 実走確認項目を記録する

## 実走確認項目

- `latched=1` 中に `footprint_sweep_clear=0` が一周期出ても即 `Recovery` へ落ちないこと
- 予測重複が 0.25 秒未満で解消した場合、`Pass -> Return -> Idle` へ完遂すること
- 予測重複が 0.25 秒以上継続した場合、`SafeSeparation abort` が発生すること
- 現在車体の確定重複、壁異常、EmergencyBrake、solver recovery は即時中止されること

## Verification

- `make autoware-build`: 25 packages build successful
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets、900 tests、0 failures
- `colcon test-result --verbose`: 対象packageは成功。既存の`build/joycon_contract_guard/package.xml`欠損警告あり
