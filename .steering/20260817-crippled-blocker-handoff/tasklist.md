# Tasklist

- [x] 最新ログから制御権喪失箇所を特定する
- [x] OvertakeLine ownership に validated immediate Mission を追加する
- [x] Recovery handoff に validated immediate Mission を追加する
- [x] 単体テストを追加する
- [x] build / test を実行する
- [x] 試走時の確認項目を記録する

## Verification

- `make autoware-build`: success (25 packages)
- `test_v2x_overtake_core`: 686 tests passed

## Trial checks

- `validated Mission immediate entry` の直後に `OvertakeLine: Idle -> ShiftOut` が出ること
- 同じ周期付近で handoff log の `immediate=1` が出ること
- `LowSpeedRejoin skipped` 後に同じ target の ShiftOut / Pass が継続すること
- 完全 Mission がない場合に SafetyBrake / Recovery の hard guard が維持されること
- wall contact、solver recovery、collision worsening が増えないこと
