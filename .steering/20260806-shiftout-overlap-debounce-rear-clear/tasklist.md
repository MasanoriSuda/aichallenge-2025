# Tasklist

- [x] 最新runと現行policyを照合する
- [x] requirements/designを記録する
- [x] ShiftOut predicted-overlap confirmationを接続する
- [x] rear-clear後のtarget lossをReturnへ接続する
- [x] core unit testを追加・更新する
- [x] 対象testを実行する
- [x] `make autoware-build` を実行する
- [x] `make dev2` の動的確認項目を記録する

## 動的確認項目

- 同一ShiftOut内の `front cap: Reapplied/Released` 反復回数
- predicted overlapが0.25秒未満で解消したとき、cap再適用が発生しないこと
- sustained overlapではcap再適用またはhard guardが働くこと
- targetが後方へ抜けた後の `ShiftOut -> Recovery, reason=locked target stale or lost` が0件になること
- `ShiftOut/Pass -> Return -> Idle` の完遂数

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25 CTest targets）
- `colcon test-result --verbose`: 896 tests、0 errors、0 failures、0 skipped
- `colcon test-result` は別packageの既存stale artifact
  `build/joycon_contract_guard/package.xml` 欠損を警告したが、今回対象の試験結果に失敗はない
- `make dev2`: 未実施。上記の動的確認項目を次回走行ログで照合する
