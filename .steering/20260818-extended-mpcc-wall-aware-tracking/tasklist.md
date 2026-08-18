# Tasklist

- [x] 20260818-192235と直前runを比較する
- [x] 既存の物理再検証・last-feasible保持を確認する
- [x] wall-aware soft referenceを純粋関数として追加する
- [x] Extended MPCCの横参照・重みに適用する
- [x] 設定とデバッグログを追加する
- [x] 単体テストを追加する
- [x] 対象テストを実行する
- [x] packageをビルドする
- [x] 変更範囲を確認してコミットする

## 動的確認

- `Extended MPCC wall-aware tracking` のadjusted/minimum_reserve/minimum_weight_scale
- `Pass -> Return -> Idle` 完遂数
- `optimized horizon escaped wall bounds`
- `optimized horizon failed physical revalidation`
- `actual footprint wall margin violated`
- `solution hard wall contact`

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 1320 tests、0 errors、0 failures
