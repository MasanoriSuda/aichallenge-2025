# Tasklist

- [x] 最新ログからabort条件を特定
- [x] rearward measured-progress completionをcoreへ追加
- [x] controllerから厳格な成立条件を渡す
- [x] unit test追加
- [x] `make autoware-build`
- [x] test結果を記録

## Definition of Done

- 対象が後方かつ実測進捗がfreshなら、予測重複だけでShortHorizonUnsafeにならない。
- 進捗停止、現在車体重複、corridor block、hard guard faultでは継続しない。
- 既存テストを壊さない。

## Verification

- `make autoware-build`: 成功（25 packages）
- `/aichallenge/workspace` で `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 978 tests、0 errors、0 failures、0 skipped
- 既存の `build/joycon_contract_guard/package.xml` 欠損による集計skip警告あり（今回の変更外）
