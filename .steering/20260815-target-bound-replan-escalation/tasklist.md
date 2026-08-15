# Tasklist

- [x] `20260815-154439`の遷移と失敗理由を集計する
- [x] target-ahead continuationへ近距離guard分類を追加する
- [x] SafeSeparationへ最大closing speedを受け渡す
- [x] target-bound hold開始時に左右再評価を即時要求する
- [x] hold中の同側／no-return前反対側置換を許可する
- [x] config / cloud configへ同一設定を追加する
- [x] 単体テストを追加する
- [x] package test / buildを実行する
- [x] 実走確認項目を記録する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
  - 25 test targets
  - `colcon test-result`: 1168 tests, 0 errors, 0 failures
- `git diff --check`: 成功

## 次回実走で見るログ

- `target-bound execution hold started ... tactical_replan=immediate`
- `target-bound escalation accepted fresh same-side Mission`
- `target_guard=1`時に`signed_closing<=0.2`
- target-bound hold中、2 mより前方でのsame/cross-side Mission置換
- `optimized horizon escaped target separation bounds`と
  `physical target separation conflicts with wall bounds`の回数
- `Pass -> Return`のうちrear-clear正常完遂数
