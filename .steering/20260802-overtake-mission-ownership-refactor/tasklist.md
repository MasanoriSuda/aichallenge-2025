# タスクリスト

- [x] 最新ログから責務競合箇所を特定する
- [x] mission ownership resolver を追加する
- [x] controller の主要判定を resolver 結果へ置換する
- [x] 単体テストを追加する
- [x] `multi_purpose_mpc_ros` のビルドを確認する
- [x] `multi_purpose_mpc_ros` のテストを確認する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）

## Definition of Done

- 速度・距離・壁余裕などの設定値に差分がない
- resolver の全 phase 分岐がテストされている
- build/test 成功
- 次の性能修正が ownership 結果の切替として実装できる
