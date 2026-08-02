# タスクリスト

- [x] Pass中のBehavior降格経路を整理する
- [x] committed Pass ownerの純粋判定を追加する
- [x] Behavior最終状態へownerを適用する
- [x] 既存ログへowner状態を追加する
- [x] 単体テストを追加する
- [x] `make autoware-build`を確認する
- [x] `multi_purpose_mpc_ros`のテストを確認する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）

## 動的確認項目

- `pass_owner=1`中の`Overtake -> Follow`回数
- Pass -> Return完遂数
- front-cap再適用時間
- wall / live corridor / solver Recovery回数
- SafetyBrake、接触、異常周の増減
