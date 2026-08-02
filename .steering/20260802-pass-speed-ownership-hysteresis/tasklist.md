# タスクリスト

- [x] 最新dev2ログとPass所有権条件を照合する
- [x] minimum-motion releaseをBehavior ownerへ伝播する
- [x] current footprint overlap確認タイマーを追加する
- [x] front-cap・front-danger・Behavior ownerで同じ猶予を使用する
- [x] 既存ログへ確認状態を追加する
- [x] 単体テストを追加する
- [x] `make autoware-build`を確認する
- [x] `multi_purpose_mpc_ros`のテストを確認する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）
- `colcon test-result --verbose`: 825 tests、0 errors、0 failures、0 skipped
- 既存build treeの`joycon_contract_guard/package.xml`欠損を読み飛ばす警告あり。対象packageの失敗ではない。

## 動的確認項目

- ShiftOut -> Pass数
- Pass -> Return -> Idle完遂数
- `pass_owner=0`へ落ちた理由と回数
- front-cap再適用・解除の反復回数
- current-overlap猶予の最大継続時間
- SafetyBrake / FollowPrepare / Recovery回数
- 接触、壁、異常周の増減
