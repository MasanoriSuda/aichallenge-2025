# Tasklist

- [x] 最新ログとside lock経路を照合
- [x] 要件・設計を記録
- [x] FollowPrepare評価でmission sideを正本にする
- [x] active mission中のcurve cooldown side解放を抑止する
- [x] FollowPrepare再開のmission side優先を実装する
- [x] 横離隔成立時の直接Pass復帰を実装する
- [x] 単体テストを追加・更新する
- [x] 対象テストを実行する
- [x] `make autoware-build`を実行する
- [x] `git diff --check`を実行する
- [x] 動的効果確認項目を記録する

## 試走判定項目

- `FollowPrepare -> ShiftOut/Pass`のsideが一時停止前と一致すること。
- `current_ey`と`goal_ey`がコース中央を挟んで反対符号になる再開がゼロであること。
- 同側corridorが不成立の周期は`FollowPrepare`を維持すること。
- 横離隔成立済みなら`FollowPrepare -> Pass`となること。
- 壁Recovery、SafetyBrake pause、Overtake完遂数を`20260801-013347`と比較すること。

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（715 tests、failure/error/skip すべて0）
- `git diff --check`: 成功
- `make dev2`による動的効果確認: 未実施（上記の試走判定項目で確認する）
