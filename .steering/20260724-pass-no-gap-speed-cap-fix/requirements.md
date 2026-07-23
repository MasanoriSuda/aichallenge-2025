# Pass中 no-gap 速度制限修正 要件

## 目的

横離隔確認済みの committed Pass 中に、実行corridor不成立から生成された
`no_gap_target_velocity: 2.0 m/s` が残り、最大制動を発生させる問題を解消する。

## 対象

- `multi_purpose_mpc_ros` の gap planner 速度制限適用条件
- 適用条件を検証する純粋関数と単体テスト
- dev2 による修正前後の走行ログ比較

## 制約

- 通常の Follow における5 m車間制御は変更しない。
- ShiftOutおよび横離隔未確認のPassでは、既存のno-gap処理を維持する。
- front risk、EmergencyBrake、solver fail-safe、Recovery速度制限を変更しない。
- ROS 2 topic/service/message、launch、評価結果schemaを変更しない。
- 既存の未コミット変更を巻き戻さない。

## Definition of Done

- committed Passかつ横離隔確認済みのdiagnostic-only corridor bypass中は、
  planner由来の2.0 m/s no-gap速度制限を適用しない。
- それ以外の既存適用条件を単体テストで維持する。
- 対象パッケージのテストとビルドが成功する。
- dev2ログで、corridor bypass発生後に約2.0 m/sへ収束する急失速が再現しないことを確認する。

