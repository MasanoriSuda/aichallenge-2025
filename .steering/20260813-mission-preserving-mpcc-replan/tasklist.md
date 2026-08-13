# Tasklist

- [x] 最新走行の中断理由を集計する
- [x] post-validationとSafeSeparationの実装経路を確認する
- [x] 候補速度別target予測の純粋関数を追加する
- [x] post-validationでtarget制約を候補速度別に再構築する
- [x] target境界失敗時の即時同側replanを追加する
- [x] SafeSeparation soft abortの即時同側replanを追加する
- [x] 単体テストを追加する
- [x] 対象packageをビルドする
- [x] 対象packageのテストを実行する
- [x] 動的試走の確認点を記録する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）
- 動的試走: 未実施（ユーザー実施予定）

## 動的確認（ユーザー実施）

- `optimized horizon escaped target separation bounds` のRecovery回数
- `rh ... speed_cap` が有限値となり、RecoveryではなくMission継続した回数
- `SafeSeparation soft abort same-side replan accepted` の回数
- ShiftOut開始数、Pass到達数、Return完遂数
- wall contact、EmergencyBrake、接触の増減
