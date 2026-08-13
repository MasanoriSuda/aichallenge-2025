# Tasklist

- [x] 最新ログと現行 hard fallback 経路を照合する
- [x] 車間 lateral bound の段階縮退を純粋関数化する
- [x] live receding-horizon へ組み込む
- [x] hard failure の具体理由を Recovery ログへ伝播する
- [x] MPCC-lite shadow を 1 Hz にする
- [x] unit test を追加する
- [x] package test/build を実行する
- [x] 動的効果確認項目を記載する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets 成功、1060 tests / 0 failures
- 新規 bound 縮退テスト: robust維持、configured縮退、physical縮退、trust拡張、物理不成立の5ケース成功

## 次回 make dev2 で見る項目

- `receding_horizon_fallback_reason=robust target separation degraded ...` が Recovery ではなく ShiftOut/Pass 継続中に出ること
- `physical target separation conflicts with wall bounds` の回数
- `ShiftOut/Pass -> Recovery`、`Pass -> Return -> Idle` の回数
- shadow評価付近の `/control/command/control_cmd` 周期遅延
