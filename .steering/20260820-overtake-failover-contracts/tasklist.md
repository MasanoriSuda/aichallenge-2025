# Tasklist

- [x] 直近試走ログと実行経路を照合する
- [x] solver bounded continuation判定と単体試験を追加する
- [x] 最終制御sourceとchange-aware traceへ接続する
- [x] DynamicWait遷移を同周期の横authorityへ接続する
- [x] runtime Mission replacement共通契約と拒否ログを追加する
- [x] package test/buildを実行する
- [x] 差分レビュー後にコミットする

## Validation

- `make autoware-build`: 成功（25 packages）
- `test_mpc_velocity_limit`: 10/10 成功
- `test_overtake_execution_orchestrator`: 17/17 成功

## 次回試走で確認するログ

- 単発solver失敗時に
  `control_source=solver-bounded-continuation` が出て、
  `solver-fallback-forced-stop` の瞬間的な -3.0 m/s2 指令が消えること。
- 危険条件または連続失敗時は
  `solver-fallback-forced-stop/continuation-<reason>` へ縮退すること。
- DynamicWait entry周期で `dynamic-wait-without-lateral` が0件であること。
- Mission置換拒否時に
  `runtime replacement contract rejected` と、予測期限・target余裕・壁余裕が
  一行で記録されること。

## Definition of Done

- 単発の安全なdynamic escape solver失敗は強制停止にならない。
- safety条件不成立または継続失敗ではforced stopを維持する。
- DynamicWait entry周期に横authorityが存在する。
- same-sideを含む全runtime Mission置換が同じtarget/wall/freshness契約を通る。
- 既存ROS 2/評価インターフェースを変更しない。
