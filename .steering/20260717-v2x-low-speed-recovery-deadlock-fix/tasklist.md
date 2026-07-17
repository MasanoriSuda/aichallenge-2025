# V2X低速回避・回復方向デッドロック修正 Tasklist

作成日: 2026-07-17
状態: Experiment Complete / Acceptance Failed

## Definition of Done

- [x] LowSpeedAvoidance stall watchdogを実装する。
- [x] stall解除時にlocal targetをresetし、即再進入を抑制する。
- [x] 回復候補の固定をactuation開始まで遅延する。
- [x] clear footprintでwall分類に依存しないForward fallbackを評価する。
- [x] unit testとbuildを成功させる。
- [ ] dev3実験で3台恒久停止の再発有無を判定する。
- [x] 実験結果を本ステアリングと`docs/spec/mpc-integration.md`へ反映する。

## Tasks

- [x] baseline `output/20260717-220801`を解析
- [x] requirements / design / tasklistを作成
- [x] stall watchdog pure helperとtest
- [x] config parse / runtime / debug log
- [x] recovery candidate commit policyとtest
- [x] Forward fallback評価範囲の拡張
- [x] 対象test
- [x] `make autoware-build`
- [x] `make dev3`
- [x] run log解析
- [x] steering / spec更新
- [x] `git diff --check`

`dev3実験で3台恒久停止の再発有無を判定する`は実験自体を完了したが、再発したため未達のままとする。
