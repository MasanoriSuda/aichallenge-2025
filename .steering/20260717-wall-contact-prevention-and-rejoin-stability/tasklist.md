# 壁接触予防・再合流安定化 Tasklist

作成日: 2026-07-17
状態: Experiment Complete / Acceptance Partial

## Definition of Done

- [x] front hazard holdを実装・testする。
- [x] LowSpeedRejoin前進preflightとblocked-path再評価を実装・testする。
- [x] clear-footprint候補をheading-awareにする。
- [x] buildと対象testを成功させる。
- [x] dev3実験を旧停止時刻より長く実行する。
- [x] run判定と正本仕様を更新する。

## Tasks

- [x] baseline `output/20260717-225927`解析
- [x] requirements / design / tasklist作成
- [x] hazard hold helper / runtime / debug / test
- [x] rejoin preflight / retry / test
- [x] heading-aware candidate selection
- [x] `make autoware-build`
- [x] target test suites（39 + 61 + 25 passed）
- [x] `make dev3`（`output/20260717-232948`、Start後約190秒監視）
- [x] log解析
- [x] steering / spec更新
- [x] `git diff --check`

## Verdict

- hazard dropoutによるCruise復帰: 解消を確認
- LowSpeedRejoin中の新規wall contact: preflightによる予防を確認
- 旧停止時刻62秒超の走行: 確認
- 3台停止列の最終解消: 未達（壁際閉塞と後続車によるreverse corridor閉塞）
