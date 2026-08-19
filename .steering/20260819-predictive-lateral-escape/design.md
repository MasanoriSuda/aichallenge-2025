# Design

## Policy

Mission は target と pass side を保持する supervisor として残す。Pass 実行層は
壁 warning 時に、現在の同側経路を次の順で再評価する。

1. nominal target clearance + robust wall clearance
2. physical target clearance + robust wall clearance
3. nominal target clearance + hard wall clearance
4. physical target clearance + hard wall clearance

後段への縮退は、現在車体 footprint が非重複で、既存の hard wall monitor が
正常な場合に限る。候補 goal を求めた後は既存の entry preflight と static wall
footprint 検証を通すため、設定値を単純に緩めて無条件採用する変更ではない。

## Refactoring

`RuntimeWallCenterContractionGoalRequest` に preferred wall interval と physical
wall interval を明示する。Resolution は target clearance と wall clearance の
どちらを物理余裕へ縮退したかを別々に返す。これにより、controller に散在して
いた暗黙の clearance 意味を pure function と単体テストへ閉じ込める。

## Runtime integration

- preferred interval: planner bounds +/- robust planning wall clearance
- physical interval: planner bounds +/- `v2x_overtake_line_min_wall_clearance`
- lookahead: 0.80 s -> 1.20 s
- per-replan lateral correction upper bound: 0.35 m -> 0.60 m

0.60 m は一度に必ず動かす値ではなく、候補探索の上限である。実際の goal は
wall/target bounds に clamp され、DP prefix と横加速度 preflight を通過した場合
だけ Mission へ原子的に反映する。

## Failure behavior

全候補が不成立なら従来どおり current-side hold、speed-preserving Return、
DynamicMissionWait / Recovery の順で処理する。物理的に通れない状況で前進を
強制する変更は行わない。

## Verification

- pure resolver の nominal / physical-target / physical-wall / fail-closed tests
- `test_v2x_overtake_core`
- package build
- `git diff --check`
- 次回 `make dev2` で clearance mode、Pass 完遂、壁接触、速度低下を確認
