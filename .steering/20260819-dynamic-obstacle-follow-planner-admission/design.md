# Design

## Planner admission

Follow state の planner admission を pure resolver へ切り出す。

scoped admission は次をすべて満たす場合だけ active とする。

- dynamic lateral escape policy enabled
- dynamic-obstacle target active
- Behavior final state is Follow
- pre-arm inactive
- active Pass gap-hold inactive
- explicit forbidden waypoint inactive
- EmergencyBrake inactive
- solver recovery/fallback inactive

通常の Follow GapPlanner admission は既存設定をそのまま使い、最終的な
`allow_gap_planner` は generic または scoped admission の OR とする。

soft curve forbidden は Mission admission の制約であり、動的障害物に対する横回避の
可行性評価までは止めない。実行は後段の GapPlanner feasible、hard state bounds、
minimum lateral shift、Emergency/solver gate を再度通過した場合だけ許可する。

## Diagnostics

planner request と実行authorityを分けて記録する。

- `requested=1, active=0`: plannerは評価したが回廊不成立または横移動不足
- `requested=1, active=1`: 横回避とFollow cap suppressionを実行
- `requested=0`: hard gateまたは対象なし

## Verification

- scoped admission のpositive test
- disabled/target missing/non-Follow/pre-arm/active Pass/explicit forbidden/Emergency/solver tests
- existing lateral authority tests
- package build と focused test
- 次回 `make dev2` で Follow `allow_gap=1` と authority `requested/active` を確認
