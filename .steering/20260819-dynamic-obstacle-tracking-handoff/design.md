# Design

## 原因

GapPlanner は時刻ごとの車両占有から空き区間を生成するが、その区間へ追従 MPC の現状態から連続的に入れることまでは保証しない。現行は選択区間をそのまま `lb/ub` と横参照へ適用し、同時に Follow cap を解除していた。このため不連続な横制約で OSQP が失敗し、次周期は fallback gate で planner を外し、その次周期に再採用するチャタリングが発生した。

## 変更方針

### Reachability bridge

scoped dynamic-obstacle Follow plan に限り、各 active sample を既存の `resolve_frenet_dp_execution_envelope()` で検証する。

- 現在 `e_y`
- `v * sin(e_psi)` の横速度
- waypoint までの path distance
- overtake guard の横加速度上限
- 設定可能な横加速度 reserve ratio

到達可能区間と corridor が交差しない候補は MPC に渡さない。交差する場合は hard corridor 自体を狭めず、横参照だけを到達可能な交差区間へ clip する。

### Two-phase handoff

bridge 済み候補は Follow cap を残したまま一度 tracking MPC で解く。成功した target/side の次周期からのみ Follow cap を解除する。target または side が変われば再確認する。

### Solver quarantine

lateral escape active 中に solver failure が起きたら、候補の保持値を消し、設定時間だけ scoped planner admission を止める。fallback と planner の一周期ごとの交互切替を防ぐ。

### Ownership boundary

scoped planner が request された周期では、bridge と authority の両方を通過した出力だけが下流の bounds/reference を所有する。通常の Follow GapPlanner、Overtake、LowSpeedAvoidance は既存挙動を維持する。

## 期待ログ

- `bridge=valid/reject`
- `qualified=0/1`
- `quarantine_remaining`
- bridge reject index/reason

1 秒周期または状態変化時の既存ログへ集約し、周期ごとの追加ログは出さない。
