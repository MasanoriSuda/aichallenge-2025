# Design

## 1. Supervisor と execution layer

既存 Behavior / OvertakeLine FSM は target 管理、hard fault、Recovery を所有する。MPCC-lite はその下で Left / Right / CurrentSideHold / Return を比較し、新規 entry の side・横 goal・closing speed を所有する。

## 2. Receding prefix

従来の complete Mission は ShiftOut、Pass、rear-clear、Return の全区間成立を要求する。一方 MPCC prefix は次だけを hard gate とする。

- ShiftOut/body-clear rollout が成立
- 局所 wall preflight が成立
- target surface clearance が検証済み
- 横加速度上限内
- runtime hard fault なし

rear-clear が現在 horizon 外の場合、body-clear 時間・距離と retained speed を terminal progress として score へ渡す。prefix は完全 Mission と区別し、active Mission の原子的な反対側置換には使わない。

## 3. Authority 範囲

- Idle / 新規 entry: Left/Right の winning prefix または complete Mission を side selection と selected Mission に反映する。
- ShiftOut / Pass / FollowPrepare: frozen Mission を基本とし、MPCC の CurrentSideHold/Return は診断と継続判断に使う。反対側置換は既存の full preflight/no-return gate を通る場合だけ許す。
- Return: Return を維持する。

この段階では無検証の全幅切替を入れず、candidate coverage と entry authority を先に成立させる。

## 4. Last feasible

選択した executable Mission 本体も cache する。再利用条件は target、generation、phase、side、最大 age の一致と runtime hard fault 不在。新規 entry の side=0 context では target と phase の一致を要求する。

## 5. 周期

両側の重い rollout は 5 Hz を初期値とする。40 Hz の低レベル MPC は前回の feasible Mission を継続する。ログは 1 Hz と recommendation 変更時に出す。

## 6. Fail-safe

MPCC が解を返さない場合は既存 side selection/FSMへ戻す。hard fault で last-feasible を使わない。従来の wall/contact/solver guards は変更しない。
