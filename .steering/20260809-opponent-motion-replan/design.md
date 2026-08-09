# Design

## Problem

現行は相手の縦速度をほぼ等速、横速度を短い線形外挿後の固定値として扱う。
一方で Pass Mission は最大 8--10 秒先まで評価する。この不整合により、
一時的な V2X 横速度や速度差が Mission 全体の false veto になり得る。

また、runtime 不成立で dynamic wait へ入ると現在 generation を失効させるが、
再評価で同じ側に新しい完全成立 Mission が得られても、現在は反対側 Mission
しか atomic replacement できない。

## Changes

### 1. Filtered opponent motion

`TrackedVehicle` に平滑速度と有界加速度を保持する。位置 jump、時刻不正、
速度上限超過時は推定をリセットする。

### 2. Shared rollout prediction

rollout request に次を追加する。

- target longitudinal acceleration
- acceleration application horizon
- lateral velocity decay time

縦方向は短時間だけ有界加速度を適用し、その後は到達速度を保持する。
横方向は `v_d` を指数減衰させ、長時間先に現在の横移動が永続すると仮定しない。

### 3. Fresh same-side replacement

dynamic wait 中も左右両方を完全 preflight する。現在側に新しい Mission が
成立した場合は、その candidate を保存する。

優先順は次とする。

1. 安定判定済みの反対側 Mission
2. fresh same-side Mission
3. 評価継続
4. どちらも不成立なら Recovery

same-side replacement も `freeze_selected_overtake_mission()` を通し、generation
を更新する。失効した固定 path の直接再開は禁止したままとする。

## Runtime evidence

次回 `make dev2` では以下を比較する。

- `Mission generation invalidated`
- `fresh same-side PassPlan replaced`
- `opponent side PassPlan replaced`
- `Pass -> Return`
- `Pass -> FollowPrepare / Recovery`
- rear-clear rollout の predicted acceleration / lateral decay

