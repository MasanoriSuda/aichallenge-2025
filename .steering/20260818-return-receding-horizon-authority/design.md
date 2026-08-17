# Design

## 現状の不連続

進捗型MPCそのものはReturnでも有効だが、次の三つがShiftOut/Pass限定だった。

- `optimize_live_overtake_line_horizon()` のactive phase
- QP solved execution trajectoryの抽出context
- solved trajectoryのphase整合・wall authority判定

このため、Passで使っていた連続解はReturn遷移時に失効し、6 mのpreflight profileを
そのまま追う。Return完了判定は横偏差だけでなくheading収束も要求するため、短距離で
中央へ戻す固定profileは壁接触とhandoff未収束を同時に起こし得る。

## 修正方針

### 1. phase分類の局所整理

同じ条件式の散在を避けるため、次をローカルhelperへ集約する。

- target interaction phase: ShiftOut / Pass
- receding-horizon execution phase: ShiftOut / Pass / Return
- solved trajectory handoff: same phase、ShiftOut -> Pass、Pass -> Return

### 2. Returnの連続最適化

Return preflight referenceはnominalとして残す。その周囲で毎周期、壁境界、横加速度、
warm start、軌道平滑性を含めて再最適化する。固定profileを削除せず、optimizer不成立時の
既存fallbackとして維持する。

### 3. Passからのwarm start

同じtarget、Mission generation、sideで、解のageが既存lease内の場合だけ
Pass solved trajectoryをReturnの初回warm startへ利用する。course progressで進み量を
補正し、Return中はReturn phase distanceをMission progressへ加算する。

### 4. Return固有の制約

- rear-clear latch済みなら、追い越し済みtargetの横boundsを解除する。
- Returnではcurve outer biasを加えず、中央復帰nominalを優先する。
- Return corridor blocker、wall、EmergencyBrake、solver/forbidden guardは維持する。
- targetが後方で観測leaseを失っても、rear-clear latchとReturn corridorが健全なら
  solved pathの短期継続を許可する。

## 期待するログ変化

- Return中も `receding=1` が継続する。
- Return開始後に新しいReturn phaseのsolved trajectoryが記録される。
- `MPCC solved trajectory released ... warning inactive` がphase遷移だけを理由に出ない。
- `Return -> Idle` が成立し、`Return -> Recovery` のwall理由が減る。
