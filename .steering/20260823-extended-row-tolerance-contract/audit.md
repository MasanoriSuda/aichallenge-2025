# Audit

## Pre-change evidence

- Replay: `output/20260823-overtake-canonical-intent-replay/d1/autoware.log`
- Overtake canonical fresh shadow:
  - evaluated 404
  - eligible 390
  - lateral contract passed 353
  - complete 353
- Rejected windows:
  - violation `0.0967127 m`, row tolerance `0.0165387 m`
  - violation `0.0607105 m`, row tolerance `0.0164280 m`
  - violation `0.0572577 m`, row tolerance `0.0164374 m`

## Important semantic correction

`evaluate_extended_lateral_constraint_contract()`は、state x0のbox rowではなく
`(stage + 1)`の横box rowを監査する。従ってログのstage 0は最初の予測状態x1。
初期Frenet座標の二重投影は今回の直接原因ではない。

## Post-change evidence

### Broad-policy rejection replay

- Replay: `output/20260823-extended-row-tolerance-replay/d1/autoware.log`
- Track/Cruiseへも`RowToleranceNormalized`を適用した試作。
- Track/Cruise production solve-failure: 45周期。
- 代表例: dynamics row 210、`maximum_normalized_violation=1.03665..1.41724`。
- retained候補はdynamic obstacle presenceで拒否され、通常authorityの停止を増やした。
- 結論: 全context一括適用は本sliceでは棄却。

### Scoped-policy final replay

- Replay: `output/20260823-extended-row-tolerance-replay-v3/d1/autoware.log`
- Domain: 1
- Source bag: `output/20260823-214300-stop-authority-replay-v2/d1/rosbag2_autoware`
- `/awsim/state=Start`を明示注入し、記録済みcontrol commandを除外して再生。

Overtake canonical fresh shadow集計:

- evaluated: 1188
- eligible/context: 1136/1136
- lateral: 1136（eligible比100%）
- complete: 1130（eligible比99.47%）
- lateral-contract reject: 0
- shadow time: 0.530 ms avg / 1.277 ms max

残る6周期はlateral rowではなく後段physical証明の棄却であり、本原因から分離する。

Track/Cruise:

- production solve-failure: 0
- broad-policy replayの45周期退行は消失。

Runtime:

- full callback overrunは165周期残る。
- Overtakeが長時間activeとなる既存処理全体の計算負荷であり、本sliceのshadow処理単体は
  1.277 ms以下。Overtake production昇格前の残課題として扱う。
- main OSQP集計は1674 cycles、15 failures、2.130 ms solve avg、19.265 ms max。
  failureにはFollowの低速・停止車局面等も含まれ、本sliceだけの因果にはできない。

### Static validation

- `make autoware-build`: success（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 programs passed
- `colcon test-result --verbose`: 1693 tests、0 errors、0 failures、0 skipped
- 既存の`build/joycon_contract_guard/package.xml`欠落警告は継続。
- mixed-unit回帰は既存
  `PersistentOsqpSolver.RowToleranceNormalizationClosesMixedUnitToleranceLeak`
  で確認。
