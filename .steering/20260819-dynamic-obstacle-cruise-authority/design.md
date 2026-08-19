# Design

## Current boundary

現行 GapPlanner は Follow 中にも全 V2X footprint を使って `lb/ub` と `target_ey`
を生成できる。今回の変更では planner を作り直さず、planner 出力と速度仲裁の間に
Mission 非依存の authority resolver を追加する。

## Admission

次をすべて満たす場合だけ dynamic lateral escape authority を有効にする。

- 設定が有効
- all-V2X dynamic-obstacle target が active
- Behavior は Follow
- GapPlanner が active かつ feasible
- GapPlanner bounds が現在の lateral plan を所有できる
- 有限な `current_ey` / `pass_target_ey`
- pass side があり、設定値以上の横移動を要求している
- EmergencyBrake と solver recovery が非 active

## Runtime behavior

authority 有効時は以下を行う。

1. GapPlanner の stage bounds と target trajectory を既存 MPC へ適用する。
2. Behavior が事前に設定した generic Follow / moving-front cap を解除する。
3. GapPlanner 自身の `target_velocity_limit`、domain speed、壁、操舵、加減速度、
   solver fallback は維持する。
4. Follow preposition より dynamic obstacle target を優先する。

authority が無効なら既存処理を一切変えない。特に planner infeasible では no-gap
velocity limit、EmergencyBrake では停止処理を維持する。

## Refactoring

admission 条件を `v2x_overtake_core` の pure resolver に切り出す。controller は
planner 出力を変換して resolver を呼び、速度仲裁とログだけを担当する。

## Diagnostics

authority の開始・終了時、および有効中は最大 1 Hz で以下を出力する。

- target ID
- pass side
- requested lateral shift
- selected corridor width
- Follow cap suppression
- front risk

## Verification

- resolver の positive / disabled / infeasible / emergency / solver / tiny-shift tests
- `test_v2x_overtake_core`
- package build
- `git diff --check`
- 次回 `make dev2` で dynamic lateral escape、Follow cap、接触、壁余裕を確認
