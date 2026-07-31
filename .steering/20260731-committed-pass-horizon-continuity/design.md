# 設計

## 原因

現行 `CommittedPassPolicy` は既存 release の hold 許可を
`constrained_horizon_release_allowed` に従属させている。

この値には `execution_path_physically_feasible` が必要なため、Pass 中に横離隔を十分
確保していても、短い先読み区間の wall / lateral acceleration 制約で horizon が
infeasible になると front cap が即座に再適用される。

## 変更

初回 release と既存 release の hold を分離する。

- 初回 release:
  - 従来どおり execution horizon の成立を要求する。
- 既存 release hold:
  - Pass、lateral latch、車体横離隔、reapply hysteresis、観測連続性、非接触を要求する。
  - horizon feasibility は要求しない。

`can_release_overtake_front_cap()` が持つ target validity、finite longitudinal、
reapply threshold の検査は維持する。

## 速度権限

この変更が外すのは locked target 由来の front-speed cap だけである。

- MPC / course / domain hard cap
- wall / lateral acceleration limit
- SafetyBrake
- Recovery velocity limit
- committed Pass speed floor の physical feasibility 条件

は変更しない。

## 診断

既存の周期 debug にある次の値で動作を確認する。

- `cap_release=1`
- `horizon_release=0`
- `speed_hold=1`
- `wall_limited` / `static_wall_limited`

horizon infeasible 中に上記の組み合わせとなれば、新しい hold が機能している。

