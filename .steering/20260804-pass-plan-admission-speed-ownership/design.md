# Design

## 方針

現行の `OvertakeMissionCandidate` は side、横目標、closing speed、rear-clear、Pass hold、Return 距離を既に保持している。一方、controller state へ個別コピーされるため、経路成立性と速度 ownership が別々に判断されている。

今回、candidate から `OvertakePassPlan` を構築して一括凍結する。経路形状は既存の距離ベース `ey(s)` (`OvertakeMissionPathRequest`) を維持する。これにより大規模な軌道生成変更を避けつつ、admission と実行中速度判断が同一計画を参照できるようにする。

## Dynamic corridor admission

候補横目標の許容区間は次の優先順位で解決する。

1. dynamic corridor に有効な観測がある: dynamic interval を採用する。
2. dynamic request は有効だが観測がまだない: static wall interval を fallback として採用する。
3. dynamic 観測があり infeasible: 棄却する。
4. 入力不正、static interval 不正、candidate gap 不成立: 棄却する。

fallback 後も以下を全て実行する。

- target footprint に対する line-entry preflight
- body-clear deadline/rear-clear rollout
- mission 全体の static wall swept-footprint validation
- lateral acceleration validation

従って `observed=0` は「安全」の証明ではなく、「動的観測が届くまで静的 bounds で候補生成を継続する」ための入口に限る。

## Frozen Pass plan

`OvertakePassPlan` は少なくとも次を一括保持する。

- pass side
- `ey(s)` の ShiftOut/Pass/Return path
- closing speed
- body-clear/rear-clear prediction
- static/dynamic valid horizon
- outer-strategy commitment
- static fallback の利用有無

選択時に validation し、実行中は target の短周期横揺れで side/goal を差し替えない。既存の同側 horizon refresh は計画の安全な延長として扱い、更新成功時のみ plan と state を同時更新する。

## ShiftOut speed ownership

Pass だけに限定されていた minimum-motion footprint release を、次を全て満たす frozen ShiftOut に拡張する。

- frozen Pass plan がある
- minimum-motion corridor が active
- locked target を継続観測し position jump がない
- actual wall contact がない
- current footprints が非重複
- footprint prediction が有効で sweep が非重複
- execution path が物理的に成立

これは全開強制ではなく locked target の follow cap を外す処理である。MPC、曲率速度、壁、別車両、加速度上限は引き続き速度を制限できる。

## ログ

- candidate selection に corridor source (`dynamic` / `static_fallback`) を出す。
- committed Pass policy に frozen plan と ShiftOut footprint release の状態を出す。
- mission freeze に path distance、side、goal、closing speed、fallback 有無を出す。

## 残リスク

現行 `ey(s)` は Pass 区間で単一横目標を保持する。ヘアピン連続区間で「外まくりが後半にインへ変わる」問題を完全には解消しない。今回のログで admission/cap の誤動作を除いた後、必要なら次段で曲率に応じた複数 lateral knot を PassPlan へ追加する。
