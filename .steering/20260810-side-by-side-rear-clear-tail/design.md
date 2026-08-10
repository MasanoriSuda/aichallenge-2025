# Design

## 原因

現行は `committed_forward_completion.active` がfalseになる理由を一つの
`committed_forward_completion_guard_lost` にまとめている。この値には、

- 現在／予測車体の物理不成立
- 壁、target continuity、EmergencyBrake、solver fault
- rear-clear rolloutが残りtime／distance budgetへ収まらない

が混在する。SafeSeparationのshort-horizon判定はこの混在値をpredictive guardとして使うため、
物理的にはclearでも予算見積りだけで即Abortする。

また通常local SafeSeparation budgetへ到達した場合、fresh progressがないと同様にAbortする。
対象車が既に後方へ移った境界付近では、V2Xの数cm変動だけでprogress ageが古くなり、
rear-clear直前にPassを捨てる。

## 方針

### 1. 物理成立と完遂見積りを分離

`PassShortHorizonGuardRequest` に、次の全条件をcontrollerで確認した
`side_by_side_rearward_completion_safe` を渡す。

- commit stageが`SideBySideCommitted`
- target identity / course progressが連続
- target longitudinalが0 m以下
- current bodyが分離
- predicted footprint sweepが分離
- execution corridorがclear
- wall / intrusion / emergency / solverのhard guardがclear

完遂rolloutが残り通常予算へ収まらない場合でも、この物理条件が成立すれば
short-horizonをsafeとする。予測欠損やoverlapへこの経路は適用しない。

### 2. local budgetをrear-clear tailではsoft化

上記のrearward completionがactiveで、forward completionが既にlatchedしている場合、
通常local time／distance limitはPass中断理由にしない。同じsideで前進を継続する。

absolute Pass time／distance limit、rear-clear、hard faultは従来どおり終了条件とする。
この継続では新しいside、横goal、追加のprediction graceを生成しない。

### 3. 観測性

rear-clear tailが初めて所有権を得た周期に一度だけ、target_s、progress age、local／absolute
budgetをログへ出す。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: typed guardとSafeSeparation reason
- `mpc_controller_cpp.cpp`: 物理条件の配線、速度所有権、状態ログ
- `test_v2x_overtake_core.cpp`: hard faultとbudget境界の回帰テスト
- topic、message、launch、yaml: 変更なし
