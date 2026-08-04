# Pass plan admission and speed ownership

## 目的

狭いコースで追い越し候補が見えているにもかかわらず追従へ留まる事象と、採用済みの最小横移動経路が車体予測上クリアなのに ShiftOut 中の前車速度 cap で失速する事象を解消する。

## 観測事実

- `output/20260804-094212/d1` では mission candidate rejection が 447 回あり、371 回は `goal_candidates=0` だった。
- 約 380 回は dynamic corridor が `observed=0, samples=0` であり、衝突を観測したのではなく、約 1 秒の動的予測範囲へ候補経路がまだ到達していなかった。
- ShiftOut 中に current/predicted footprint が clear でも front cap が残り、指令速度が低速車速度付近まで低下する区間があった。
- Pass へ入った後は footprint policy による cap 解除が機能するが、解除が遅く並走までの時間を失っている。

## 必須要件

1. 動的 corridor が未観測なら、静的壁 corridor と既存の全 mission wall/kinematic preflight を使って候補評価を継続する。
2. 動的 corridor が観測済みで非実行可能な場合は fallback せず棄却する。
3. fallback は target footprint、rear-clear 予測、壁、横加速度の既存 hard guard を迂回しない。
4. 選択した side、横目標、ShiftOut/Pass/Return 距離、closing speed、rear-clear 予測を一つの immutable Pass plan として凍結する。
5. 凍結計画上で current/predicted footprint sweep と物理経路が clear なら、ShiftOut 中から locked target の front cap を解除できる。
6. 車体重複、予測重複、壁接触、target position jump、経路不成立では front cap を解除しない。
7. topic/service/message と提出インターフェースは変更しない。
8. 加速度上限や gap/wall パラメータは今回変更しない。

## 非対象

- 多点の曲率依存 lateral knot を持つ MPCC 相当のオンライン経路最適化
- 接触後 Recovery の再設計
- AWSIM/localization 誤差の補正
- 追い越し閾値の一括攻撃化

## 完了条件

- pure core test で dynamic observed/unobserved/infeasible の admission 境界を固定する。
- pure core test で frozen plan のみ ShiftOut footprint release を獲得できることを固定する。
- 対象 package が build し、既存 test を壊さない。
- 実走ログで `observed=0` による `goal_candidates=0` と、footprint clear な ShiftOut の cap 継続時間を比較可能にする。
