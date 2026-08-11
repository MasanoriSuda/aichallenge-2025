# Design

## 1. Body-clear setup candidate

Mission candidate探索中、次を満たした最良候補を entry setup として保持する。

- 初期 ShiftOut preflight が成立
- 横加速度上限内
- kinematic rollout が有効かつ body-clear deadline を満たす

rear-clear rollout、全 Mission preflight、Return は setup の成立条件に含めない。
setup は lateral goal を出力せず、既存 pre-arm と同じ時間・距離上限、front-risk guard
の下で `target speed + bounded closing speed` のみを要求する。完全 Mission が成立した
周期だけ OvertakeLine へ横経路を渡す。

## 2. Committed target continuity

ShiftOut/Pass の Behavior ownership は、`locked_target_seen` 単独ではなく、frozen Mission
の target ID と現在 Behavior target ID が一致し、front/side/engagement observation が
継続していることを条件にする。一周期の course-relative geometry 欠落は許容するが、
position jump、course progress reject、pass-side intrusion、Emergency、solver recovery は
許容しない。

## 3. Soft horizon loss

短い horizon 不成立は現在経路の無条件継続には使わない。現在車体が分離し、target が
連続し、壁/corridor hard fault がない場合に限り、Mission ownership を保持した
FollowPrepare/dynamic wait へ移し、fresh Mission または last-feasible Mission を再評価する。
現在重複や壁 fault は Recovery のままとする。

## ログ評価

- `overtake entry setup active`
- `Overtake -> Follow` while OvertakeLine ShiftOut/Pass
- `Pass -> Return`
- `SafeSeparation aborted` 後の DynamicMissionWait/Recovery

