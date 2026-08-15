# Design

## Observed failure

最新走行では `ShiftOut -> Pass` へ入る一方、最適化後の経路が target separation
境界を外れ、既存の target-bound execution prefix の 1.5 s / 8 m budgetを消費後に
FollowPrepareへ遷移していた。これにより、抜きかけで前車速度付近まで失速する。

## Policy

既存の短い target-bound prefix budget は、通常の optimizer repair deadline として
維持する。budget超過後は、次を全て満たす場合だけ progress extensionを許可する。

- phaseがPass
- frozen Missionと同側の物理prefixが現在も実行可能
- target observationが連続し、車体が分離済み、またはrecoverable side contact
- target longitudinalが直近の設定時間内に0.05 m以上改善
- Pass開始からの絶対時間・絶対距離上限内
- wall/front/solver/forbidden waypointのhard faultなし

延長中も左右shadow evaluationは毎周期再armし、fresh same-side Missionを優先する。
progressが古くなれば同一generationをexhausted扱いとし、同じMissionでは再armしない。

## Refactoring

短期budgetとprogress extension、absolute budgetの組合せは
`can_hold_target_bound_execution_for_replan()`へ集約する。controllerは観測値と
Mission runtimeを渡し、policyの真偽を重複実装しない。
