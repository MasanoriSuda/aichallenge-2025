# Design

## 再発ログ

`output/20260820-000147/d1/autoware.log`では次の順に発生した。

1. `squeeze response entered`
2. `front_cap=Reapplied`
3. 約23 ms後に`Overtake -> Follow`
4. `squeeze response ended`, `reason=front-cap-not-released`
5. 約148 ms後に`Follow -> SafetyBrake`

response自身が実行したfront cap再適用により、次周期の事前条件を満たさなくなっていた。
そのため0.15 mの横逃げは1周期だけで失効し、Passの横Missionも失っていた。

## 修正方針

`prior_front_cap_release_active`を初回取得にだけ必須とする。
前周期にresponseがactiveであれば、front capが再適用済みでも次の共通条件を再評価する。

- Passとminimum-motion corridorが有効
- target identityが連続
- current footprintが未重複、またはcurrent-overlap確定待ち
- predicted sweepの重複が継続
- ContactContinuationが未所有

保持中のreasonは`active-held-after-front-cap-reapply`とし、新規の毎周期ログは追加しない。
既存の周期debugと状態遷移WARNから判別できるようにする。
