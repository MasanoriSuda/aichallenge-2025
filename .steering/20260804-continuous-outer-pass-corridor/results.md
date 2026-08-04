# 実装結果

## 変更概要

- Pass中の基準曲率を12 m先までローリング評価し、外側が3 m以上継続して反転する
  場合だけside transitionを要求するようにした。
- locked targetが4 m以上前方にあり、車体非重複、予測連続、壁・solver hard guard正常の
  場合だけ、反対側goalを生成する。
- transitionは次カーブまでに完了でき、rollout上も完了時target longitudinalが2 m以上残る
  場合だけcommitする。横並びから反対側へ横断しない。
- goal、side、PassPlan、front-cap stateを一括更新し、検証済みdistance-domain rampで実行する。
- early transition後、切替理由となったカーブへ到達するまでは旧曲率による逆要求を抑止する。
- rolling replan有効時は、将来の曲率符号反転だけで初回missionを棄却しない。wall、body、
  rear-clear、横加速度、絶対Pass上限は維持する。

## 設定値

```yaml
v2x_overtake_continuous_outer_replan_enabled: true
v2x_overtake_continuous_outer_replan_lookahead_distance: 12.0
v2x_overtake_continuous_outer_replan_min_curve_distance: 3.0
v2x_overtake_continuous_outer_replan_min_front_distance: 4.0
v2x_overtake_continuous_outer_replan_cooldown_sec: 1.0
v2x_overtake_continuous_outer_replan_max_lateral_adjustment: 3.2
v2x_overtake_continuous_outer_replan_max_shift_distance: 8.0
v2x_overtake_continuous_outer_replan_max_count: 3
```

## 検証

- `make autoware-build`: 成功
- package全25 test target: 成功
- test result: 841 tests、0 errors、0 failures、0 skipped
- overtake core: 344 tests、0 failures
- `git diff --check`: 成功

動的走行は未実施。次の`make dev2`では成功ログ
`OvertakeLine rolling outer transition accepted`、side遷移、front-cap再解除時間、接触、
SafeSeparation/Recovery理由を確認する。
