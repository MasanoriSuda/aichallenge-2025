# Design

## 1. 現行実装の不整合

現行のsame-side extensionには次の問題がある。

1. `extension_shift_distance` が常に0.5 mである。
2. rolloutを旧fixed goalで先に評価し、その後のstatic preflightでgoalを補正している。
3. atomic replacementのgoal差を、通常追従用の `max_target_change=0.04 m/cycle` で制限している。
4. replan要求はvalidation horizon期限だけで、confirmed predicted overlapを直接扱わない。
5. extension失敗理由が単一の `extension unavailable` に潰れている。

このため、実車位置が旧goalからずれた場面や対象車が横へ寄った場面では、必要な横補正を
rolloutへ反映できず、extension 0/3となる。

## 2. Predicted-overlap trigger

既存の予測重複確認timerをhorizon判定より前で一度だけ更新する。

```text
Pass
and minimum-motion corridor
and front cap release済み
and target continuity正常
and current footprint非重複
and targetがまだside-by-sideより前
and predicted footprint overlapがconfirm時間継続
  -> RequestSameSideExtension
```

rear-clear、absolute limit、現在重複、Emergencyは従来どおり先に評価する。extension上限到達後に
再び予測重複した場合はbounded Holdへ進む。

## 3. Same-side replacement goal

現在のfixed goalだけをrolloutへ渡さず、予測可能な場合は予測終端のtarget横位置、
予測不能時は現在target位置と壁範囲から同じ側の最小安全goalを求める。

```text
wall-feasible interval
  ∩ selected-side target separation
  -> nearest feasible goal from current fixed goal
```

反対側candidateは作らない。候補goalが現在fixed goalから離れられる総量は、新設する
`pass_horizon_extension_max_lateral_adjustment` で制限する。既定は0.60 mとし、通常の
`max_target_change`（1周期の平滑化）と責務を分ける。

## 4. Lateral-adjustment distance

replacementの横移動距離は固定0.5 mではなく、横移動量、現在速度、横加速度上限から求める。

```text
required_time = sqrt(2 * abs(goal - current_ey) / max_lateral_accel)
required_distance = current_speed * required_time
replacement_shift_distance = clamp(
  required_distance + margin,
  0.5,
  configured ShiftOut distance)
```

この距離を使ってrolloutを実施し、そのrear-clear距離からPass保持距離を再計算する。最後に
replacement Shift/Pass/Return全体をstatic preflightへ通す。

## 5. Atomic commit

commit条件は既存のgeneration、target、side、planner age、prediction expiry、absolute limitを維持する。
同一制御周期でdynamic valid-untilを更新した結果とreplacementが同値になるケースは、Pass保持距離が
前進し、effective valid-untilが後退していなければ採用可能とする。
採用時に次を一括更新する。

- fixed goal
- replacement shift distance
- Pass保持距離
- Return開始距離とReturn距離
- rear-clear予測
- static/dynamic valid-until
- planner/prediction時刻
- mission generation / extension count

Pass中のreplacement shiftは新しいphaseを増やさず、Pass path内部の先頭区間として生成する。
その区間だけは新goalを即時保持せず、保存した開始横位置からsmoothstepで進める。

## 6. Failure diagnostics

extension結果を次へ分類し、要求1回につき1行だけ出す。

- invalid runtime input
- fresh prediction unavailable
- no same-side separated goal
- lateral adjustment exceeds limit
- rollout invalid / rear-clear infeasible
- dynamic Pass distance infeasible
- static replacement preflight rejected
- stale/non-advancing commit

失敗後は既存の1.0秒/3.0 m bounded Holdへ入り、同じfallback開始点を再装填しない。

## 7. 検証

- 純粋関数: predicted-overlapによるhorizon action
- 純粋関数: 横移動量からreplacement shift距離を算出
- 純粋関数: 通常平滑化値とは独立したatomic goal変更上限
- 既存 `test_v2x_overtake_core`
- Release build
- `make autoware-build`
- 次回 `make dev2` ではextension成功数、SafetyBrake、Reverse、Pass完遂を比較する
