# Design

## 1. Pass live rolloutの残横移動化

Pass中のlive rear-clear予測では、`line_cfg.shift_distance`を再利用しない。
current `e_y`からcommitted goalまでの横差を
`resolve_same_side_replan_shift_distance()`へ渡し、0.5 m以上の短い補正rampだけをrolloutする。

これにより、完了済みShiftOutをPass必要距離へ再加算して
`rear_clear_window`をPass開始直後に誤発火させる経路を除く。

## 2. Atomic commit判定の構造化

boolだけを返す`can_commit_same_side_extension()`に加え、判定結果と棄却理由を返す
`evaluate_same_side_extension_commit()`を追加する。既存bool APIはwrapperとして維持する。

replacementは次を必須とする。

- active Pass、同一target、同一side、同一generation
- planner result ageとprediction expiryがfresh
- absolute距離内
- Pass hold/static終端が現在より前進
- lateral goal変更が上限内

dynamic valid distanceは1秒V2X horizonごとに再生成される短期値であり、static Pass終端の
前進性とは別物である。fresh expiryを満たす限り、旧dynamic距離より数cm短いことだけでは
replacementを棄却しない。

## 3. SafeSeparation sub-FSM

OvertakeLineのPass side/goal所有を維持するため、広範なphase追加ではなくPass内sub-FSMとする。
extension不能またはbounded horizon限界時に、次をすべて満たす場合だけ開始する。

- locked target継続観測
- current body footprintsが非重複
- wall contact/sample lossなし
- target jump/pass-side intrusionなし
- Emergencyなし
- solver recoveryなし

SafeSeparation中の縦速度参照は相手との前後関係から決める。

```text
targetが前または横並び:
  ego_ref = max(0, target_speed - separation_delta)
  -> targetを前へ離す

targetが後ろ:
  ego_ref = target_speed + separation_delta
  -> targetを後ろへ離す
```

横目標はcommitted pass goalを固定する。targetがrear-clearになればReturn、targetが十分前へ
離れればRecoveryへ移る。時間・距離上限または安全条件喪失時は従来Recoveryへfall backする。

## 4. 設定

- `v2x_overtake_safe_separation_enabled: true`
- `v2x_overtake_safe_separation_speed_delta: 0.8 m/s`
- `v2x_overtake_safe_separation_front_clear_distance: 2.0 m`
- `v2x_overtake_safe_separation_max_sec: 3.0 s`
- `v2x_overtake_safe_separation_max_distance: 8.0 m`

既存Pass horizonの1秒/3 m holdはShiftOut fresh-horizon待機用として維持し、Pass extension失敗後は
独立したSafeSeparation上限を使う。

## 5. Non-goals

- legacy completion guardの削除・統合
- V2X course-progress予測の有効化
- latch閾値、`ay_max`、曲率閾値の変更
- wall/overlap/Emergency guardの緩和
