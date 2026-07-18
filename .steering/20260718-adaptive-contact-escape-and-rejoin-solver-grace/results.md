# Adaptive Contact Escape and Rejoin Solver Grace Results

実施日: 2026-07-18
判定: 部分成功。適応操舵と2.0 m離脱は確認したが、3台全停止は未解消。

## 実装結果

- Side / Mixed contactでStraightと左右0.05〜0.25 radを評価し、contact減少最大の角度を選択・固定する。
- LowSpeedRejoinのsolver再初期化を最大1.0秒停止保持する。timeoutは従来どおりSafeStopする。
- 最大stepを8から10へ増やした。総後退距離3.0 m、速度、時間、static / V2X gateは維持した。
- 成功したepisodeのstep / attempt予算を、次の独立したRecovery開始時にresetする。

## 検証

- `make autoware-build`: 25 package成功。既存のsetuptools deprecation warningのみ。
- `test_stuck_recovery_core`: 67/67成功。
- `test_recovery_footprint`: 26/26成功。
- 合計: 93/93成功。

## dev3結果

| Run | 主な結果 | 判定 |
|---|---|---|
| `output/20260718-162748` | D2は8 step・1.820 mで2.0 m直前に上限停止。D1は後方車block、D3は0.614 m後に0.447 m先のstatic collisionを予測して停止。 | step上限不足を再現、安全gateはPass |
| `output/20260718-163329` | D2は10 step・2.182 mでLowSpeedRejoinし、`rejoin_complete`後にWP161からWP199まで前進。後の詰まりでstep 10/10を持ち越す不具合を検出。 | 10 stepと再合流はPass、episode予算は要修正 |
| `output/20260718-164220` | 最終コード。D3は実操舵`-0.15`、`-0.20`、`-0.25 rad`を選択し、contactを154から99へ低減。D2は6 step・2.006 mでLowSpeedRejoin。 | 適応操舵と離脱はPass、全停止はFail |

最終runのD1は0.35 m先のstatic collisionにより`maneuver_direction_unknown`、D2は
LowSpeedRejoinの5秒以内に姿勢整合できず`rejoin_timed_out`、D3はcontactを減らしたが
10 step・1.076 mで`escape_step_limit_reached`となった。いずれも危険候補を強制実行せず、
Rear V2X不完全時の後退やcontact悪化中の継続駆動は観測していない。

2回目runで判明したstep予算持ち越しは、最終コードの2 episode連続unit testで
`1/1 -> rejoin_complete -> 0/1 -> 新規RequestReverse`を確認した。最終runでは最初の
episodeが完了しなかったため、runtimeでの再観測には至っていない。

solver graceは復旧・timeoutのunit testに成功した。dev3でLowSpeedRejoinに入った周期のsolverは
正常だったため、`solver_recovery_pending`の実commandは未観測である。

## 結論

適応角度探索は深いcontactでも実際に中間角を選んで接触量を減らし、step上限10は旧上限直前停止を
解消した。一方、全車走行継続には至っていない。次の優先課題は安全gateの緩和ではなく、通常MPCが
深いwall contactへ入る前の抑制と、LowSpeedRejoinで横偏差を時間内に収束させる経路・姿勢制御である。
