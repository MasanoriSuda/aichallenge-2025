# V2X低速回避・回復方向デッドロック修正 Results

実施日: 2026-07-17

## 検証結果

| 検証 | 結果 |
|---|---|
| `make autoware-build` | 成功、25 packages |
| `test_v2x_overtake_core` | 36 / 36成功 |
| `test_stuck_recovery_core` | 59 / 59成功 |
| `test_recovery_footprint` | 25 / 25成功 |
| `make dev3` | 起動成功、受け入れ失敗 |

対象unit testは合計120件成功した。

## dev3 run

- run: `output/20260717-225927`
- baseline: `output/20260717-220801`
- 3台のAutowareとAWSIMは正常起動し、各DomainでStartを受理した。
- `LowSpeedAvoidance`遷移と`low_timeout=1`は0件で、baselineと同じ停滞条件は発生しなかった。
- D1はWP 118付近のwall contactからstepwise ReverseLeftを5ステップ実行し、contactを
  44 cellsから0まで減らした。その後ReverseStraightを含む累積2.015 mの退避を完了したが、
  LowSpeedRejoin中に新規contactが発生し、Startから約54.1秒で`rejoin_unsafe` SafeStopとなった。
- D3はWP 183付近でpathに対して約1.71 radずれた姿勢で停止した。現在footprintはclearだったが、
  forward static fallbackが成立せず、Reverse corridorもD2に塞がれた。Startから約62.1秒で
  `clearance_wait_timed_out` SafeStopとなった。
- D2は共通コース進捗でD3を前方危険車両として検出し、WP 183でSafetyBrake停止した。
- したがって3台恒久停止は再発した。今回の修正は安全gateを維持してfail-closedに動作したが、
  crash後の自車単独回復だけでは解消できない配置だった。

## 判定

コード単体の受け入れ条件は成立したが、dev3の「3台すべてが恒久停止しない」は不成立である。
次の修正対象は、回復gateの緩和ではなく、D1のLowSpeedRejoin新規接触とD3のWP 183進路喪失を
起こす前段のtrajectory/MPC/V2X横目標である。安全なstatic / V2X rolloutがない状態で
ForwardまたはReverseを強制することはしない。
