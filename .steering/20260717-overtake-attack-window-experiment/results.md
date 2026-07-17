# Results: Overtake Attack Window Experiment

## Validation

- `make autoware-build`: 25 packages succeeded。
- `test_v2x_overtake_core`: 26 tests passed。
- `make dev3`: `output/20260717-074229`で約2周分を走行して停止。

## dev3 observations

AWSIMの`Start`通知以降を車両ごとに集計した。

| Vehicle | Overtake entry | SafetyBrake entry | Completion block samples | Solver error log lines | Debug samples below 0.1 m/s |
|---|---:|---:|---:|---:|---:|
| d1 | 0 | 4 | 18 | 2 | 0 / 125 |
| d2 | 0 | 6 | 3 | 1 | 0 / 133 |
| d3 | 0 | 0 | 0 | 0 | 0 / 140 |

- `collision`、`FATAL`は3台とも0件だった。
- Start後のdebug sampleでは長時間停止はなく、最低速度はd1 3.40 m/s、d2 1.98 m/s、
  d3 3.59 m/sだった。
- d1/d2はスタート直後の近接区間でSafetyBrakeへ複数回遷移したが、その後は走行を継続した。
- gap自体が左右で成立した場面でも、次のhard curveまでの利用可能距離より推定完遂距離が
  長く、completion guardが追い越し開始を拒否した。今回の配置では、壁側gap幅より
  completion distanceが主な開始ボトルネックだった。
- AWSIM Ready中に車両が動き始め、d1は公式Start通知前に1回だけOvertakeへ入った。
  ShiftOutの`speed_cap=1`は確認できたが、OSQP連続失敗がabort閾値8へ達してRecoveryへ
  移行したため、Pass速度cap解除の実走確認はできなかった。pure helperテストでは
  ShiftOut capとPass releaseの両方を確認済み。

## Conclusion and next A/B

hard safety gateを維持したまま、stage speed、path-time prediction、completion guard、
multi-front outside corridorを切替可能な形で実装できた。今回のdev3では接触や停止を
増やしていない一方、公式Start後の追い越し成立は0件だったため、完遂率向上は未確認である。

次回は同一初期条件で次を比較する。

1. A: 現設定のcompletion guard有効。
2. B: completion guardだけ無効にしたSIM限定対照群。
3. Bで追い越しが成立する場合は、利用可能距離と必要距離の分布からcurve/merge bufferを調整する。
4. Ready中のOvertake開始はスタート状態連携の別課題として切り分ける。
