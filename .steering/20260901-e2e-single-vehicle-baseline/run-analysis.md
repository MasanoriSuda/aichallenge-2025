# E2E Single-Vehicle Run Analysis

## Summary

起動・モデルartifact・LiDAR入力・制御出力の統合は成立した。既存checkpointは
0.6 m/s²で2周を再現して走れるが、3周連続Gateには未合格である。

## Evidence

### `output/20260901-020346`

- 先行300秒Gate。
- lap: 103.95秒、90.61秒。
- 3周目途中で設定timeout。LiDARはbag未収録だったため後続runで修正。

### `output/20260901-021032`

- 420秒Gate、0.6 m/s²。
- lap: 103.64秒、90.66秒。
- LiDAR 750点、約20 Hz、8483 messages。
- control 8484 messages、accelerationは全て0.6 m/s²。
- inference stale / exceptionは0。
- 3周目で壁際に停止し、184.55秒間0.1 m/s未満。
- 低速区間開始はbag開始239.80秒後。
- 直前10秒の最小LiDAR距離中央値0.159 m、停止後は約0.146 m以下。
- 右側に開放空間があっても操舵出力は約0.03 radへ縮小した。

### `output/20260901-022146`

- 配布設定0.3 m/s²の反証A/B。
- スタートライン前で速度ほぼ0となり、加速0.3、操舵-0.215 radのまま回復不能。
- 0.6の失敗を速度域だけでは説明できず、失敗状態の教師データ不足が主因と判断。

## Topic Chain

`/sensing/lidar/scan`からTinyLidarNetを経て
`/control/command/control_cmd`まで、欠損なく20 Hzで継続した。GNSS publisherは
同梱AWSIMのReady/Grounded初期化にだけ使われ、TinyLidarNetのsubscriberはLiDARと
`/clock`だけである。

## Root Cause Classification

インフラ停止や推論例外ではない。壁へ接近した分布外状態で、開いた側へ戻る操舵を
出せず、同じLiDAR状態と小操舵を自己生成し続けるclosed-loop covariate shiftである。
固定加速度だけを調整しても根治しない。

## Next Slice

1. scan/control同期誤差を閾値でrejectし、常に保存する。
2. train/validationをsampleではなくrun単位で分割する。
3. MPC教師をshadow記録できる起動経路を用意する。
4. 壁接近・逸脱直前・復帰区間をfailure/recovery datasetとして追加する。
5. 同じ3周Gateで再評価する。
