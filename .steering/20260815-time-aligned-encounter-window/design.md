# Design

## 観測結果

`output/20260815-170645/d1/autoware.log`では、19 episode中、
`Pass -> Return`は2回だった。一方、target-bound系の失敗遷移は13回あり、
target-bound prefixは4回作動したがfresh replacementは得られなかった。

## 予測方式

各sampleについて次を計算する。

```text
sample_time = path_distance / candidate_ego_speed
relative_rate =
  (target_longitudinal_at_base_horizon - target_longitudinal_now)
  / base_prediction_horizon
target_s(sample_time) =
  target_s_now
  + relative_rate * sample_time
  + (nominal_ego_speed - candidate_ego_speed) * sample_time
```

横位置は既存の1秒予測まで補間し、それより先では予測終端を保持する。
横速度を長時間外挿して別車線まで動かす誤予測を避けるためである。

## Encounter window

- `sample_time <= encounter_prediction_max_sec`のsampleだけ予測対象にする。
- その中で`abs(target_s) <= 車体前後半長 + buffer`のsampleだけtarget
  separation boundを有効化する。
- 最大時間より遠いsampleは壁制約とMission参照だけで解き、次周期の
  receding horizonで再評価する。
- 初期値は上位ログに合わせて3.0秒とする。

## 診断

target-bound prefix開始時に次を追加表示する。

- target constraint sample数
- 最後にtarget制約が有効だったpath距離
- 最大予測時間より先として除外したsample数
- horizon内の最大sample到達時刻

これにより、失敗が近傍の実制約か遠方予測の過拘束かを区別する。
