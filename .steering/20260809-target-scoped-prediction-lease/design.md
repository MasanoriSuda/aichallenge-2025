# Design

## Target-scoped stopped evidence

`StationaryBlockerEntryOverrideRequest`へ停止証拠とMission targetの同一性を明示する。controllerは停止確認を蓄積したIDを`V2XBehaviorOutput`へ渡し、現在の`target_vehicle_id`との一致を確認する。速度上限には停止確認と同じ`low_speed_avoidance_max_front_speed`を使う。

## Typed prediction failure

`PassRefreshFailureReason`を導入し、lease requestはboolや文字列ではなくtyped reasonを受け取る。same-side refreshの各失敗を分類し、lease可能なのは`TargetPredictionUnavailable`だけとする。汎用`PassHorizonAction::Abort`からavailabilityだけを見てprediction lossへ変換しない。

## Lease speed ownership

lease開始時の自車速度を保存する。lease中は速度floorを無効化し、速度referenceを「lease開始時速度以下」に制限する。これは急制動を要求する停止車速度capではなく、予測欠損中に新しく加速を積み増さないための所有権である。fresh prediction復帰時にlease状態を全て解除する。

## 非対象

- イン／アウト候補ranking
- Recovery後のfailure fingerprint
- 停止車横並び専用Recovery profile
- Mission commit-levelの再構成

