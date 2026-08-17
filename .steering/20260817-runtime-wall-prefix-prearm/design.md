# Design

## Path-aware wall forecast

wall warning用の将来姿勢は、現在yawの直進外挿ではなくreference trajectory上の将来点を使う。
横位置は次の優先順位で決める。

1. 実行中のFrenet DP path
2. 現在位置からfixed Mission goalへの残りShiftOut profile
3. 現在横位置の保持

横profileの勾配からheading offsetも計算し、reference trajectoryの曲率と横移動の双方を
footprint予測へ反映する。active pathが壊れている場合のみ従来の直進外挿へ戻す。

## Early local prefix

wall preplan lookaheadは0.80秒、8 samplesとする。8 m/sなら最大6.4 m先まで確認でき、
nominal 4.0 m shiftと最大1.0 m holdを警告後に評価できる。

ローカルprefixはasync same-side Missionを長時間待つ必要がないため、fallback delayを40 Hzの
1周期（0.025秒）にする。fresh candidateが間に合えば従来どおり優先し、なければ次周期で
center contractionを評価する。

prefix距離は次で制限する。

```text
available_distance = ego_speed * predicted_wall_ttc
hold_distance      = 0.5 .. 1.0 m
shift_distance     = min(configured_shift_distance,
                         available_distance - hold_distance)
```

最小0.5 m未満にはしない。短縮したprefixが横加速度制限を満たさない場合は採用せず、既存どおり
Missionを終了して反対側を含む再探索へ移る。安全制約を緩めて無理に採用する処理ではない。

## Compatibility

既存設定キーの値とdefaultを更新する。新しいROSインターフェースは追加しない。
`config.yaml`と`config_for_cloud.yaml`を揃える。
