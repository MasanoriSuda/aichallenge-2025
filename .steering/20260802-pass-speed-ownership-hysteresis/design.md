# 設計

## 1. Pass所有権の判定統一

現行のminimum-motion Passは、固定1.5m横離隔より先に、現在車体と予測sweepが
非重複ならfront-capを解除できる。しかしBehavior ownerは旧横離隔latchだけを要求する。

Behavior ownerの物理コミット条件を次のORへ変更する。

```text
legacy lateral exclusion latch
OR
minimum-motion front-cap release latch
```

これにより、同じ周期で速度側はPass、Behavior側はFollowという不整合を解消する。

## 2. current footprint overlapの短時間デバウンス

V2X更新周期と40Hz制御周期の境界では、車体矩形が接する付近で
`separated / overlap`が短周期に往復する。release獲得済みのminimum-motion Passに限り、
current-overlapが設定時間連続するまで直前のreleaseを保持する。

```text
Pass + fixed minimum-motion corridor
+ prior front-cap release
+ target continuity valid
+ attack mode enabled
+ current overlap duration < confirm_sec
  => Pass/front-cap/front-danger suppressionを保持
```

初回releaseの獲得には猶予を使わない。継続重複が`confirm_sec`へ達した場合は、
front-cap再適用とSafetyBrakeを再び許可する。

既定の確認時間は0.10秒とする。約2回のV2X更新を要求し、0.25秒の予測重複確認より短くする。

## 3. 安全責務

猶予は現在車体重複の判定だけに限定する。次は常時Hard abortである。

- target continuity不成立
- position jump / course progress rejection
- pass側侵入
- 明示的な禁止waypoint
- actual wall contact / 実行経路不成立
- solver recovery要求
- current overlapが0.10秒以上連続

## 4. ログ

既存の周期debugとfront-cap遷移ログへ次を追加し、ログ行数は増やさない。

- `current_overlap_confirmed`
- `current_overlap_elapsed/confirm_sec`
- `current_overlap_grace`

