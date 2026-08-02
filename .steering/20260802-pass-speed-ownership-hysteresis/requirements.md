# 要件

## 目的

検証済みminimum-motion Passへ入った後、横離隔の旧latch条件や単発のV2X
footprint境界揺れによってBehaviorと速度所有権が失われ、Follow/SafetyBrakeへ落ちる現象を減らす。

## 対象事象

`output/20260802-143215/d1/autoware.log`ではShiftOut 10回すべてがPassへ到達した一方、
正常なReturn完了は3回だった。Pass中には次が観測された。

- `cap_release=1`かつ`front_danger_suppress=1`でも`pass_owner=0`
- entry/curve/gapの再評価だけで`Overtake -> Follow`
- current footprint判定が制御周期ごとに境界を往復し、front-capが再適用・解除を反復
- front-cap再適用後にSafetyBrakeまたはFollowPrepareへ移行

## 制約

- 新規Entry、ShiftOut、左右選択の攻撃度は変更しない。
- front-capを未獲得のPassへ猶予を与えない。
- 継続したcurrent footprint重複は従来どおりHard abortとする。
- target消失・position jump・course progress不連続・pass側侵入・禁止waypoint・壁・solver異常を緩和しない。
- ROS 2 topic、message、launch、提出インターフェースを変更しない。

## 完了条件

- minimum-motionのfront-cap releaseをBehaviorのPass所有権でも採用する。
- release獲得後の単発current-overlapだけを設定時間デバウンスする。
- デバウンス時間を超えたcurrent-overlapではfront-capとPass所有権を解除する。
- 単体テストとビルドが成功する。

