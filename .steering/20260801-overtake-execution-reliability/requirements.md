# Overtake execution reliability 改善要件

## 目的

通常 Overtake が開始時に選んだ経路を、物理的に実行可能な形で Pass まで完遂する
割合を上げる。競走停止・復帰系は本作業の対象外とする。

## 対象ログ

`output/20260731-235231/d1/autoware.log`

同走行では4回の ShiftOut のうち2回が完遂し、残る2回は次の理由でRecoveryとなった。

1. `static wall clamp exceeds lateral acceleration limit`
2. `solver failure threshold reached`

## 必須修正

### 壁clamp後の到達可能性

- 壁余裕付きtargetをclampした結果だけが横加速度上限を超えた場合、即Recoveryにしない。
- 横加速度上限内の中間targetを作り、車体実寸のstatic map検査を再実行する。
- 中間targetが車体実寸で非接触の場合だけ、一時的なwall-margin縮小を許可する。
- 中間targetも不成立、actual footprint接触、map不明の場合は従来どおりRecoveryとする。

### ShiftOut中の左右反転

- 旧pass side方向への横速度が残っている間は、反対側への直接replanを禁止する。
- 単なるquality差なら旧sideを維持する。
- 選択sideが実際に塞がっている場合は直接横断せずRecoveryへ移行する。
- 直接replanを許可する横速度閾値はparam yamlで調整可能にする。

## 変更しない条件

- actual footprint wall contactとwall sample failureは緩和しない。
- SafetyBrake、Emergency、front-risk、MPC hard boundsを変更しない。
- solver failureの成功判定や最大反復数を変更しない。
- Pass速度継続、左右候補のquality計算、追従距離を変更しない。
- ROS 2 topic、message、launch、評価インターフェースを変更しない。
- P2の競走停止、Stuck Recovery、リバース速度は変更しない。

## Definition of Done

- 壁clamp後に到達可能かつ車体実寸で安全な中間targetを継続利用できる。
- unsafeな中間targetは従来どおりRecoveryになる。
- 旧side方向へ横移動中の直接side反転が抑止される。
- `make autoware-build`が成功する。
- `multi_purpose_mpc_ros`の全テストが成功する。
