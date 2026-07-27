# Results

## 実装結果

- commit済みの`ShiftOut` / `Pass`について、新規追い越し候補用の
  curve、completion距離、cooldown、soft forbidden判定を再適用しないようにした。
- locked targetの連続性、前方判定、pass側侵入、live corridor、
  明示禁止waypoint、Emergencyをcommit後の継続判定として分離した。
- pass側侵入と明示禁止waypointのRecovery reasonを具体化した。
- 速度、壁margin、横加速度、開始条件のパラメータは変更していない。
- actual footprint wall、static wall、横加速度、solver failure、
  target位置ジャンプ、観測期限切れのhard guardは維持した。

## 静的・単体検証

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
  - 650 tests
  - 0 errors
  - 0 failures
  - 0 skipped
- `git diff --check`: 成功

## 実走で確認する項目

`make dev2`のP1ログで次を確認する。

1. `reason=locked target no longer executable`が大幅に減ること
2. `same target gap reacquired during recovery`の直後再試行が大幅に減ること
3. `ShiftOut -> Pass`の比率が上がること
4. `Pass -> Return -> Idle`が成立すること
5. 壁、corridor、pass側侵入、明示禁止waypointによるhard guardが残ること

実走による追い越し成功率と接触有無は未確認であり、ユーザー確認を残作業とする。
