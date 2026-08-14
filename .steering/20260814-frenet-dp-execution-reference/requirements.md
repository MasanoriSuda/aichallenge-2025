# Requirements

## 目的

`20260814-continuous-frenet-dp-corridor` で導入したFrenet DPは、現状では候補評価と
固定goal不成立時の単点bridgeにだけ使われている。実走では全DP候補が
`bridge=0`のまま、従来の単一 `pass_lateral` Missionを実行し、10試行中の正常完遂は
1回だった。

選択したDP横位置列をShiftOutからrear-clearまでの実行参照へ接続し、既存の
receding-horizon optimizerが時系列回廊に沿って壁・相手制約を再最適化できるようにする。

## 必須要件

- 選択候補へDPの距離列・横位置列をselection-transparent metadataとして保持する。
- freeze/replacement時にDP列をactive Missionへatomicに移す。
- ShiftOut/PassではMission進行距離に合わせてDP列を既存MPC horizonへリサンプルする。
- DP列は参照値にだけ使い、壁、車体、横加速度、target bounds、no-return guardを迂回しない。
- DP列が不正、不成立、side不一致、coverage切れの場合は従来の単一goal参照へ戻す。
- Return、Recovery、start-grid breakoutの既存経路を変更しない。
- `/control/command/control_cmd` 等のROS 2インターフェースを変更しない。

## 非対象

- longitudinal solverの置換
- Recoveryアルゴリズムの変更
- Return経路のDP化
- 30回連続OSQP失敗の修正

## Definition of Done

- DP実行参照のvalidation、進行距離resample、coverage切れfallbackの単体テストが通る。
- runtime logでDP実行参照のactive/coverage/traveledを確認できる。
- `multi_purpose_mpc_ros` がビルドできる。
- package testが0 failureで完了する。
