# Requirements

## 目的

`20260814-frenet-dp-execution-reference`でFrenet DP列を実行参照へ接続したが、
`output/20260814-234659/d1/autoware.log`では全てのDP列が5点、約6.8〜10 mで終了した。
開始時点でも20点のMPC horizon中8〜10点しか覆わず、ShiftOut後半またはPass中に
従来の単一横goalへ戻り、7試行中5回が
`optimized horizon escaped target separation bounds`によるDynamicMissionWaitへ入った。

target占有区間だけでなく、同じGapPlanner計算で得た前後の静的壁回廊までDPへ渡し、
rear-clearに必要な距離を覆う実行参照を作る。また、active Missionと同一target・同一sideの
新しい可解DP列を走行中にatomic更新し、一時的に再計画できない周期は直前の可解列を保持する。

## 必須要件

- Missionの動的target制約の意味は変えず、DP専用profileだけをGapPlanner全horizonへ延長する。
- target非占有区間は静的壁回廊とrobust wall clearance内に限定する。
- DP実行列はShiftOut/Pass中、同一target・同一side・fresh predictionの場合だけ更新する。
- 更新は距離列と横位置列を検証後にatomic commitし、Mission generationやPass進捗をリセットしない。
- 新しいDP列が不正・古い・side不一致の場合は、現在の可解列を保持する。
- wall、target、横加速度、no-return、Recoveryのhard guardを緩和しない。
- Return、start-grid breakout、ROS 2 interfaceを変更しない。

## 非対象

- `v2x_prediction_time`の変更
- 左右branch authorityの変更
- longitudinal solver、Recovery、Returnの変更
- full nonlinear MPCCへの置換

## Definition of Done

- DP full-horizon extensionとrolling refresh admissionの単体テストが通る。
- runtime logでDP path点数・距離とrolling refresh回数を確認できる。
- `multi_purpose_mpc_ros`がビルドできる。
- package testが0 failureで完了する。
