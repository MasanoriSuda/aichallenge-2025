# Results

## 実装結果

- cooldownと制御再採用を`ExtendedSolverReentryGate`へ分離した。
- extended solve / conversion / build failure後はrequalificationを要求する。
- cooldown終了後の成功解は3周期連続までshadow probeとし、その間は3-state MPCCを継続する。
- 3周期目の成功からextended解をatomicに再採用する。
- velocity mode handoffを0.15秒から0.30秒へ延長した。
- runtime telemetryへ`requalifying`と`reentry_streak`を追加した。

## 静的検証

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- `test_mpcc_progress`: 34/34成功
- package CTest: 28/28成功

`ament_uncrustify`は巨大な既存controller全体に既存style divergenceを報告したため、全体の
機械整形は行っていない。今回差分は既存近傍のstyleへ合わせ、`git diff --check`とbuildで確認した。

## 次回試走で見る項目

1. `Extended MPCC runtime`の`requalifying`がfailure後に最大2周期現れること。
2. `reentry_streak=0/3`へ戻った後、extendedの`success`が継続すること。
3. ShiftOut / Pass中の`failure -> circuit_skip -> success -> failure`反復が減ること。
4. `Steering debug`の切替直後に操舵飽和が増えていないこと。
5. `Pass -> Recovery reason=solver failure threshold reached`が減ること。

## 未検証

AWSIM動的試走はユーザー実施。今回の変更はモデル切替チャタリングを対象とし、
`physical target separation conflicts with wall bounds`などの経路成立性問題は別課題として残す。
