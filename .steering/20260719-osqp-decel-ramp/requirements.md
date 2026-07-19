# Requirements

## Purpose

`make dev3`走行中に発生する急な失速を、OSQP失敗とV2X速度上限の不連続に分けて改善する。

## Scope

- OSQP失敗ログへsetup/solve/status/iteration/residual等の原因情報を残す
- SafetyBrake、Follow、Overtake Recovery等の動的速度上限を到達可能な減速ランプにする
- 既存の最大減速度`a_min`、V2X状態判定、ROS interfaceは維持する

## Acceptance criteria

- OSQP失敗時に一律`OSQP failed`ではなく失敗stageと利用可能なsolver情報を出す
- 現速度より低い速度上限の先頭値は`a_min * control period`以内の変化になる
- horizon後半は設定された最終速度上限へ収束する
- pure core単体テストと`make autoware-build`が成功する
- `make dev3`で走行中OSQP失敗件数と急減速状態遷移を再計測する
