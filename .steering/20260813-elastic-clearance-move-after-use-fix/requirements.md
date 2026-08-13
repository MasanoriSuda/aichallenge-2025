# Requirements

## 目的

`20260813-200341` の2周目に発生した `mpc_controller_cpp` の
SIGSEGVを解消し、制御ノード停止に起因する逸走を防ぐ。

## 原因

弾性クリアランスの採用判定で、`candidate_horizon`を
`validated_horizon`へmoveした後に、移動済みの
`candidate_horizon.target_ey`を参照していた。

## 制約

- 追い越し軌道とクリアランス判定の仕様は変更しない。
- ROS 2 topic、message、launch、評価インターフェースは変更しない。
- 修正はmove後参照の除去に限定する。

