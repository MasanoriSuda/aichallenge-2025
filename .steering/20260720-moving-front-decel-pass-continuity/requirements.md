# Requirements

## 目的

`output/20260720-001354`で確認した、moving frontへの接近時に速度上限が突然0 m/sへ
落ちる挙動と、追い越し開始後にPassを完了できない挙動を改善する。

## 要件

- front riskが`EmergencyBrake`の場合は従来どおりSafetyBrakeを維持する。
- 停止・低速前車が停止距離内にいる場合もSafetyBrakeを維持する。
- moving frontが幾何学的停止距離内に入っただけでは速度上限を0 m/sにせず、既存の
  front-risk、Follow、ShiftOut相対速度capへ制御を渡す。
- moving frontの非Emergency状態でhazard holdを再armしない。
- hazard対象が観測され、前車が自車以上の速度で離れている場合はholdを即時解除する。
- dev3暫定設定ではhazard holdを0.25秒へ短縮する。
- 追い越し到達性ガードの横加速度上限を、実行軌道側と同じ4.0 m/s²へ揃える。
- locked Passの一時的なtarget欠落許容を0.75秒へ延長する。
- ShiftOutの最低相対速度を0.5 m/sとし、距離budget枯渇時の完全速度一致を避ける。
- ROS 2 topic、service、message、Domain、評価インターフェースは変更しない。

## 完了条件

- moving front、停止前車、EmergencyBrakeの応答をpure coreで単体テストする。
- hazard holdのsafe-moving-target即時解除を単体テストする。
- 既存のV2X overtake core testが成功する。
- `make autoware-build`が成功する。

