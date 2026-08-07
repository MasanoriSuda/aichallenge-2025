# Requirements

## 目的

2026-08-07 の `make dev2` ログで確認した、追い越し完遂直前の失速と誤った pre-arm 引き継ぎを局所的に解消する。

## 対象事象

1. side-by-side forward completion が成立して SafeSeparation に入った直後、距離推定の再計算結果が閾値を外れ、`target clear ahead` で FollowPrepare に戻る。
2. pre-arm の速度確認時間が target ID だけに紐付き、Mission が消えた期間や左右 Mission が変わった後にも引き継がれる。

## 制約

- 車両寸法、壁余裕、加減速度などの攻撃度パラメータは変更しない。
- 実接触、予測 footprint 重複、壁異常、EmergencyBrake、solver recovery は従来どおり fail closed とする。
- ROS 2 topic/service、提出・評価インターフェースは変更しない。

## 完了条件

- 一度認可した forward completion は rear-clear または hard guard まで維持される。
- latched completion 中に `target clear ahead` だけを理由に FollowPrepare へ戻らない。
- pre-arm は target/side/closing speed が同一の有効 Mission にだけ蓄積される。
- Mission 消失・変更・時間/距離上限で pre-arm をリセットする。
- pure core test と package build/test が成功する。
