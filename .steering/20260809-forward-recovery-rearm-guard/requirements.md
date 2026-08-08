# Requirements

## 背景

`output/20260809-002226/d1`では、非制限Followがstuck recoveryを阻害する問題は改善した。
一方、Forward escapeからLowSpeedRejoinを完了した約2.24秒後に、停止車を再捕捉して
coordinated recoveryが再始動し、Forward／Reverse recoveryを反復した。

- Forward escape実測距離は約0.208 m。
- `RejoinComplete`後の共通cooldownは1.0秒。
- 再始動時は新しいwall evidenceまたはsolver fallbackではなく、SafetyBrakeと
  coordinated stopが入口だった。

## 必須要件

- Forward escapeからのRejoin完了後だけ、Recovery再armを一時抑制する。
- ガードは時間または前方コース進捗で必ず解除する。
- ガード中も通常のFollow／SafetyBrake制御は変更しない。
- ガード開始後の新しい衝突、wall evidence、solver fallbackでは即座に解除する。
- Reverse recovery完了後には適用しない。
- セッション境界と新規Recovery開始時に残留状態を消去する。
- ROS 2 topic/service/message、評価基盤、加減速度上限を変更しない。
- シミュレーション競技向け設定として無効化可能にする。

## 非対象

- Forward escape距離、Reverse距離、SafetyBrake閾値の変更
- MPC solverまたはOSQP設定の変更
- V2X追い越し候補生成、Contact Continuation、Pro案rolloutの変更
- 壁・車体overlap hard guardの緩和

## Definition of Done

- pure helperで時間、距離、hard evidenceによるガード解除を確認する。
- detectorがガード中の観測窓をリセットし、解除後に新しい確認窓を開始する。
- Forward Rejoin完了時だけadapterがガードをarmする。
- 設定、起動ログ、状態ログからガードのarm／releaseを確認できる。
- package build/testと`git diff --check`が成功する。
