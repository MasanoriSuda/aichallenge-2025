# Requirements

## 背景

`output/20260808-235118/d1`では、前方車が逸走してV2X速度制限が解除された後も、
停止中のP1が通常走行へ戻らない区間がある。

- `Follow`表示でも`limit=inf`、`follow_cap=0`となる区間がある。
- 現行stuck detectorは`Follow && has_front_vehicle`を一律`deliberate_stop`として扱うため、
  実際には前進要求があるsolver failureでもRecovery確認時間を開始できない。
- Passではtargetが既に後方かつ同側経路上で前進中でも、wall-clockの絶対時間上限により
  SafeSeparationからRecoveryへ落ちる場合がある。

## 必須要件

- Follow表示だけを理由に意図的停止と判定しない。
- Followが速度上限または減速要求を実際に所有している場合は、従来どおり意図的停止として扱う。
- targetが後方、同側の短期経路が安全、前方進捗がfreshなPassでは、時間上限到達後も
  絶対距離上限の範囲でrear-clearまで前進を継続する。
- 進捗停止、短期経路不成立、車体・壁のhard fault、絶対距離上限では従来どおり中断する。
- ROS 2 topic/service/message、評価基盤、加減速度上限を変更しない。
- 2025 AWSIM競技シミュレーション向け挙動は設定で無効化可能にする。

## 非対象

- MPC solver本体またはOSQP設定の変更
- 壁・車体overlapのhard guard緩和
- 逸走車のV2X tracking履歴削除
- AWSIM衝突ペナルティの解除

## Definition of Done

- 非制限Followがstuck recoveryの観測を妨げないことをpure unit testで確認する。
- 実制限中のFollowはdeliberate stopを維持する。
- 後方targetへのfreshな前進中だけ時間上限を距離・進捗判定へ切り替える。
- 進捗なし、前方target、距離上限、unsafe horizonでは従来どおりAbortする。
- `multi_purpose_mpc_ros`のbuild/testと`git diff --check`が成功する。
