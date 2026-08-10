# Requirements

## 目的

`output/20260810-193518` で、完全な ShiftOut/Pass/Return Mission が成立し、
自車が低速前方車へ実測で追いつき始めても、通常の 0.3 s 速度確認中に車間が
SafetyBrake 領域へ入り、Overtake を一度も開始できなかった事象を修正する。

## 要求

- 通常の新規 Overtake は、従来どおり実測相対速度を連続確認する。
- 現在の完全な Mission と hard guard が成立している場合に限り、近距離の低速車へ
  短い確認時間で Overtake を開始できる。
- 緊急制動、無効な V2X、位置飛び、禁止 waypoint、solver recovery、Mission 不成立、
  最低開始距離未満では早期開始しない。
- 実壁接触を伴う Recovery の reverse-only 保護は変更しない。
- 判定は純粋関数として単体テスト可能にする。

## 変更範囲

- `multi_purpose_mpc_ros` の V2X Overtake entry admission
- 対応する設定、起動ログ、単体テスト

## 変更しないもの

- ROS 2 topic/service/message 契約
- 評価基盤
- Recovery -> Overtake direct handoff の solver/reverse-only gate
- 車両寸法と壁 clearance
