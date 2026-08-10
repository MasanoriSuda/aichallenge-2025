# Requirements

## 目的

`output/20260811-000600` で確認した、外側として開始した Pass が rear-clear 前に
内側へ変化して SafeSeparation へ落ちる事象と、無効 Mission を FollowPrepare で
4 秒保持して反対側の再選択が遅れる事象を局所的に解消する。

## 必須要件

- predicted rear-clear の直後に起きる曲率反転も初期 Mission 選択で検出する。
- 外側継続が無効になり、相手が十分前方へ離れた場合は、SafeSeparation の終端を
  待たずに fresh な反対側 Mission を再検証できるようにする。
- 反対側候補は、現在 pose から壁・車体・横加速度を再度 preflight してから置換する。
- dynamic Mission wait は短時間で打ち切り、古い side lock を解放して両側を再比較する。
- 実 overlap、壁接触、EmergencyBrake、solver recovery では従来どおり fail closed とする。
- ROS topic/service、評価結果、提出物のインターフェースを変更しない。

## 対象外

- 速度・加速度上限の変更
- 車体寸法・壁余裕の緩和
- フル MPCC への置換
- recovery/reverse の変更
