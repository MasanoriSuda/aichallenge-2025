# Requirements

## 目的

`output/20260807-224341` で確認した、latch 済み forward completion が安全な
同一側 corridor と新しい相対進捗を維持しているにもかかわらず、固定の local
distance extension 回数を使い切っただけで `Recovery` へ落ちる事象を解消する。

## 必須要件

1. Pass 中は現在の target 相対位置・ego/target 速度から rear-clear までの必要距離を毎周期更新する。
2. local 距離上限到達時、必要距離と必要時間が残りの absolute Pass 上限内なら同一側の完遂を継続する。
3. 動的延長には新しい実測相対進捗、車体・予測 footprint 非重複、有効 corridor を必須とする。
4. absolute Pass 上限、壁、確定車体重複、持続予測重複、EmergencyBrake、solver recovery は緩和しない。
5. 既存の通常 progress extension 回数制限と 0.25 秒予測重複 debounce は維持する。
6. ROS 2 topic/service、車両寸法、加減速度、評価インターフェースは変更しない。

## 完了条件

- pure policy test で通常延長回数を使い切った後の動的延長を確認する。
- 残り absolute 距離・時間不足および short-horizon unsafe では延長しないことを確認する。
- `multi_purpose_mpc_ros` の build/test が成功する。
