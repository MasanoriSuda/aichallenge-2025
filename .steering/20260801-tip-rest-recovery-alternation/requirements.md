# Requirements

## 目的

AWSIM競技シミュレーションで車両が尻もち・壁接触姿勢になった際、未脱出のまま
低速Rejoinへ移行したり、効かない同一Forwardプリミティブを無制限に再試行して
競争停止する事象を防ぐ。

## 対象事象

- `20260801-175818/d2/autoware.log` では、Reverse中に候補方向がForwardへ変化した
  周期で、Reverseを含む走行距離がForwardの0.30 m脱出条件に流用された。
- Rejoinタイムアウト後、`forward_left`が87回連続選択され、
  `aggressive_retry=86`まで進んだが復帰しなかった。

## 要件

1. 脱出確認に使う方向は、評価中の次候補ではなく実際に最後に駆動した方向とする。
2. 脱出確認距離は、現在の駆動プリミティブで実測した距離とし、反対方向の移動を
   加算しない。
3. Forward時間切れが連続した場合、静的・V2Xロールアウトを再確認した上で
   Reverseを少なくとも1回強制評価する。
4. 強制Reverse中は、aggressive force-motionのleast-bad候補選択もForwardへ
   すり抜けない。
5. 実車設定には影響させず、既存のsimulation-only aggressive recovery内に閉じる。
6. ROS topic/service/提出インターフェースを変更しない。

## 完了条件

- 方向切替時の距離誤流用を単体テストで再現し、防止できる。
- 同一Forward時間切れ2回後にReverse強制方針へ切り替わる。
- 既存のstuck recovery単体テストとパッケージビルドが成功する。

