# Requirements

## 目的

Progressive Entryで追い越し開始率は上がった一方、body-clear直後の同側経路が未検証のstatic fallbackへ入り、壁違反・SafeSeparation・Recoveryへ遷移する事象を減らす。

## 要求

- Complete Missionの選択・優先順位は変更しない。
- Progressive Entryはbody-clear予測後、同じ側を6.0 m継続できる場合だけ許可する。
- Progressive Entryにはbody-clear deadlineの余裕を0.6秒以上要求する。
- static fallbackのProgressive Entry横移動上限を2.2 mから1.8 mへ下げる。
- entry準備用の速度評価は維持し、Progressive Entry不成立だけを理由に準備処理を失わない。
- hard wall clearanceは0.15 mから0.20 mへ小幅に増やす。
- 横加速度上限6.0 m/s^2、Complete Missionの選択条件は変更しない。
- ROS topic/service/launchおよび評価基盤は変更しない。

## 対象外

- Return/rear-clearを含む完全Mission検証の緩和
- Recovery/Reverseの変更
- wall hard violationの無視
