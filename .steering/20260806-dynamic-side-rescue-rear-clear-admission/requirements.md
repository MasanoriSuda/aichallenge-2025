# Requirements

## 背景

`fcf2571` では、横並び後の不用意な速度 cap 再適用を避けるため、同じ側を維持して前進完遂する処理を追加した。しかし `output/20260806-233616` では、5 回の forward completion のうち完遂は 1 回で、残りは実車体重複、予測重複、壁制約、SafetyBrake へ遷移した。

また、横並び前に現在側の予測 sweep が重複すると、反対側の全 Mission を評価する前に side replan 自体が禁止されている。そのため、空いた反対側を利用できず、現在側の失敗を待つ構造になっている。

## 必須要件

1. no-return 前かつ現在車体が非重複なら、現在側の予測重複を反対側 Mission 評価のトリガーとして扱う。
2. 反対側へ切り替える場合は、既存どおり ShiftOut・Pass・Return を含む全 Mission が成立したものだけを atomic に採用する。
3. 横並び後の forward completion は、予測 sweep が非重複で、現在速度差から rear-clear まで到達できる距離がローカル上限内に収まる場合だけ許可する。
4. 予測不能、確定車体重複、壁接触、緊急制動、solver recovery は fail closed を維持する。
5. Overtake entry pre-arm の速度差は、採用済み Mission の closing speed と一致させる。
6. ROS topic/service/message と評価インターフェースは変更しない。

## 対象外

- 壁境界モデルや MPC solver の全面的な変更
- 車両寸法、壁余裕、最大加速度などのパラメータ攻撃化
- no-return 後の左右横断
- 評価基盤 `aichallenge_system` の変更

## 完了条件

- pure policy の単体テストで、予測重複時の反対側救済と、rear-clear 距離不足時の forward completion 拒否を確認する。
- `multi_purpose_mpc_ros` がビルドできる。
- 対象テストが成功する。
