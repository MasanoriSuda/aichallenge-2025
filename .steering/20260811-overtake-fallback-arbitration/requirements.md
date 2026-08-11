# Requirements

## 目的

追い越し経路の継続に失敗した際、成立済みの別側候補があるのに過去の no-return latch だけで再選択できず、直ちに Recovery へ落ちて失速する挙動を解消する。

## 要件

- SafeSeparation 中でも、相手が十分前方へ離れ、車体・予測 sweep・壁・corridor が安全な場合に限り、未使用の別側 Mission を一度だけ再選択できること。
- 横並び中、接触中、EmergencyBrake、壁接触、solver recovery、target 不連続では再選択しないこと。
- 別側候補は既存の完全 preflight と残時間・残距離・最低速度・壁余裕の再検証を通すこと。
- 次善 Mission が成立しない軟失敗では、相手が十分前方かつ物理的に分離済みなら Recovery 速度制限を掛けず通常 Follow へ戻すこと。
- hard fault は従来どおり Recovery とすること。
- 同じ失敗側を即再試行しないよう side retry cooldown を維持すること。
- ROS 2 topic、message、launch、評価インターフェースを変更しないこと。

## 対象外

- gap、壁余裕、加速度などの攻撃度パラメータ変更
- 接触時の hard-fault 分類変更
- 追い越し候補生成・相手予測モデルの全面改修
- 実車向け挙動の保証

