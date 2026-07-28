# 通常 Overtake 所有権リファクタリング要件

## 目的

通常 Overtake の挙動を変えずに、状態遷移と左右選択の決定箇所を明確にする。
今後の追い越し性能修正を、既存 guard への例外追加ではなく、テスト可能な判断ロジックの変更として行える状態にする。

## 対象

- `multi_purpose_mpc_ros` の通常 Overtake
- `OvertakeLine` の active transition 優先順位
- `FollowPrepare` から再開するときの pass side 選択
- 上記判断の単体テスト

## 対象外

- 設定値の変更
- 左右選択方針そのものの変更
- SafetyBrake、wall guard、Recovery の発動条件変更
- 速度 cap / floor の計算変更
- LowSpeedAvoidance、Stuck Recovery の挙動変更
- ROS 2 topic、message、launch、評価インターフェース変更

## 挙動不変条件

1. active transition の優先順位を変更しない。
2. `FollowPrepare` 再開時は、現行どおり有効な Behavior side を優先する。
3. 通常開始時は、既存 mission side、Behavior side の順で選択する。
4. 既存の遷移 reason 文字列と主要ログを変更しない。
5. `config.yaml` を変更しない。

## Definition of Done

- active transition が純粋関数の decision として表現される。
- 再開 side の決定元が resolution に明示される。
- 現行優先順位を固定する単体テストが追加される。
- `make autoware-build` が成功する。
- 対象 package のテストが成功する。

