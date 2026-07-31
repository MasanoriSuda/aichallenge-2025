# Committed Pass horizon continuity 性能修正要件

## 目的

Pass で一度獲得した front cap release を、一時的な execution horizon 不成立だけで
失わないようにし、低速・停止車両を抜き切る途中の不要な速度低下を減らす。

## 対象

- `multi_purpose_mpc_ros` の通常 Overtake
- `CommittedPassPolicy` の既存 release hold 条件
- hold が外れる安全条件の単体テスト
- 実走で効果を判定できる既存 debug 出力

## 性能変更

Pass 中に次をすべて満たす場合、execution horizon が一時的に infeasible でも
既存の front cap release を維持する。

1. Pass phase である。
2. lateral exclusion が latch 済みである。
3. front cap が既に release 済みである。
4. locked target との車体横離隔が成立している。
5. reapply 用横離隔閾値を維持している。
6. locked target の位置飛びがない。
7. 実壁接触がない。
8. target が有効に観測されている。

## 変更しない安全条件

- ShiftOut または初回 release では、従来どおり実行可能な horizon を要求する。
- 横離隔が reapply 閾値未満なら cap を再適用する。
- target position jump、target loss、実壁接触時は hold しない。
- committed Pass 速度 floor は、従来どおり実行可能な経路がある場合だけ使う。
- wall / lateral acceleration / Recovery / MPC hard limit は引き続き速度権限を持つ。
- `config.yaml` は変更しない。

## 対象外

- wall Recovery の発動条件変更
- pass corridor、左右選択、Persistent Mission の変更
- 設定値のアグレッシブ化
- ROS 2 topic、message、launch、評価インターフェース変更

## Definition of Done

- physically committed Pass が horizon 不成立だけでは cap を再適用しない。
- 初回 release、ShiftOut、横離隔喪失、位置飛び、実壁接触の既存保護をテストで確認する。
- `make autoware-build` が成功する。
- `multi_purpose_mpc_ros` のテストが成功する。

