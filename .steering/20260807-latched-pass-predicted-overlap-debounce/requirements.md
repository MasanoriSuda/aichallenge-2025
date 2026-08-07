# Requirements

## 目的

`output/20260807-091548` で確認した、latch 済みの side-by-side forward completion が
単発の予測 footprint 重複で即座に `Recovery` へ落ちる事象を局所的に解消する。

## 必須要件

1. 初回の forward completion 認可では、従来どおり予測 footprint sweep の非重複を必須とする。
2. 認可済みの forward completion に限り、予測上だけの重複は既存の 0.25 秒連続確認後に中止する。
3. 予測が再び非重複になった場合は確認時間をリセットする。
4. 現在車体の確定重複、壁異常、target discontinuity、pass-side intrusion、EmergencyBrake、solver recovery は即時中止を維持する。
5. 車両寸法、壁余裕、加減速度、ROS 2 topic/service、評価インターフェースは変更しない。

## 完了条件

- pure policy test で初回認可と latch 後保持の境界を確認する。
- 0.25 秒以上継続した予測重複では latch を解除できることを確認する。
- `multi_purpose_mpc_ros` の build/test が成功する。

