# Requirements

## Purpose

復帰処理の性能修正を追加する前に、`LowSpeedRejoin`の進捗監視と
Forward/Reverse候補選択方針を分離し、状態や真偽値の意味を局所的に追えるようにする。

## Evidence

`output/20260801-112425/d2/autoware.log`では、P2がLowSpeedRejoin中に
中心線を通過した後も操舵を継続して横偏差を悪化させ、その後Forward候補を優先した
再評価が反復した。現行コードでは次の責務が呼び出し元とSupervisorに分散している。

- 正規化したRejoin整列誤差と最終進捗時刻の更新
- Forward候補を評価してよいかという許可
- Forward候補を最初に選ぶかという優先順位

## Constraints

- 今回は挙動を変えない。閾値、タイムアウト、候補順序、状態遷移、設定値を維持する。
- ROS topic/service/message、gear、launch、評価出力の契約を変更しない。
- `aichallenge_system`、生成済みresult、rosbagには触れない。
- 次の性能修正で、Rejoin横偏差悪化と方向固定を各部品内で扱える構造にする。

## Definition of Done

- Rejoin進捗の状態保持と計算が専用トラッカーに分離される。
- Forward候補の評価許可と優先条件が型付きの方針として分離される。
- 既存条件を固定する単体テストが追加される。
- 対象packageのテスト、ビルド、`git diff --check`が成功する。
