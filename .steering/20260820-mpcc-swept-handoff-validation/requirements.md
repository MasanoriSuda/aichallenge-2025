# Requirements

## 背景

`output/20260820-230700` では、追い越し計画が 1.27--2.57 m の
Frenet corridor reserve を持つとして採用された後、実行予測では車体と壁の
物理距離が 0--0.35 m となり、必要な約 0.40 m を満たさない事象が 10 回発生した。
うち 1 事象では ShiftOut 中断後に実車体の壁接触へ至った。

通常の solved-MPCC execution source 検証は horizon の離散姿勢だけを確認し、
現在の実測姿勢から最初の姿勢までを含む連続 swept path を検証していない。
このため、端点が安全でも接続区間が壁を横切る解を一時的に採用できる。

## 目的

- solved-MPCC execution source を採用する前に、現在姿勢から全 horizon までを
  物理 footprint で連続検証する。
- 新解が不合格なら、既存の安全な execution authority を上書きしない。
- ログだけで離散検証か連続検証か、新解か last-feasible か、棄却理由は何かを
  判別できるようにする。

## 制約

- ROS 2 topic、service、message、launch、提出物の契約を変更しない。
- wall clearance の設定値は変更しない。
- Recovery や追い越し戦術の機能追加は行わない。
- `output/`、rosbag、既存ユーザー変更を編集しない。

## Definition of Done

- 通常の latest / last-feasible / stitched handoff がすべて swept validation を通る。
- swept validation 不合格時に新解が promote されない。
- promote / pending ログに検証範囲が明示される。
- 関連単体テストとパッケージビルドが成功する。
