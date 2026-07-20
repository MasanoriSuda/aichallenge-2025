# Design

## 現象

`output/20260720-164126/d2/autoware.log`では直接制御が`speed=3.00`で開始し、
停止車両のrear clearanceは約23秒で成立した。一方、解除には開始時の
`target_ey=-2.61`へのpose settleも必要だったため、latchは63.4秒継続した。
解除直後にはOSQPが25周期連続で失敗した。

## 状態

- Pass: 停止車両列が残る間、選択したpass targetを直接追従する。
- Rejoin: 車両列clear hold成立後、reference pathの有効横制約内で0 mに最も近い
  横位置を直接追従する。
- Handoff: Rejoin pose成立後にMPCを試行し、成功時だけ通常MPCへ戻る。

Rejoin中に関連車両が再検出された場合はPassへ戻し、保存したpass targetを復元する。

## Solver handoff

`init_problem()`は従来どおり毎周期MPC問題を構築する。Rejoin pose成立後、
`get_control()`でその問題をprobe solveする。

- 成功: latch状態をクリアし、同じsolutionを通常制御出力に使う。
- 失敗: failure counterを増やさず、Rejoin直接制御を継続する。

これにより不成立MPCへの切替で減速fallbackへ入ることを避ける。

## 影響範囲

- `mpc_controller_cpp.cpp`: 状態、再合流、solver handoff。
- `v2x_overtake_core.*`: clear hold判定の純粋関数。
- `test_v2x_overtake_core.cpp`: 判定境界テスト。
- `docs/spec/mpc-integration.md`: 暫定制御仕様。

