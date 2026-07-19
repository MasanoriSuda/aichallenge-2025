# Results

## Verification

- `make autoware-build`: 25 packages成功。
- `test_recovery_mpc`: 6/6成功。Forward/Reverse符号、曲率feedforward、操舵変化上限、
  invalid configのfail-closedを確認した。
- package全体: 18 target中17 target成功。失敗1件は作業開始前から変更中の
  `traj_mincurv.csv`が閉路重複終点を持たないことによる`test_path_core`で、Recovery変更外。

## dev3: `output/20260719-121902`

Recovery MPCを有効にして約2分20秒監視した。起動ログで3 Domainとも
`recovery_mpc=enabled/10x0.20 m/beam=48`を確認した。

- D1: Reverse候補をMPCで`-0.10 rad`へ誘導したが、D2とのV2X overlapで0.019 m後に中断。
  その後Forwardを5 step、episode 0.300 mまで実行してLowSpeedRejoinへ入った。
  MPCは`-0.088 rad`を出したが実速度は0.001〜0.003 m/sのまま、`e_y`は
  2.594 mから2.737 mへ悪化し、5秒で`rejoin_timed_out`。最後はD2にcorridorを塞がれ
  `clearance_wait_timed_out`でSafeStopした。
- D2: Mixed contactに対しForwardRightを9 step実行した。contactは一時減少したが、
  episode移動は0.179 mに留まり、次候補が`contact_worsened`となって
  `maneuver_direction_unknown`でSafeStopした。
- D3: `e_y=2.565 m`、`e_psi=2.525 rad`の大姿勢誤差かつoccupancy grid外となり、
  rolloutを生成できず`maneuver_direction_unknown`でSafeStopした。
- `rejoin_complete`: D1/D2/D3すべて0件。
- 強制rejoin、無制限retry、安全gate bypass: 0件。

## Decision

Recovery MPC候補は不採用。複数点予測そのものは動作したが、今回の停止原因は、車両同士が
物理的に噛んで目標加速度でも動かないこと、深いmap contact、map外の3種類であり、操舵最適化だけでは
解除できなかった。`stuck_recovery.recovery_mpc.enabled: false`を正式設定とし、元のbounded
stepwise Recovery + P rejoinを有効状態に戻した。planner実装とunit testは次のA/B用に残す。
