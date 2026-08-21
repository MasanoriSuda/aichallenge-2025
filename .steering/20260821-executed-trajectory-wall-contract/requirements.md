# Requirements

## 背景

`output/20260821-222837`では、追い越しentryのMPCC参照経路が物理検証済みとして
採用されたにもかかわらず、同じ制御周期に生成された実制御予測が壁余裕不足で
却下された。

- 参照経路のplanner reserve: 1.473 m
- 実制御予測の物理壁距離: 0.124 m
- 必要な物理壁距離: 0.400 m

参照経路の証明と実際に発行するsolver解の証明が別契約になっているため、
`Idle -> ShiftOut -> FollowPrepare`を38 msで往復した。

## 目的

1. 追い越し中に実際に発行するsolver解を、発行前にswept-footprintで検証する。
2. 参照証明だけでは実行可能と扱わない。
3. 未走行entryの実行解が不成立なら、solver故障やRecoveryへ送らず安全な現経路を保持する。
4. 不成立側を一時禁止し、次周期から反対側を含む新規探索を可能にする。
5. 参照証明、実行解証明、最終出力の関係をログで追跡可能にする。

## 制約

- ROS topic、message、launch、提出インターフェースを変更しない。
- 壁余裕0.40 mを緩和しない。
- `aichallenge/result-summary.json`の既存変更を触らない。
- MPCC一本化そのものは本ステアリングに含めない。

## Definition of Done

- active `ShiftOut/Pass/Return`のsolver解が発行前の物理壁検証を通る。
- 不成立時はsolver failure counterを増やさない。
- 未走行`ShiftOut`は`FollowPrepare`で待たず、entryをロールバックする。
- ログにtarget、generation、side、phase、validation scope、required clearance、
  rejection reason、failover actionが一行で出る。
- 対象packageのbuild/testが成功する。
