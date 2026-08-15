# Design

## 方針

新しい状態機械は追加しない。既存の MPCC-lite/Frenet DP、左右 shadow 評価、
last-feasible maneuver、target-bound physical prefix を接続し、Pass 実行を
receding-horizon として継続させる。

## 変更点

### 1. Runtime SafeSeparation budget refresh

SafeSeparation 開始後、毎周期得られる有限かつ feasible な rear-clear rollout から
必要な残距離・残時間を受け取る。局所 window の start から見た必要量へ変換し、
現在値より大きい場合だけ更新する。

```text
required_local_distance = local_traveled + runtime_remaining_distance
required_local_time     = local_elapsed  + runtime_remaining_time

local_limit = min(max(old_limit, required_local),
                  local_consumed + absolute_remaining)
```

これにより予算を毎周期リセットせず、40 m / 10 s の絶対上限も維持する。

### 2. Runtime rollout の座標・reserve整合

entry-time の dynamic pass distance は completion reserve として設定値 1 m を使う。
runtime も同じ値を使う。従来は SafeSeparation の反復可能総距離 24 m を reserve と
して加算していたため、必要距離を過大評価していた。

soft limit は既定 SafeSeparation budget、hard limit は現在時点の絶対残距離とする。

### 3. Infeasible rollout を左右再評価トリガにする

現在側の rollout が有効だが絶対残予算内に rear-clear 不可能な場合は、既存の
last-feasible alternate Mission を先に再検証する。利用できなければ現在の安全な
prefixを維持し、MPCC-lite side evaluation を次周期に即時再実行する。

side-by-side/no-return 後の全幅横断は許可せず、hard fault も従来の Recovery を維持
する。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure budget refresh resolver
- `mpc_controller_cpp.cpp`: runtime rollout接続、既存再計画の起動
- `test_v2x_overtake_core.cpp`: resolverの正常・無効・絶対上限テスト

設定値、ROSインターフェース、評価基盤は変更しない。
