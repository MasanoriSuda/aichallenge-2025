# Design

## Bound hierarchy

各live-horizon sampleを次の二層に分離する。

### Preferred layer

- robust wall planning clearance
- robust/configured target center separation
- Mission trust region

optimizerは可能な限りこの区間を使う。成立しない場合はtarget離隔、Mission trust、壁追加余裕の順に縮退できる。

### Hard execution layer

- `v2x_overtake_line_min_wall_clearance`
- `v2x_vehicle_radius`による物理target中心間隔
- static-map上の物理footprint

post-validation、projection repair、last-feasible leaseはこの層を越えてはならない。逆にpreferred層を外れただけではRecoveryにしない。

## Initial bound construction

従来のpreferred wall interval内でtarget境界を解く。物理target離隔まで縮退しても解がない場合、elastic modeではhard wall intervalへ切り替えて再度target境界を解く。hard intervalでも物理離隔が成立しない場合だけhard infeasibleとする。

## Post-validation

optimizerに採用したtarget離隔はpreferred値として保持する一方、execution constraintへ保存する値は物理中心間隔とする。static-map／横加速度補正後の経路はhard wall＋物理target境界へ射影・再検証できる。

このため、1.75 mを目標に生成した経路が補正後1.60 mとなっても、1.45 m以上かつhard壁余裕を満たせば実行可能になる。

## Last feasible

同一Mission generation、side、phaseのlease内に限り、last-feasible trajectoryをpreferred optimizer boundsではなく現在のhard execution boundsで再検証する。現在のhard faultがある場合は保持しない。

## Configuration

`v2x_overtake_receding_horizon_elastic_clearance_enabled`を追加する。

- `true`: preferred/hard二層境界を使用
- `false`: 従来の初期採用target境界を実行hard境界として使用

提出・ローカル設定はいずれも`true`とする。

