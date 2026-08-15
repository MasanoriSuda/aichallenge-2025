# Design

## Correction of the observed horizon

`plan_N=52` は52 mではなく52サンプルである。参照経路解像度0.6 mのため、
実距離はおよそ31.2 mとなる。

## Horizon split

Frenet DPの各sampleを距離で2領域に分ける。

- `path_distance <= 20.0 m`: hard wall clearance = 0.20 m、
  preferred wall clearance = robust clearance
- `path_distance > 20.0 m`: hard wall clearance = 0.0 m、
  preferred wall clearance = 0.20 m

遠方でもraw track boundsとtarget-side corridorはhard boundsとして残すため、
壁外や相手車体を横切るbranchは作らない。追加の0.20 m追従余裕だけをsoft化する。
経路が20 m以内へ近づくとrolling refreshで同じsampleがhard領域へ入り、実行前に
再検証される。

## Configuration

`v2x_overtake_mpcc_frenet_dp_hard_horizon_distance: 20.0`

既定値と本設定は20 mとする。

## Verification

- 距離境界で適用clearanceがhardからsoftへ切り替わるunit test
- 既存Frenet DP / overtake core test
- `make autoware-build`
