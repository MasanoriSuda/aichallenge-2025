# Design

## 観測事実

最新走行では `invalid local path target` は 0 件になった一方、static wall
execution preflight の collision が 77 件あり、少なくとも 28 件は最初の姿勢で
棄却された。既存実装は実寸 footprint に `v2x_wall_clearance_margin` を加えた
footprint だけで、現在姿勢を含む経路全体を二値判定している。

そのため、実車体は壁に触れていなくても余裕領域だけが壁へ重なると、そこから
中心へ戻る candidate まで最初の pose で失敗する。

## 方針

静的壁判定を次の二層に分ける。

1. Physical layer
   - 実寸 footprint の swept path を全区間検証する。
   - occupied / unknown / out-of-map は hard reject とする。
2. Clearance layer
   - 通常は従来どおり余裕込み footprint の全区間 clear を要求する。
   - 現在姿勢だけが余裕不足の場合、candidate の初期区間が中心方向であり、
     短距離内に完全 clear となる場合だけ margin escape として許可する。
   - 壁沿い走行では raster 化された同一壁の接触セル数が増減するため、セル数は
     診断値に限定し、実寸 swept path と中心方向性を安全契約にする。
   - 一度 clear になった後の再接触は reject する。

判定本体は `recovery_footprint` の純粋関数へ置き、単体テストで境界を固定する。
controller は Frenet candidate が中心方向かを確認してから限定 fallback を使う。

## ログ

planning decision trace の preflight に以下を出す。

- `preflight_mode=clear|margin-escape|rejected`
- `preflight_raw_poses`
- `preflight_margin_contacts=initial/max/final`
- `preflight_margin_clear=<index>@<distance>m`
- 詳細な reject reason

## rear-clear

DynamicMissionWait resolver は既に rear-clear を Mission invalidation より先に評価する。
今回のログでは wait の時間上限が rear-clear 観測より先に Mission を Idle へ解放して
いるため、この変更では推測による閾値緩和を行わない。まず margin escape によって
Pass 継続性を改善し、決定ログで rear-clear の未成立理由を再計測する。
