# Design

## 1. Physical wall envelope cache

`find_clear_lateral_interval_with_heading()` は footprint 全セル走査を伴う。前回変更では
MPCC horizon の各 stage、hard/preferred reserve ごとにこの探索を 40 Hz callback 内で
繰り返したため、同じ waypoint とほぼ同じ姿勢の計算を再実行していた。

MPC 内に上限付き cache を置き、waypoint、探索区間、preferred offset、heading、clearance、
sample step を細かく量子化した key で feasible result のみ再利用する。再利用時は区間を
小さく収縮し、現在の scalar bounds と再交差させる。候補軌道に対する既存の完全な
footprint/wall 再検証は残すため、cache は hard safety の代替ではない。

非同期 tactical snapshot には cache をコピーしない。各 snapshot は独立した空 cache を
持ち、live controller の callback cache と競合しない。静的壁 geometry 設定時に cache を
破棄する。

## 2. Latched Pass target-bound continuation

target-wall conflict は相手予測と選択 side の組合せによる将来制約矛盾であり、現在の実壁
接触そのものではない。Pass で横離隔 latch 済み、locked target 継続、現在の車体非重複、
実壁 hard guard 正常なら、同側の物理 wall 検証済み prefix を保持して次周期の左右再評価を
促す。

保持を許可しない条件は次のとおり。

- actual footprint contact / wall margin blocked / wall sample unavailable
- current body overlap（recoverable side contact を除く）
- target jump / progress reject / continuity loss
- EmergencyBrake / solver recovery / forbidden waypoint

これにより「予測上の将来競合」を停止理由にせず、現在安全な実行軌道を捨てない。
