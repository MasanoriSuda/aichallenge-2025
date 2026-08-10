# Requirements

## 背景

`output/20260810-000242/d1/autoware.log` では、通常 Overtake の Pass 中に次の二つの
抜き切り失敗が確認された。

1. target が約 0.9 m 後方でも通常 rear-clear 2.0 m は未成立なのに、wall margin を
   契機として Return を開始し、直後に同じ車両へ再接近して SafetyBrake へ入った。
2. `SideBySideCommitted` 後、短い body-overlap 確認窓の途中で fresh prediction が
   得られず、同側完遂へ移らず Pass Mission を破棄して Recovery へ入った。

## 要求

- 物理壁接触は従来どおり即 Recovery とする。
- lateral separation 済みかつ target が hard-curve clearance 以上後方の場合でも、
  通常 rear-clear が未確認なら中央へ Return しない。
- 上記の margin-only 状態では同じ Pass side と横目標を保持し、通常 rear-clear を作る。
- 通常 rear-clear と return corridor 成立後だけ Return する。
- `SideBySideCommitted` 以降は SafeSeparation から RecoverBehind しない。
- current body overlap が未確認の短い debounce 中は、target continuity、壁、緊急制動、
  solver guard が正常なら同側 SafeSeparation への移行を許可する。
- confirmed overlap、物理壁接触、EmergencyBrake、solver recovery、絶対 Mission budget は
  緩和しない。
- ROS 2 topic / service / message 契約と既存パラメータ値は変更しない。

## 制約

- 既存の未コミット LowSpeedDirect side-completion 修正を保持する。
- `aichallenge/result-summary.json` の既存変更には触れない。
- `output/` と rosbag は変更しない。

