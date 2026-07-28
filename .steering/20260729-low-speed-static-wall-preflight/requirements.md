# LowSpeed静的壁preflight整合化 要件

## 背景

`output/20260729-060157/d1/autoware.log` では、停止車に対して
`LowSpeedAvoidance` とdirect Shiftを開始できている一方、約1〜2秒後に
`wall clearance margin violated` で停止している。

現行の開始判定はtrajectory waypointの横境界とV2X車両円を使うが、実行中の
wall guardはraw pose、静的occupancy grid、実車矩形footprint、
`v2x_wall_clearance_margin`を使う。このため、開始時に「通れる」としたsideを
実行時判定が直後に否定できる。

## 必須要件

- LowSpeed direct controlを開始する前に、実行時wall guardと同じ静的grid、
  実車footprint、横wall marginで選択sideへの掃引を検証する。
- direct controllerが即時に追う`pass_target_ey`までの横移動と、その先の
  planning horizonを検証対象にする。
- `auto` side選択で最初のsideが静的壁preflightを通らない場合、反対sideを
  同じ条件で評価し、通れる場合は反対sideを採用する。
- 両sideが不成立、静的geometryが利用不能、map外、unknown/occupied接触の
  場合はdirect controlを開始しない。
- 実行中のraw-pose wall guardは最後の安全防護として維持する。
- ROS 2 topic/service/message、launch entry、Domain、評価JSON契約を変更しない。
- 既存parameter名と既定値を変更しない。

## 非対象

- 通常OvertakeLineのwall guard調整
- wall clearance値の緩和
- MPC solver failure対策
- AWSIM/評価基盤側の変更

## Definition of Done

- endpointだけがclearでも途中に壁があるpathを掃引検査が棄却する単体テストがある。
- clear path、map外、invalid inputをfail-closedで判定できる。
- LowSpeed候補の静的preflight不成立時に、`auto`なら反対sideを試す。
- 対象packageのbuild/testが成功する。
- `git diff --check`が成功する。
- 実走確認項目をtasklistへ残す。
