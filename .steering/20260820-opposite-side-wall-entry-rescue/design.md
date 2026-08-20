# Design

## 1. Extended branchの物理実行契約

左右のextended MPCC solve後、5-state解を既存3-state実行形式へ変換し、stage lateral
軌跡を抽出する。現在のraw odometry poseを始点として全stage間を補間し、静的壁mapへ
車体footprintと実行時required clearanceを適用する。

検証NGのbranchはQP上のbound reserveやobjectiveが良くても`feasible=false`とする。
これにより既存のdual branch selectorが、片側NGなら反対側を自然に選べる。

## 2. Atomic entry admission

dual branchが有効な新規entryでは、幾何・DP候補だけをfallback採用しない。
`ExtendedBranchEntryAdmission`を採用境界の正本とし、次を判定する。

- 選択が有効か
- 選択sideと実際に渡すMission sideが一致するか
- dual評価が不要な文脈か

両側不成立時はMission、side、longitudinal ownershipを公開せず、現行レーシングライン
または通常Followを維持する。active Missionでは既存no-return / same-side継続契約を
維持し、この新規entry gateで強制的に反対側へ横断させない。

## 3. ログ

既存の `Overtake MPCC-lite async` 一行へ左右それぞれ次を追加する。

- physical wall validation attempted/passed
- validation scope
- required clearance
- failure reason

新規entryを両側不成立で保留した場合は、target、左右理由、actionを明示する。

## 影響範囲

- `mpcc_progress.*`: atomic entry admission純粋判定とtelemetry
- `mpc_controller_cpp.cpp`: branch解の物理検証、entry gate、ログ
- `test_mpcc_progress.cpp`: admissionと逆側選択の回帰試験
