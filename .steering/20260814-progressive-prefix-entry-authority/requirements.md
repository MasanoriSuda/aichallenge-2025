# Requirements

## 背景

`output/20260814-075236`ではMPCC-liteが左右のhard-feasibleなprogressive prefixを複数回
生成していたが、入口でのprefix admissionは`active_execution=false`により`inactive`となった。
入口authorityも完全なrear-clear/Return Missionだけを許可するため、横回避を開始できないまま
pre-arm加速だけが継続し、最後は前車へ接近してSafetyBrakeへ入った。

## 目的

- Follow／Idle中でも、ShiftOutからbody-clearまでを検証済みの短期prefixを実行可能にする。
- progressive prefixの実行権を得た周期だけ、追い越しMissionへ縦横同時にhandoffする。
- MPCC-lite評価間隔中は、同一target・同一sideのfreshなcurrent prefixだけを短時間再検証して使う。
- rear-clear／Returnは既存のrolling replanで継続検証し、完全Missionであるとは偽装しない。

## 制約

- current body、target sweep、wall reserve、body-clear deadline、速度・時間・距離budgetは緩和しない。
- EmergencyBrake、V2X異常、position jump、solver recoveryではentry authorityを与えない。
- staleなprefix自体を再実行しない。leaseはauthorityのside/targetだけを保持し、経路はcurrent cycleで
  再生成・再検証されたprefixを使う。
- topic/service/Domain/提出インターフェースは変更しない。

## 完了条件

- new-entry contextでhard-feasible progressive prefixが`SelectEntry`を得る。
- active Missionのno-return、SafeSeparation、wall/target/body-clear拒否は従来どおり維持される。
- progressive entryが`has_executable_mission`、縦方向ownership、Behavior handoffへ一貫して伝播する。
- 対象packageのbuild/testが成功する。
