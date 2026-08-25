# Design

## Causal chain

```text
make dev (vehicles=1)
  -> AWSIM native V2X publisher emits no message
  -> V2X snapshot has no observation identity
  -> retained current-world proof rejects as dynamic-observation-unavailable
  -> canonical Track/Cruise production authority unavailable
  -> Emergency Stop publishes speed=0
  -> count-mode AWSIM never observes launch motion
  -> Ready -> Start does not occur
```

最初に破られた不変条件は「正常走行authorityにはcurrent dynamic-world observationが必要」
ではなく、その証拠を単車シナリオのproducerが供給していないことである。controllerのfail-closeと
Emergency Stopは正しい下流動作であり、修正対象ではない。

## Selected fix

シナリオ構成を知るMake/launch層からvehicle countを伝播し、simulationかつvehicle countが1の
場合だけ、車両Domain内でheader timestamp付きの空`V2XVehiclePositionArray`を周期publishする。

```text
Make target vehicle count
  -> Compose environment
  -> run_autoware.bash launch argument
  -> aichallenge_system.launch.xml
  -> single_vehicle_empty_v2x_publisher (only count == 1)
```

2台以上ではnodeを起動しないため、AWSIM native V2Xとのproducer競合を作らない。

## Rejected alternatives

### Treat NoData as empty in MPCC

多車両時のV2X断を空コースとして扱うため不採用。既存fail-close契約を破る。

### Add controller timeout/grace/expected count

参加者controllerにシナリオ構成責務を混ぜ、過去のauthority分岐を再導入するため不採用。

### Publish `/admin/awsim/start` from Autoware

Domain 0管理契約とcount-modeの責務を破り、原因であるdynamic-world evidence不足も残るため不採用。

## Interface impact

- 既存topic名・型は変更しない。
- `/v2x/vehicle_positions`のproducer契約を、単車シミュレーションについて明示化する。
- Domain 0/1..Nの分離を維持する。
- 実車ではexplicit-empty producerを起動しない。
- 評価・提出JSONや提出tar.gz契約に変更なし。

## Production paths

- participant controllerのnormal authority追加: 0
- participant controllerのfallback追加: 0
- system harness process追加: 単車simulation時のみ1
- remaining legacy normal authority: 変更なし

## Rollback

Baseline commit: `bad3e24`
