# Requirements

## Objective

`make dev`、`make dev2`、`make dev3`の全構成で、canonical MPCCのdynamic-world
証明を維持したままAWSIMの走行を開始できるようにする。

## Observed failure

- baseline: `bad3e24`
- run: `output/20260826-045217/d1`
- `make dev`ではAWSIMが`Ready`まで遷移するが`Start`へ進まない。
- fresh canonical MPCC solveは成功している。
- `/v2x/vehicle_positions`が未受信のため、retained current-world proofが
  `dynamic-observation-unavailable`で全候補を拒否する。
- production authorityが得られずEmergency Stopをpublishし続ける。

## Repaired invariant

単車シナリオでは、シナリオ構成を所有する起動層が「他車両0台」のcurrentな
`V2XVehiclePositionArray`を明示的に供給する。通信未受信をempty worldと推測してはならない。

## Scope

- `make dev`から車両数1をAutoware launchへ伝播する。
- `make dev2`、`make dev3`から実際の車両数を各Autoware launchへ伝播する。
- simulationかつ車両数1のときだけ、車両Domain内にexplicit-empty V2X producerを起動する。
- `make eval`の現行単車構成にも同じ契約を適用する。
- 起動契約を静的テストで固定する。

## Non-scope

- MPCCのNoData fail-close条件を緩和しない。
- `expected_v2x_vehicle_count`を参加者controllerへ再導入しない。
- Domain bridgeを追加しない。
- `/admin/awsim/start`のownerを変更しない。
- 制御パラメータ、solver設定、wall marginは変更しない。

## Definition of Done

- NoDataと明示的empty worldが引き続き区別される。
- `make dev`でempty V2X観測がcurrentとなり、Track/Cruise authorityが選択されて発進する。
- `make dev2`、`make dev3`ではexplicit-empty producerが起動せず、AWSIMのnative V2Xを使用する。
- package testと`make autoware-build`が成功する。
- 動的確認ログに`state=start`があり、単車で`dynamic-observation-unavailable`が継続しない。
