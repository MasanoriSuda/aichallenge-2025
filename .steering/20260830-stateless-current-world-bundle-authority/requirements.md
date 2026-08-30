# Requirements

## Objective

`output/20260830-152829`で確定したpersistent artifact lifecycle defectを修正する。
historical artifact progressの差だけでnormal authorityを失わず、現在のpose、speed、
serialized steering、wall、全peer、terminal successorから再構築した完全な
current-world ManeuverBundleをproductionの正本にする。

## Root cause

Return中のdecision 3157でhistorical artifactとの差が1.541660 mとなり、既存1.5 m gateが
current-world proofより先にrejectした。同一requestをgate後まで評価すると全証明は
`accepted/proof=1`だった。物理的不可能ではなく、artifact clockをauthority prerequisiteに
したことがEmergency Stop、壁接触、Reverseへ波及した。

## Constraints

- Mission resume、lease、grace、timeout、fallbackを追加しない。
- progress、solver tolerance、wall/vehicle clearanceを変更しない。
- wall、dynamic obstacle、terminal Stop suffixの証明を省略しない。
- target/homotopy、commit/no-return identityを維持する。
- A失敗時だけBを試すfallback構造を残さない。
- production昇格と同時に旧`ProgressLiftRejected` authority branchを削除する。
- 既存の単一normal publisherとEmergency supervisor境界を維持する。

## Acceptance

- historical progress差はdiagnosticとして残るが、それ自体はauthorityを所有しない。
- full current-world proof成功時だけproduction authorityを得る。
- progress rebaseしたproofはstateless Bundleとしてpublication ledgerへ記録される。
- old artifactを未変更のexecuted planとして誤って昇格しない。
- source-contract testが旧progress-reject branchの不在を固定する。
- dev2で同型のReturn中progress差がEmergency Stopへ波及しない。
