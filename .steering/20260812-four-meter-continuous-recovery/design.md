# Design

## 1. 4 m continuous Reverse

`reverse_escape_distance_m` と `fast_rejoin_min_reverse_distance_m` を4.0 mへ揃え、
adaptive retryは4.0 -> 8.0 mとする。clear-footprintでは既存のfull swept rolloutを使う。

初期接触中は、4 m全体を `RequireImprovement` で評価する。全swept sampleで接触数が
増えず、新規contact patchやout-of-mapがなく、終端で接触が減るcandidateだけを
continuous candidateとする。不成立時だけ既存0.4 m stepwiseへfallbackする。

## 2. RecoveryIncidentLedger

Supervisorのper-maneuver budgetとは別に、controller lifetimeで物理incidentを管理する。

- incident start time
- total/reverse/forward distance
- aggressive retry count
- gear request count
- normal forward progress after rejoin

SafeStop retryやcandidate reassessmentではresetしない。Rejoin後も保持し、正常前進5 mで
終了する。同一incidentのaggressive retryはadaptive reverse levelも進め、4 m失敗後は
8 mを要求する。

## 3. Runtime motion guard

固定0.05 m corner-motion上限を廃止する。実dtに対し、設定最大速度と最大操舵から求めた
center/yaw motion上限へodometry toleranceを加え、明らかなteleportだけをrejectする。
static rolloutは従来どおり0.05 m以下のswept sampleへ分割するため、計画側の解像度は
緩和しない。

## 4. Gear resend

`max_command_requests=3`, interval 0.2 sとする。同一gear state内の再送は新しいmaneuver
attemptとして扱わず、incident ledgerでは観測可能にする。

## 影響範囲

- `stuck_recovery_core.hpp/.cpp`: incident ledger、motion guard、adaptive retry API
- `mpc_controller_cpp.cpp`: controller統合、continuous contact preflight、ログ
- `config.yaml`, `config_for_cloud.yaml`: 4 m初期値とgear再送
- `test_stuck_recovery_core.cpp`: ledger/motion guard/retryテスト

