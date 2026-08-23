# Follow canonical intent contract design

## Root cause

Follow QPはtyped contractとproblem fingerprintまで持つが、その下流にある二つの独立した
allow-listがTrack/Cruiseだけを許可する。これはSlice 4未実装の明示的blockerであり、solver
availabilityとは別問題である。

## Correction

`canonical_normal_intent_supported(ControlIntent)`をexecution contractの正本とし、次が共有する。

- `validate_canonical_execution_plan`
- `qualify_canonical_normal_candidate`
- `resolve_canonical_normal_authority` request validation

support対象はTrack、Cruise、Followのみ。Follow固有のtarget完全性は既存
`problem_context_complete`へ明示し、target ID自体とobservation/target generationの両方を
必須にする。これにより空targetのFollowがfingerprint上だけ完全になる経路も閉じる。

## Authority boundary

このSliceはpure contractだけを変更する。controllerはFollow resultをcanonical candidateへ変換して
いないため、production/shadowの選択数とpublisherは変化しない。

## Deletion

planとauthorityに重複していたTrack/Cruise比較を削除し、共通support関数へ置換する。
