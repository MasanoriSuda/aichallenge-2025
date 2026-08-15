# Design

## Terminal wait policy

`DynamicMissionWaitRequest` に次を追加する。

- `terminal_budget_abort`
- `current_replacement_tactical_rearmed`

terminal waitでは、alternate complete Missionを最優先し、同側はstrict re-arm済み
candidateだけを1 episodeにつき1回だけ採用する。2回目のterminal budget到達または
評価が完了しても候補がなければ
`ReleaseForFreshSearch` を返す。通常waitのhold/resume contractは維持する。

## Strict same-side re-arm

FollowPrepare中の同側candidateを `MpccLitePrefixExecutionRequest` で再評価する。
要求するのは以下。

- Pass-origin terminal wait
- locked target continuity
- current/predicted footprint separation
- wall/corridor/hard-fault clear
- progressive and physically feasible candidate
- body-clear/target-clearance/minimum-speed checks
- DynamicMissionWaitの短いtime/distance lease内

## Fresh search handoff

strict同側prefixもalternate Missionもない場合は、失敗側を
`PhysicalOrCommittedFailure` として短時間blockし、OvertakeLineをresetする。
次周期の通常entry plannerが反対側を含めて再探索する。Recovery速度へは落とさない。

## Local refactor

controllerの `DynamicMissionWaitRequest` aggregate初期化を明示field assignmentへ変更し、
terminal policyの入力生成を一箇所にまとめる。
