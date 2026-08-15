# Design

## Progress-bounded retention

DynamicMissionWait開始時にlocked target longitudinalをcheckpointとして保存する。
FollowPrepare中に0.10 m以上target longitudinalが減少した場合だけcheckpointと最終進捗時刻を
更新する。最終進捗からDynamicMissionWait timeout以内だけ `target_progress_recent` とする。

## Retention authority

短いleaseを越えるrear-clear retentionは次を全て満たす場合だけ許可する。

- DynamicMissionWaitがactive
- Pass-originまたは既にcommit済み
- wall検証済みforward prefixがactive
- full closing authorityまたはruntime lease内のcontinuous DP authority
- target progressがrecent

予測重複によりclosing authorityを失った場合や、接触ペナルティなどで相手が離れていく場合は
retentionを解除する。

## Fresh-search handoff

retention解除後は既存の `PausedMissionTerminalAction::Expire` を使用する。失敗側を1秒blockし、
OvertakeLineをresetして通常entry plannerへ返す。反対側candidateを含むfresh searchを行い、
Recovery速度は使用しない。

## Local refactor

`DynamicMissionWaitRetentionRequest` を明示field assignmentで構築し、prefix authorityと
progress freshnessを型の入力として分離する。
