# Design

## Root cause

`CanonicalExecutionIdentityRequest::overtake_line_active` が二つの責務を兼ねていた。

- 戦術上のShiftOut/Pass/Return encounter identityが生存しているか
- その周期のlegacy OvertakeLine stage corridorが利用可能か

後者がfalseになると前者まで消え、seven-state MPCCはcurrent target tubeとtrack wall envelopeから新しい問題を作る機会を失っていた。これはwall不成立そのものではなく、再計画入口のauthorityをlegacy geometry availabilityで閉じる接続不良である。

## Change

1. request fieldを `overtake_execution_requested` へ改名し、identityの責務を明示する。
2. callerはShiftOut/Pass/Returnというlive tactical phaseからexecution identityを要求する。
3. stage corridor availabilityは従来どおり、stage-wise boundsとauthority traceで独立管理する。
4. corridor inactive時はbase physical wall boundsとcurrent target tubeを使ってcurrent-world seven-state問題を構築する。
5. certified siblingの採用は既存のsame-epoch、before-no-return、publisher-token契約を変更せず利用する。

## Non-goals

- unsafeな現側branchの継続
- sibling branchの無条件採用
- Pass phaseの延命
- wall marginやsolver設定の緩和
