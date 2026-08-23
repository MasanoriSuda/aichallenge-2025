# Follow canonical intent contract

## Purpose

Follow five-state MPCCをcanonical execution planへ流せるよう、Track/Cruise専用になっている
plan/authority intent admissionを一つのtyped contractへ統合する。

## Earliest violated invariant

`ControlIntent::Follow`とtarget provenanceを持つ完全なfive-state contextでも、
`CanonicalExecutionPlan`と`CanonicalNormalAuthority`が別々のTrack/Cruise限定ifで拒否する。

## Scope

- canonical normal intentのsupport判定を一か所へ集約する。
- Track、Cruise、Followをsupportする。
- Followは完全なtarget identity/generationを持つcontextだけが既存complete checkを通る。
- pure plan/authority/command testsを追加する。

## Non-scope

- Follow runtime solve後処理、plan store、publisher接続。
- Hold/Stop/overtake intentの追加。
- config、solver、margin、fallback変更。

## Acceptance

- target provenance付きFollow plan/candidate/commandがpure testsを通る。
- target provenance欠落、intent mismatch、Pass等のunsupported intentはfail closedを維持する。
- full package testとbuildを通す。
- runtime authorityやROS出力に差分がない。
