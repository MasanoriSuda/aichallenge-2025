# Follow fresh canonical shadow

## Purpose

Follow five-state MPCC shadowの合格条件を「QPの縦制約を満たした」から、同一decisionで
物理証明・canonical plan・candidate・authority・actuation・commandまで再構成できた状態へ
引き上げる。

## Root cause

現状のFollow shadowはexecution primalと進捗上限を検査した時点でacceptedとしている。
このため、将来production authorityへ昇格するときに必要な壁証明、plan/cursor identity、
actuationの非変形性をまだ観測できない。

## Scope

- Follow shadow解からactuation proposalを抽出する。
- current-pose connectorを含む物理壁証明を行う。
- target hard-gap contractをobstacle certificateの根拠としてcanonical solutionへ保持する。
- canonical plan/cursor/candidate/fresh authority/actuation/commandを同一decisionで検証する。
- 各段階のreject理由と集計をFollow telemetryへ追加する。

## Non-scope

- Follow commandのpublisher接続。
- retained Follow plan。
- Track/Cruise authorityの変更。
- solver、margin、速度、wall parameter調整。
- Hold/Stop/overtake intentの昇格。

## Acceptance

- canonical commandまで成立した周期だけを`canonical-ready-shadow`として数える。
- 物理壁証明、identity、cursor、actuation一致のどれかが欠ければfail closedになる。
- ログは`authority=shadow, selected=0`を維持する。
- focused/full testsと標準buildを通す。
