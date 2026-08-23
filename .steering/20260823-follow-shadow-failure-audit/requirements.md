# Follow shadow failure audit requirements

## Purpose

Canonical MPCC Follow authorityの昇格前に、`output/20260823-134218`で観測した
solve failureとexecution-primal rejectionの根本原因を特定する。

## Scope

- Follow shadowの同一周期入力とhorizon境界を記録する。
- execution primal rejectionをfield、stage、value、violation、toleranceへ分解する。
- 設定を変えずに`make dev3`を再実行し、失敗の最上流原因を分類する。
- Hold/Stopのownership監査結果をauthority移行判断へ反映する。

## Constraints

- Followをproduction authorityへ昇格しない。
- solver設定、wall margin、Follow距離、速度制約を調整しない。
- 新しいretry、fallback、timeout、flag、clampを追加しない。
- SafetyBrakeとRecoveryのemergency ownershipを変更しない。
- ユーザー所有の`aichallenge/result-summary.json`を変更・stageしない。

## Definition of Done

- 各Follow shadow失敗が最初に破った境界またはsolver failureとして識別できる。
- 同じログ行からego速度、target gap/速度、先頭速度制約、terminal進捗区間を照合できる。
- 動的証拠に基づく原因・反証・次の最小修正を`tasklist.md`へ記録する。
- authority昇格やパラメータ調整は原因確定後の別Sliceに残す。
