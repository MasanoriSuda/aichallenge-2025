# Design

## Comparison boundary

同一のimmutable source plan、current world、control origin、published command履歴を使い、
retained revalidationだけを二通り評価する。

### A: persistent artifact pipeline

- published artifact clockでcursorを進める。
- physical progress差が既存toleranceを超えたらrejectする。
- 現行production behaviorを一切変えない。

### B: stateless current-world rebase shadow

- Aと同じcursor、control inputs、target/homotopy identityを使う。
- progress差はdiagnosticとして保持するが、その一点では打ち切らない。
- current control pose/speed/serialized steeringからcontinuationを再構築する。
- measured-to-control path、exact wall、dynamic obstacle、terminal successorを通常どおり証明する。
- 最終Reasonとproof availabilityだけをAのResultへshadow診断として添付する。

## Implementation

1. 既存`evaluate()`本体をprivate `evaluate_impl(request, enforce_progress_gate)`へ分離する。
2. public `evaluate()`はまずAを評価する。
3. Aが`ProgressLiftRejected`の場合だけBを評価する。
4. Aのreason/proofは変更せず、Bのreason/proof availabilityをdiagnosticへ保存する。
5. controller telemetryへB結果を集約する。

## Interpretation

- A fails, B succeeds: persistent artifact lifecycle/model join defect。
- A/B fail, same later reason: physical/certificate failure。
- B wall/dynamic/terminal failure: candidate physical infeasibility。
- B succeeds but live fresh candidateが間に合わない: scheduling/lifecycle defect。

## Non-goals

- Bをproductionへ昇格する。
- progress gateを削除または緩和する。
- sibling adoption条件を増減する。
- MPCC cost/constraint parameterを調整する。
