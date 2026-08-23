# Side-split Follow QP audit

## Observation

前Sliceで、two-sided rowに一つのscaleを適用してもlower/upperそれぞれのphysical toleranceを
同時表現できないことを確定した。

## Tested hypothesis

physical rowをsolver-spaceだけでlower/upperのone-sided rowへ分割し、各sideを独立scaleすれば、
OSQP terminationとphysical certificateが一致する。

## Implemented observation candidate

- physical `A/l/u`とpost-solve certificateは不変。
- finite non-equality two-sided rowのみsolver-spaceで二分。
- solver dualをphysical rowへ加算合成。
- physical warm-start dualを符号でlower/upperへ分解。
- production/Follow authority接続なし。

## Static evidence

- `make autoware-build`: 25 packages成功。
- 最初のfocused run: 12/13成功。単一変数fixtureでは旧policyが偶然十分正確に解けたため、
  failure-first条件を実ログと同じmixed-unitへ修正。
- 修正後focused run: 12/13成功。
- dual合成、lower/upper符号、warm start、workspace updateは成功。
- failure-first caseでは新side-split policyも失敗:
  - OSQP status: solved、50 iterations。
  - physical upper violation: 0.0895179。
  - physical normalized violation: 66.8284。

## Root-cause conclusion

solver row分割は可行領域とKKT dual mappingを保てるが、OSQPの内部scalingとglobal stopping
criterionを各physical sideのtoleranceへ変えるものではない。大きく異なるscaleを持つ同一rowの
重複は、むしろconditionを悪化させ得る。

過去に棄却したrow normalization、scaled termination、full nondimensionalizationと同じく、
algebraic coordinate変換だけでは固定solver-space absolute toleranceとphysical certificateの
一致を保証できない。

## Decision

static gateで棄却。dynamic試走、Follow shadow接続、authority昇格、parameter変更は行っていない。
候補コードとtestは全て削除した。

既存の`RowToleranceNormalized` Follow shadowは保持する。run `20260823-144839`の98.7% coverageと
run `20260823-150821`のtyped rejection provenanceを、Slice 4 authority設計の既知のsolver
unavailabilityとして扱う。次は数値変換を繰り返さず、fresh failureをsame-formulation retained
candidateで連続化できるかを監査する。
