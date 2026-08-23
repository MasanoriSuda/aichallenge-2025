# Validation

## Result

Followをcanonical normal intentとして扱うpure contractは成立した。controller、plan store、
publisher、ROS parameterには変更を加えていないため、このSlice単独ではproduction authorityも
車両挙動も変化しない。

## Contract evidence

- `canonical_normal_intent_supported`がTrack/Cruise/Followのallow-list正本になった。
- plan validation、candidate qualification、authority request validationは同じ正本を参照する。
- Followはtarget ID、observation generation、target obstacle generationが揃わない限り
  `IncompleteProblem`としてfail closedになる。
- Passなど未対応intentは引き続き拒否される。
- fresh certified Follow candidateからcanonical commandまでintentとtarget provenanceが保持される。

## Commands

```text
focused build/test:
  test_mpcc_execution_contract: 50 passed
  test_canonical_execution_plan: 12 passed

make autoware-build:
  25 packages finished
  Build successful

colcon test --packages-select multi_purpose_mpc_ros:
  1587 tests, 0 errors, 0 failures, 0 skipped
```

## Diff audit

- normal controllerのauthority選択箇所は未変更。
- Follow shadow solverの出力変換は未変更。
- canonical plan storeとpublisher接続は未変更。
- config、solver設定、wall margin、fallbackは未変更。
- ユーザー所有の`aichallenge/result-summary.json`は検証・commit対象外。

## Next gate

次SliceではFollow shadow solve結果を、同一decisionのactuation、physical wall certificate、
canonical plan/candidate/commandへ変換する。ただし最初は`authority=shadow, selected=0`を維持し、
Track/Cruiseのproduction authorityを置換しない。
