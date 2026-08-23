# Validation

## Root cause and invariant

Follow shadowは以前、five-state QPと縦方向contractが成立した時点でacceptedとしていた。
しかしproduction authorityに必要な、現在姿勢からの壁証明、canonical plan/cursor identity、
fresh candidate、actuation非変形、canonical commandは再構成していなかった。

本Sliceでは、前提Sliceで追加した`progress + lag`のtarget hard-gap制約とcertificateを用い、
同一decision・同一normalized primalから完全なfresh chainを再構成できた周期だけを
`canonical-ready-shadow`とする。

## Implemented boundary

- Follow normalized primalからtyped actuationを直接抽出する。
- current pose connectorを含むswept wall certificateを実行する。
- wall certificateとFollow effective-gap certificateを一つのphysical certificateへ束ねる。
- canonical plan、exact cursor、fresh candidate、fresh authority、stored actuation、commandを検査する。
- direct primalとstored actuationの差が`1e-12`を超えればfail closedにする。
- fresh chainは`canonical_execution_plan_adapter`へ集約し、controller内へidentity処理を複製しない。
- commandはFollow shadow resultとtelemetryにだけ保持し、`MpcControlCycleResult`やpublisherへ渡さない。

## Static validation

- `git diff --check`: pass
- `make autoware-build`: 25 packages、success
- focused tests:
  - `test_canonical_execution_plan_adapter`: 8 passed
  - 前段で`test_mpcc_execution_contract`、`test_mpcc_progress`、`test_race_mpcc_foundation`: 135 passed
- package regression:
  - 38 test executables passed
  - 1594 tests、0 errors、0 failures、0 skipped

## Authority audit

- Followの実行結果は`canonical-ready-shadow`でも`authority=shadow, selected=0`を維持する。
- Follow commandを`MpcControlCycleResult::canonical_normal_command`へ代入する変更はない。
- Track/Cruise retained storeとproduction selectorへの変更はない。
- パラメータ、solver設定、wall marginの変更はない。

## Dynamic evidence required before promotion

- Follow eligible周期に対する`canonical-ready-shadow`率。
- reject段階別の件数と最初の不成立理由。
- physical wall reject位置とFollow effective-gap reject率。
- solve/certificate/total時間の平均・最大値。
- legacy Follow指令との差。shadow commandは採用しない。
