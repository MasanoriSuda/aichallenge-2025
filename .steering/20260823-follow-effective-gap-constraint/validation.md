# Validation

## Root cause confirmed

従来のFollow boundはvirtual progressだけを上限化しており、world poseへ加算されるFrenet lagを
車間証明へ含めていなかった。failure-first fixtureでは、virtual progress gapが2.1mで旧判定を
通る一方、lagが+0.3mのためphysical gapは1.8mとなり、2.05m hard gapを0.25m割る。

## Implemented correction

- typed Follow contractへ`hard_gap_m`を保持。
- Follow shadowだけ各stateに`lag + progress <= target_progress - hard_gap`を追加。
- 同じ式をpure post-solve certificateとして再検査。
- constraint rowを`follow-effective-gap`としてstage付きで識別。
- lateral certificateとexecution-primal normalizerが、標準layoutまたは完全なFollow gap row block
  だけを受理するよう更新。
- Track/CruiseとOvertakeのQP row blockは追加していない。

## Commands and results

```text
test_race_mpcc_foundation: 22 passed
test_mpcc_progress: 63 passed
make autoware-build: 25 packages finished, Build successful
colcon test --packages-select multi_purpose_mpc_ros:
  1591 tests, 0 errors, 0 failures, 0 skipped
```

## Authority audit

- Follow shadowのcommand生成とpublisher接続は未変更。
- Track/Cruise production authorityは未変更。
- config、hard gap値、solver tolerance、iteration、scalingは未変更。
- Follow telemetryのacceptedはeffective-gap certificate成立周期だけを数える。
- `authority=shadow, selected=0`を維持。
- ユーザー所有の`aichallenge/result-summary.json`はcommit対象外。

## Next dynamic evidence

次回試走では次を確認する。

- Follow eligible周期のsolve率とaccepted率。
- `follow-effective-gap` row reject回数とstage。
- minimum/terminal effective gapが2.05mを下回らないこと。
- Followの実制御挙動が変化しないこと（shadow-only）。
