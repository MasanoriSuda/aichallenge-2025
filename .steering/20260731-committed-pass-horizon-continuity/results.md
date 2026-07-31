# 結果

## 実施内容

- Pass で既に release 済みの front cap hold を、execution horizon feasibility から分離した。
- 初回 release は従来どおり feasible horizon を要求する。
- horizon infeasible 中に committed hold が実際に速度権限を維持している場合、既存の
  `speed_hold` 診断が 1 になるようにした。
- committed Pass 速度 floor は、従来どおり execution path が physically feasible な場合だけ
  適用する。

## 維持した安全条件

次の場合は front cap release をholdしない。

- 初回 release 前
- ShiftOut
- lateral exclusion 未latch
- reapply 用横離隔閾値未満
- locked target の車体横離隔不成立
- locked target position jump
- locked target loss
- 実壁接触

wall / lateral acceleration limit、Recovery、SafetyBrake、MPC hard limit は変更していない。

## 自動検証

- `make autoware-build`
  - 25 packages finished
  - build successful
  - 既存の setuptools deprecation warning のみ
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`
  - 695 tests
  - 0 errors、0 failures、0 skipped
- `git diff --check`
  - 成功

## 実走確認

`make dev2` で低速車への通常 Overtake を確認する。

### 期待ログ

Pass 中に horizon が infeasible になった場合:

```text
cap_release=1, horizon_release=0, speed_hold=1
```

従来多発した次の再適用ログが、横離隔維持中には減ることを確認する。

```text
OvertakeLine execution front cap: Reapplied, reason=execution horizon constrained
```

### 合否

- Pass 中の `execution horizon constrained` による cap 再適用回数が減る。
- Pass 開始後に前車速度へ戻る回数が減る。
- `Pass -> Return` 完遂率が上がる。
- 壁接触、車両接触、Recovery 回数が悪化しない。

