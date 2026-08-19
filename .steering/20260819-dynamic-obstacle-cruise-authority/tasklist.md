# Tasklist

- [x] 現行の dynamic-obstacle target、GapPlanner、Follow cap の境界を確認する。
- [x] Dynamic lateral escape authority resolver を追加する。
- [x] Follow 中の実行可能な dynamic corridor を速度仲裁へ接続する。
- [x] Follow preposition より dynamic target を優先する。
- [x] config / startup log / runtime log を追加する。
- [x] resolver の単体テストを追加する。
- [x] package build と focused test を実行する。
- [x] 意図したファイルだけをコミットする。

## Definition of Done

- Mission admission が不成立でも、実行可能な動的横回避経路が MPC へ入る。
- 横回避中は generic Follow cap が再適用されない。
- EmergencyBrake、solver fault、planner infeasible は従来どおり fail closed となる。
- OvertakeLine active 時の Mission authority を変更しない。
- ユーザー変更 `aichallenge/result-summary.json` をコミットしない。

## Verification result

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R
  test_v2x_overtake_core --output-on-failure`: 成功
- `colcon test-result --verbose`: 1343 tests、0 errors、0 failures
- 追加した dynamic lateral escape authority 4 tests の実行を XML で確認
- `git diff --check`: 成功
