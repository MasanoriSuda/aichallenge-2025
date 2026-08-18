# Results

## 実装結果

- 拡張QPのthetaを `absolute_s - current_s` へ変更した。
- 解変換時にcurrent_sを加算し、既存の物理再検証は絶対progressのまま維持した。
- shifted warm-startのthetaへ `previous_origin - current_origin` を加算した。
- 拡張QP専用のpath/lag/progress重みを導入し、旧3状態MPCCの重みは変更していない。
- 拡張solve失敗後0.75秒は拡張QPを試さず、旧3状態MPCCを直接使用する。
- `Extended MPCC runtime` を1 Hz集約ログとして追加した。

## 検証

### Build

```text
docker compose run -T --rm --no-deps autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

`multi_purpose_mpc_ros`のstderrは既存のsetuptools deprecation warningのみ。

### Test

```text
colcon test --packages-select multi_purpose_mpc_ros
100% tests passed, 0 tests failed out of 28
Summary: 1314 tests, 0 errors, 0 failures, 0 skipped
```

追加テストでは、絶対progress復元、warm-start原点再基準化、0.75秒cooldownを確認した。
`colcon test-result --verbose` はテスト外の古い
`build/joycon_contract_guard/package.xml`欠損をERROR表示したが、対象packageのテスト結果は
0 errors / 0 failuresでexit code 0だった。

## 次回試走で確認するログ

- `Extended MPCC runtime` の attempts / success / solve_failure / circuit_skip
- 全OSQP failure率（前回3.16 %から低下するか）
- Control callback overrun率（前回1.02 %から低下するか）
- `maximum iterations reached`と同一周期legacy fallbackの反復
- `Pass -> Return`比率およびhard-wall authority release回数

## 未検証

AWSIMでの動的効果確認はユーザー試走待ち。今回の変更は壁制約を緩和していないため、
収束改善後もhard-wall releaseが残る場合は、QP内壁包絡と物理再検証の差を次に扱う。
