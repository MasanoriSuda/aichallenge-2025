# Validation

## Failure first

削除契約`test_legacy_boost_normal_authority_is_physically_deleted`を実装前に直接実行し、
legacy token検出による`AssertionError`を確認した。host全体のpytest収集は同directoryの
`localization_scope` import環境不足で止まったため、最終確認は正規Docker環境で行った。

## Static validation

- production source、launch、CMake、message定義にlegacy tokenなし。
- Python controller／provider／launchの`py_compile`成功。
- `git diff --check`成功。
- C++ final publisherはnormal/failsafe/shutdownを同じ`command_pub_`へ収束。
- `/awsim/cmd` official Boost publisherとgear Recovery publisherは残存。
- `SolverDerivedBypass`はproduction producerなしの拒否試験用表現として維持。

## Build

```text
make autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

## Tests

```text
colcon test --packages-select multi_purpose_mpc_ros
100% tests passed, 0 tests failed out of 49

colcon test-result --verbose
Summary: 1844 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result`の`joycon_contract_guard/package.xml`警告は既存build artifactの欠損参照で、
今回対象packageの49 targetはすべて成功している。

## Dynamic evidence

checked-in提出launchではlegacy flagが常にfalseだったため、本Sliceは到達可能な提出挙動を変更しない
物理削除である。新しい試走は必須gateとはしない。次回通常試走では、canonical normal publication、
Emergency、Recoveryおよび`/awsim/cmd` StartOnceだけが観測されることを確認する。

## Protected user state

`aichallenge/result-summary.json`は既存のユーザー変更であり、編集・stage・commit対象から除外した。
