# 結果

## 実施内容

- `OvertakeLine` の active transition 優先順位を純粋関数へ切り出した。
- `FollowPrepare` 再開を含む execution side の選択元を純粋関数へ切り出した。
- controller は純粋関数が返した action に対応する既存副作用だけを実行する構造にした。
- 現行の遷移順と side 優先順位を固定する単体テストを追加した。

## 挙動不変

- config、速度 reference / hard limit / floor の合成順序は変更していない。
- wall、SafetyBrake、Recovery、side replan、rear-clear、watchdog の条件とログ理由は変更していない。
- `FollowPrepare` 再開時に Behavior side を優先する現行方針も変更していない。
- topic、service、Domain、launch、評価成果物の契約は変更していない。

## 自動検証

- `make autoware-build`
  - 25 packages finished
  - build successful
  - 既存の setuptools deprecation warning のみ
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 25/25 test suites passed
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`
  - 685 tests
  - 0 errors、0 failures、0 skipped
- `git diff --check`
  - 成功

## 次段階の候補

今回明示した所有権を基準に、実走ログで多発している次の挙動変更を一件ずつ行う。

1. `FollowPrepare` 再開時に Behavior side が mission side を反転させる条件
2. Pass 中の壁・横加速度失敗を減らす経路成立性
3. front cap の解除後再適用と Pass 速度維持

これらは性能変更になるため、今回の挙動不変リファクタリングには含めない。
