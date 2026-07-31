# 結果

## 実施内容

- ShiftOut / Pass の front cap release、hold、reapply を
  `resolve_committed_pass_policy()` へ集約した。
- committed Pass 速度 floor の判断も同じ resolution へ集約した。
- front cap 状態変化の理由を enum 化し、現行ログ文字列を固定した。
- controller は現在の観測・状態・設定を request へ渡し、resolution を出力と
  内部状態へ適用する構造にした。
- current behavior を固定する回帰テストを追加した。

## 挙動不変

- config は変更していない。
- front cap の解除、維持、再適用条件は変更していない。
- committed Pass 速度 floor の条件と値は変更していない。
- start-grid breakout の速度所有権は変更していない。
- ログの prefix、出力タイミング、reason 文字列は変更していない。
- topic、service、Domain、launch、評価成果物の契約は変更していない。

## 自動検証

- `make autoware-build`
  - 25 packages finished
  - build successful
  - 既存の setuptools deprecation warning のみ
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`
  - 693 tests
  - 0 errors、0 failures、0 skipped
- `git diff --check`
  - 成功

## 次段階

実走ログで多発した次の現行規則は、今回の回帰テスト
`PreservesCurrentHorizonInfeasibleCapReapplyBehavior` に明示された。

> Pass で物理横離隔を保持していても、execution horizon が一時的に infeasible になると
> front cap を再適用する。

次の性能変更ではこの規則だけを変更し、物理接触・横離隔・target position jump を
安全条件として残しながら、短い horizon 制約だけで抜き切り速度を失わないポリシーを
A/B 評価する。

