# Results

## 実装結果

- 追い越し実行時の進捗MPCCへ5状態3入力モデルを追加した。
  - state: `[e_y, e_lag, e_psi, v, theta]`
  - input: `[a, kappa, v_theta]`
- 既存のstage corridor、壁制約、曲率上限、加減速度上限をhard constraintとして再利用した。
- stageごとの速度referenceとhard capを分離した。
- Pass中はstage速度weightを `8 -> 24`、terminal速度weightを `12 -> 45` へ強める初期値とした。
- 入力変化costを加速度、曲率、仮想進捗へ追加した。
- 拡張QP専用のpersistent OSQPと5x3 warm startを追加した。
- 拡張QPの準備・solve・解変換が失敗した周期は、現行3状態MPCCへ戻す。
- 拡張解は既存3x2形式へ変換し、従来の壁再検証、予測、操舵post-processをそのまま通す。

## 設定

`config/config.yaml` で `progress_contouring_extended_dynamics_enabled: true`。
新しいweightはすべてyamlから調整可能。

## 検証結果

### ビルド

```text
make autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

setuptoolsの既存deprecation warning以外のbuild errorはない。

### テスト

```text
colcon test --packages-select multi_purpose_mpc_ros
100% tests passed, 0 tests failed out of 28
Summary: 1284 tests, 0 errors, 0 failures, 0 skipped
```

追加確認:

- 直線上で速度状態と仮想進捗が進むこと
- 実進捗と仮想進捗の差がlag stateへ現れること
- committed Passで速度costだけが増え、hard capは緩まないこと
- 5x3解を既存3x2形式へ正しく変換できること

## 試走時の確認点

1. `ShiftOut -> Pass` 後の最低速度とrear-clearまでの時間。
2. `Extended velocity-progress MPCC unavailable; using 3-state MPCC` の回数と理由。
3. `Pass -> Return` 成功数、`Pass -> FollowPrepare/Recovery` 数。
4. wall contact、solver failure、急な左右切替が増えていないか。

## 残課題

- 拡張QPは現時点で左右branchを並列非同期solveしていない。
- 拡張QP自身の2回目RTI-SQP refinementは未実装で、既存3状態MPCC側だけに残る。
- weightの最終値は `make dev2` の動的結果でA/Bする。
- Recovery/Reverseは今回変更していない。
