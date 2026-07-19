# Results

## 実装結果

- `resolve_front_danger_action()`を追加し、moving frontの非Emergency近接を
  `RelativeSpeedLimit`へ分離した。
- EmergencyBrakeと停止・低速前車の停止距離内判定は`SafetyBrake`を維持した。
- moving targetが観測されclosing speedが0以下の場合、hazard holdを即解除するようにした。
- dev3設定を以下へ更新した。
  - `v2x_front_hazard_hold_sec: 0.25`
  - `v2x_overtake_shiftout_min_closing_speed: 0.5`
  - `v2x_overtake_guard_max_lateral_accel: 4.0`
  - `v2x_overtake_target_hold_sec: 0.75`
- V2X debugへ`danger_action`を追加した。

## 検証

### Build

```text
make autoware-build
Summary: 25 packages finished [1min 16s]
[build_autoware] Build successful.
```

stderrは既存の`setup.py install is deprecated`警告のみ。

### 対象単体テスト

```text
test_v2x_overtake_core
[==========] Running 91 tests from 14 test suites.
[  PASSED  ] 91 tests.
```

追加したmoving-front action 3件、hazard safe release 1件を含む。

### パッケージ全体テスト

```text
95% tests passed, 1 tests failed out of 19
Summary: 503 tests, 0 errors, 2 failure records, 0 skipped
```

失敗は`PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`のみ。
現在の`traj_mincurv.csv`が終端重複を持たない347点へ変更されているのに対し、既存testが
「終端重複を1点削除する」ことを前提としているためで、今回のV2X変更とは独立である。

## dev3確認項目

- moving front近接時に`danger=1, danger_action=RelativeSpeedLimit`となり、`limit=0`へ
  直行しないこと。
- `SafetyBrake`回数と1秒以上のhazard hold tailが減ること。
- `ShiftOut -> Pass -> Return`が発生すること。
- EmergencyBrake、停止車、衝突時のSafetyBrakeが残ること。

