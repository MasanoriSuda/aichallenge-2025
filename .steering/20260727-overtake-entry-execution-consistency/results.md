# Results

## 実装結果

- 通常の新規ShiftOut候補を、実行時と共通のhorizon評価で事前検証するようにした。
- path bound、壁余裕、実車体static-map footprint、横加速度のいずれかで
  実行不能な側は、ShiftOutへcommitする前に棄却する。
- 実行中にstatic wall系の幾何失敗が発生した場合、同一対象・同一側を
  `v2x_overtake_line_entry_retry_cooldown_sec`の間は再選択しない。
- 既定値は1.0秒。反対側探索、通常Follow、Emergency、物理接触、solver guardは
  変更していない。
- `v2x_prediction_use_course_lateral_velocity: false`、速度cap、車体寸法、壁余裕、
  横加速度の既存値は変更していない。

## 静的確認

- `git diff --check`: 成功
- ROS topic/service、Domain、評価成果物のインターフェース変更なし
- 全ファイルへの`ament_uncrustify`適用は既存コード全体に大規模な差分を生むため
  実施しない。今回の変更と無関係な一括整形は行っていない。

## ビルド・テスト

```text
make autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

```text
colcon test --packages-select multi_purpose_mpc_ros
Summary: 675 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result`は対象package外に残っている
`build/joycon_contract_guard/package.xml`欠損を警告したが、
`multi_purpose_mpc_ros`のテスト結果には失敗がない。

## 未確認

実走効果は`make dev2`で確認する。期待する差分は次のとおり。

- 実行不能な側では`ShiftOut execution preflight rejected`が出て、
  `Idle -> ShiftOut -> Recovery`へ進まない。
- 実行中の幾何失敗後は`OvertakeLine entry retry blocked`が出て、
  同一対象・同一側へ即再突入しない。
- 通過可能な候補は`Idle -> ShiftOut -> Pass -> Return -> Idle`まで継続する。
