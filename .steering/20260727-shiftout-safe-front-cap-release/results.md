# Results

## 実装結果

- ShiftOut / Passの横目標horizonをfront-cap解除判定より先に評価する。
- 横離隔1.50 m到達だけではlocked target由来capを解除しない。
- pass側横目標への到達と、次の3条件がすべて非制限であることを要求する。
  - lateral acceleration
  - wall bound
  - static map wall
- 解除後でも横目標未到達、horizon制限、横離隔1.30 m未満、target観測欠損の
  いずれかでcapを再適用する。
- Behavior側はOvertakeLineが確定した解除状態だけを利用し、解除判断を二重化しない。
- start-grid breakout、closing speed設定、Recovery条件、config値は変更していない。

## 検証

```text
make autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

`multi_purpose_mpc_ros`のsetuptools deprecation warningのみで、
コンパイルエラーはない。

```text
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose
100% tests passed, 0 tests failed out of 24
Summary: 648 tests, 0 errors, 0 failures, 0 skipped
```

```text
git diff --check
成功
```

ROS topic/service、Domain、評価成果物のインターフェース変更はない。

## 実走確認

ユーザーの`make dev2`で次を確認する。

- ShiftOut中に横離隔が1.50 mへ到達しても、`lateral_complete=0`または
  `horizon_clear=0`ならcapを解除しない。
- 壁側へclamp中は`cap_release=0`、`speed_cap=1`を維持する。
- 横目標到達後、`horizon_clear=1`かつ横離隔成立時だけ
  `OvertakeLine execution front cap: Released`となる。
- `actual footprint wall margin violated`と壁接触が前回より減る。
- 追い越し成功率は実走結果で別途評価する。
