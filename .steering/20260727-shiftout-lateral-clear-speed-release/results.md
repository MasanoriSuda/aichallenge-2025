# Results

## 実装結果

- locked targetの現在横離隔を、Passだけでなくcommit済みShiftOutでも判定する。
- ShiftOut / Pass中に現在横離隔が1.50 m以上となった場合、locked target由来の
  front-speed capを解除する。
- 解除状態をShiftOutからPassへ引き継ぎ、横離隔が1.30 m未満へ縮んだ場合は
  capを再適用する。
- Pass継続用のfront-overlap exclusion latchはPassでのみ確定し、
  ShiftOut中の一時的な離隔をcommitted Pass continuityへ流用しない。
- 別の前方車、EmergencyBrake、front risk、wall/corridor、curve、
  domain/global速度、`a_max`、solver/odometry guardは変更していない。
- config値の変更はない。

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
Summary: 648 tests, 0 errors, 0 failures, 0 skipped
```

```text
git diff --check
成功
```

ROS topic/service、Domain、評価成果物のインターフェース変更はない。

## 実走確認

ユーザーの`make dev2`で次を確認する。

- P2が壁側へ逸れて横差が1.50 m以上になった周期に、
  `OvertakeLine execution front cap: Released, phase=ShiftOut`が出る。
- 同じ周期付近で`cap_release=1`、`speed_cap=0`、`desired_v=11.11`となる。
- 1.30〜1.50 mのヒステリシス帯では解除を維持し、1.30 m未満で
  `Reapplied`となる。
- Emergency、別の前方車、wall/corridor不成立では従来の減速・中止が残る。
