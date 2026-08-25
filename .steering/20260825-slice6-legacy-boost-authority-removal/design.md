# Design

## Observed data flow

```text
use_boost_acceleration=true
  -> controller内の速度／操舵閾値でaccelerationとboost_modeを再決定
  -> AckermannControlBoostCommandを/boost_commander/commandへpublish
  -> boost_commanderが/control/command/control_cmdへ再publish
```

これは canonical MPCC solution から最終指令までの一対一対応を壊す。さらに同じflagが
Recovery reverse actuationと2026公式Boostを無効化するため、I/O互換層ではなくauthority selector
として働いている。

## Selected correction

1. controllerは常に通常のAckermann command publisherを生成する。
2. normal accelerationはtyped canonical commandだけから、異常時はEmergencyまたはRecoveryだけから得る。
3. legacy custom commandと中継nodeを削除する。
4. Python比較実装も同じobsolete optionを持たないようにし、将来の再接続を防ぐ。
5. source-contract testで物理削除を固定する。

## Rejected alternatives

- launchで`false`固定: 実装と別publisherが残り、直接起動や将来編集で再接続できる。
- deprecated warningのみ: authority競合の構造を残す。
- 2026公式Boostまで削除: `/awsim/cmd`はnormal trajectory authorityではなく有限回の公式actuator機能で、別責務である。

## Compatibility

- 提出launchは既にlegacy flagを`false`としており、到達可能な提出挙動は変えない。
- `/control/command/control_cmd`、`/control/command/control_cmd_raw`、`/awsim/cmd`の契約を維持する。
- `multi_purpose_mpc_ros_msgs` packageはPathConstraints／BorderCellsのため維持し、legacy messageだけ削除する。
