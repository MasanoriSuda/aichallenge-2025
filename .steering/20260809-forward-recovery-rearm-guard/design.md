# Design

## 1. 有界rearm guard

Forward escape由来の`LowSpeedRejoin -> Normal (RejoinComplete)`でのみ、adapterへ
次のanchorを保存する。

- steady clock開始時刻
- コース進捗開始値

ガード判定はpure core helperで行う。

```text
guard_active =
  enabled && simulation && armed &&
  elapsed < maximum_duration &&
  forward_progress < release_distance &&
  !new_collision && !wall_evidence && !solver_fallback
```

初期設定は最大3.0秒または前進3.0 m。どちらかへ先に到達した時点で解除する。
無進捗でも3秒で解除されるため、実際に抜けられない閉塞を無期限に隠さない。

## 2. detector統合

`DetectorInput`へ`recovery_rearm_guard_active`を追加する。active中は
`StuckRejectReason::RecoveryRearmGuard`で観測窓をリセットする。

通常のSafetyBrakeやFollow速度制限はそのまま最終制御へ残す。変更するのは
Recovery FSMへの再entryだけであり、前方車への停止保護を無効化しない。

## 3. hard evidence

次はガードを即解除し、既存Recoveryへ渡す。

- ガード開始後に受信した新しいAWSIM collision
- current footprintのwall evidence
- MPC solver fallback

ガード開始前のcollision receiptは解除根拠にしない。直前の接触から復帰した場合に、
古い5秒collision hintだけでガードが常時無効になるのを避ける。

## 4. 状態管理と観測性

- session reset、新しいRecovery episode開始、設定無効時にanchorを消去する。
- `armed`ログに時間／距離上限を出す。
- `released`ログにelapsed、progress、hard evidenceを出す。
- detectorログは`reject=recovery_rearm_guard`を出す。

## 5. 影響範囲

- `stuck_recovery_core.hpp/.cpp`: pure判定、detector reject reason
- `mpc_controller_cpp.cpp`: config読込、anchor、arm/release、起動ログ
- `config.yaml`: simulation向け有効値
- `test_stuck_recovery_core.cpp`: helper／detector test
- `docs/spec/mpc-integration.md`: 永続仕様
