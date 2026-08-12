# Design

## 方針

現行のpre-armは対象ID単位であり、左右候補そのものは速度履歴の同一性条件に含まれていない。この設計を維持しつつ、実走で短すぎた時間・距離・validation leaseを調整する。

```text
前車を遠方で捕捉
  -> 30 m horizonで左右の完全Missionを継続評価
  -> 同じtargetの速度差履歴を保持
  -> freshな完全Mission + 速度条件成立
  -> その周期のpreflightを使ってShiftOut
```

## 設定

| Parameter | Before | After |
|---|---:|---:|
| `v2x_overtake_gap_lookahead_distance` | 24.0 m | 30.0 m |
| `v2x_overtake_entry_prearm_validation_hold_sec` | 0.15 s | 0.50 s |
| `v2x_overtake_entry_prearm_max_sec` | 2.0 s | 3.0 s |
| `v2x_overtake_entry_prearm_max_distance` | 8.0 m | 30.0 m |
| `v2x_overtake_entry_prearm_retry_cooldown_sec` | 0.75 s | 0.25 s |

`prearm_max_distance`は前方車までの距離ではなく、同じ対象へpre-armしている間の自車走行距離上限である。30 mから計画できるようにする設定は`gap_lookahead_distance`が担当する。

## 安全境界

- pre-arm中はベースラインを維持し、横目標を先行適用しない。
- ShiftOutは現在周期で完全Missionが成立した場合だけ許可する。
- EmergencyBrake、V2X不正、位置jump、壁・solver hard faultは従来どおりpre-armを無効にする。

## 動的確認

- 30 m以内で最初にpre-armした距離
- pre-arm timeout回数
- 前方18 m以上での`Idle -> ShiftOut`率
- 前方7.2 m以下まで遅れた`Idle -> ShiftOut`率
- `ShiftOut -> Pass -> Return`完遂率
- wall/contact/solver Recovery件数

