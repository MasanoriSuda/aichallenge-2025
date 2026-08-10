# Design

## 観測された失敗経路

1. 車間 3.55 m、相手速度 1.69 m/s、自車速度 2.79 m/s で完全な Mission が成立。
2. 相対速度 1.10 m/s は条件を満たしたが、確認時間は 0.05/0.30 s。
3. Follow/pre-arm を継続し、約 0.5 s 後に車間 2.19 m で SafetyBrake。
4. OSQP fallback が reverse-only Recovery をラッチし、直接ハンドオフ対象外になった。

## 方針

通常の 0.3 s 確認を全体では緩めず、次の限定条件を満たす `slow blocker urgent entry`
だけを immediate execution override として扱う。

- current complete Mission validated
- entry hard guards clear
- front vehicle is the current target
- measured relative speed >= normal entry minimum
- relative-speed stable time >= 0.05 s
- front speed <= 2.0 m/s
- front distance is within `[guard_min_front_distance, guard_min_front_distance + 1.0 m]`

この条件なら今回の 3.55 m 地点で lateral ownership を OvertakeLine へ渡せる。
相対速度が負、遠距離、単一サンプル、EmergencyBrake、solver recovery、壁／Mission
不成立では従来の pre-arm を維持する。

## 設定

- `v2x_overtake_slow_blocker_urgent_entry_enabled: true`
- `v2x_overtake_slow_blocker_urgent_entry_max_front_speed: 2.0`
- `v2x_overtake_slow_blocker_urgent_entry_distance_margin: 1.0`
- `v2x_overtake_slow_blocker_urgent_entry_confirm_sec: 0.05`

## ログ

発火時に target、side、distance、front speed、relative speed、stable time を1回記録する。
既存の Behavior 遷移ログで `Follow -> Overtake` と照合する。
