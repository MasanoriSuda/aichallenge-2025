# V2X共通進捗・停止デッドロック正式修正 Design

作成日: 2026-07-17
状態: Complete（fail-closed runtime制約あり）

## 1. 共通経路射影

ROS非依存の`v2x_overtake_core`へ、2次元polyline上の前方進捗射影helperを追加する。

入力は共通reference path、現在waypoint、circular設定、自車位置、対象車位置・速度、
look-behind/look-ahead、最大横距離とする。現在waypointの少し後方から前方だけを順に走査し、
自車と対象車を同じpolylineへ射影する。

出力:

- `forward_distance_m`: 対象進捗 - 自車進捗
- `lateral_m`: 対象車の経路横偏差
- `along_track_speed_mps`: 対象速度の経路接線成分
- `cross_track_distance_m`: 射影距離
- `segment_index`

対象速度が有意で、候補segmentと逆向きの場合はそのsegmentを採用しない。探索範囲を1周未満に
制限することで、物理的に近いがコース進捗が遠い別のヘアピン枝を採用しない。

## 2. V2X behaviorへの統合

`evaluate_v2x_behavior()`で1周期につき一度reference path点列を作り、各active V2X車両を射影する。

- front/danger/risk/overtake用の距離と前車速度: 共通進捗値
- locked targetの経路距離: 共通進捗値
- side vehicle: 従来の自車接線による物理近接
- low-speed clearance hold: 共通進捗が有効ならその距離

`v2x_front_progress_detection_distance`を既存Follow/停止距離へ`max()`で合成し、24 mから判断を開始する。
速度制限は既存required decel、curve guard、SafetyBrakeへ渡すため、新しい直接制御は作らない。

## 3. 停止デッドロック解除

既存の`stuck_recovery`をSIMの3 Domainで有効化し、短いstepごとに停止・再評価する形へ強化する。
実装は次を満たす。

1. SafetyBrake/Follow(frontあり)は`deliberate_stop`として検出から除外。
2. 先頭車の停止継続、前進要求、pose/path無進捗、solver/contact証拠を確認。
3. occupancy map上のswept footprintを検査。
4. freshかつ完全なV2X観測で後退/前進corridorを検査。受信freshnessは受信clock、source stampの
   単調性・ageはsource clock内で検証し、異なるepochを直接減算しない。
5. map contactあり、またはmapと物理接触が不一致の場合は0.40 mだけ動き、停止して候補を再選択する。
   contact悪化と単step時間上限もepisode全体を即破棄せず、停止・再評価へ戻す。
6. 後方車でreverseが塞がれる場合、Forward Straight / Left / Rightのうちstatic / V2X双方が安全で、
   終端heading errorを最も減らす候補へ切り替える。候補の操舵符号を実commandへ渡す。
7. gear reportとBoost停止を確認後、距離・時間・step数上限付きで実行する。

これにより停止列は先頭から解除され、後続はSafetyBrakeを保ったまま前方間隔が回復するのを待つ。

## 4. Config

```yaml
stuck_recovery:
  enabled: true
  domain_enabled:
    1: true
    2: true
    3: true
  simulation_only: true

mpc:
  v2x_front_progress_detection_enabled: true
  v2x_front_progress_detection_distance: 24.0
  v2x_front_progress_lookbehind_distance: 3.0
  v2x_follow_speed_limit_enabled: true
  v2x_overtake_guard_min_front_distance: 5.0
  v2x_overtake_close_follow_enabled: false
  v2x_overtake_front_velocity_limit_enabled: false
  v2x_moving_follow_speed_margin: 0.8
  v2x_moving_safety_brake_distance: 3.0
  v2x_moving_safety_brake_margin: 1.0
  v2x_moving_safety_brake_time_headway: 0.8
```

数値は2025 AWSIM / final_ver3向けローカル暫定値で、2026公式仕様ではない。

## 5. Compatibility / rollback

- ROS topic/service/messageは変更しない。
- participant package内だけを変更する。
- 共通進捗は`v2x_front_progress_detection_enabled=false`でlegacy接線判定へ戻せる。
- recoveryは`stuck_recovery.enabled=false`またはDomain overrideで即時無効化できる。
- 実車は`simulation_only=true`で無効になる。

## 6. Validation result

- 対象test: `stuck_recovery_core`、`v2x_overtake_core`、`recovery_footprint`は全suite成功。
- build: `make autoware-build`成功。
- dev3: `output/20260717-093647`では前走車への連続追突による3台停止列は再発せず、P1/P3は走行継続。
- P2の独立した深いwall接触では、4 step・累積1.219 mでmap contactを29から0へ減らした後、
  次のsafe rolloutが存在せずSafeStopした。安全gateを弱めず、通常制御のwall侵入抑制を別課題とする。
- 最終binary: `output/20260717-094634`でも3台停止列は再発せず、P1はWP 323、P2はWP 318へ到達。
  `ForwardLeft / steering=+0.250 rad`の候補選択を確認した。P3のWP 121深いwall侵入と、P2周回端の
  内部circular閉路点によるdegenerate segmentを確認し、ゼロ長閉路点をskipする修正・testを追加した。
- 修正後`output/20260717-095229`ではWP 193までdegenerate segmentと車両同士の追突は再発しなかった。
  一方、P3の深いwall侵入を安全に離脱できず、P2/P1も停止した。wall侵入抑制、side-only deadlock分類、
  追加primitiveの安全設計は残課題であり、現行Recoveryは安全な候補がなければfail-closedとする。
