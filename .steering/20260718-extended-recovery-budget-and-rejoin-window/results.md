# Extended Recovery Budget and Rejoin Window Results

実施日: 2026-07-18
判定: Rejected

## 実験条件

- `make autoware-build`: 成功（25 packages）
- `make dev3`: 2回
- 実験値: `max_escape_steps=16`, `rejoin.timeout_sec=10.0`
- 維持した安全上限: 最大後退3.0 m、単step 0.40 m、0.8 m/s、4.0秒

## Run 1: `output/20260718-165739`

| 車両 | 結果 | 観測 |
|---|---|---|
| D1 | `maneuver_direction_unknown`でSafeStop | `invalid_grid`かつrear contact。step 0、移動0 m |
| D2 | `escape_not_confirmed`でSafeStop | contactを97から71へ低減したが、step 4、0.828 mで候補が`contact_worsened`となり`rear_static_blocked` |
| D3 | `escape_not_confirmed`でSafeStop | contactを115から92へ低減したが、step 1、0.235 mで同様に候補喪失 |

16 stepの追加予算へ到達する前に停止したため、このrunではstep延長の効果なし。
LowSpeedRejoinへ到達した車両はなく、10秒猶予は未評価。

## Run 2: `output/20260718-170218`

| 車両 | 結果 | 観測 |
|---|---|---|
| D1 | 実験終了時も走行 | 終了直前の速度約5.6 m/s |
| D2 | `rejoin_timed_out`でSafeStop | step 10、2.174 mで`escape_confirmed=1`。LowSpeedRejoin中の横偏差は1.207 mから0.915 mまでしか収束せず、10.0秒でtimeout |
| D3 | 実験終了時も走行 | 終了直前の速度約2.7 m/s |

D2は従来上限と同じ10 stepで2.0 mのescape条件へ到達したため、11〜16 stepを使用しなかった。
LowSpeedRejoinはstatic、V2X、gearのhard gateがclearな状態でも、10秒以内に横偏差0.5 mへ収束しなかった。

## 安全性確認

- 3.0 m後退上限超過なし（最大2.174 m）
- contact悪化後の新しい後退step開始なし
- V2X不完全状態での後退なし
- LowSpeedRejoin timeout後はSafeStop
- 実験後に`make down`で全コンテナ停止

## 結論

`max_escape_steps=16`と`rejoin.timeout_sec=10.0`は採用しない。実験前の10 step / 5.0秒へ戻す。

次の修正候補はパラメータ延長ではなく、以下のロジック変更とする。

1. LowSpeedRejoinの横偏差収束を改善する経路・操舵制御。
2. 後退中にstatic候補が`contact_worsened`へ変わった場合、Driveへ戻して直ちに`escape_not_confirmed`とするのではなく、安全停止後に候補を再評価する遷移。

いずれも安全ゲートを維持した別ステアリングで設計・単体テスト・dev3実験を行う。
