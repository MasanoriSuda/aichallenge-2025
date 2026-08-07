# Design

## 現象

最新走行では forward completion 認可後、12 m の local window を1回延長した。
2枠目の終端でも target 相対距離は改善中で、予測 footprint と corridor は安全、
`forward_allowed=1` だったが、固定延長回数 `1/1` のため `local distance limit` で中断した。

## 方針

通常の progress extension は従来どおり回数制限する。その回数を使い切った場合でも、
次をすべて満たすときだけ dynamic completion extension として local window を再設定する。

```text
forward completion latch が有効
AND 現在の hard/short-horizon guard が安全
AND 現在・予測 footprint と corridor が安全
AND local window 内で相対距離が規定量以上改善し、改善時刻が新しい
AND 毎周期更新した rear-clear 必要距離 <= absolute 上限の残距離
AND 毎周期更新した rear-clear 必要時間 <= absolute 上限の残時間
```

必要距離は既存の `resolve_committed_pass_forward_completion()` が現在値から算出する。
したがって admission 時の固定見積りではなく、Pass 実行中の target 相対位置・速度を使う。

## 境界

- 動的延長は lateral line、side、速度上限を変更しない。
- absolute 上限自体は再設定しない。
- 実測進捗が止まった場合は従来どおり local limit で中断する。
- 予測重複 debounce、壁・車体・Emergency・solver の fail-closed 判定は変更しない。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: SafeSeparation の動的延長入力・理由
- `mpc_controller_cpp.cpp`: 現在の完遂見積りと absolute 残量の接続、ログ、設定読込
- `config.yaml`: A/B可能な有効化スイッチ
- `test_v2x_overtake_core.cpp`: pure policy 境界テスト
