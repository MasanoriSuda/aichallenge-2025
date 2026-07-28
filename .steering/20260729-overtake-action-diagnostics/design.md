# 設計

## ログ形式

`OvertakeLine action:`を固定prefixにし、次の値を一行で出す。

- `action`
- `phase`
- `target`
- `mission_side` / `behavior_side` / `candidate_side`
- `wall_contact` / `wall_margin` / `wall_unknown`
- `return_blocked` / `rear_clear`
- `side_ready` / `side_abort`
- `watchdog`
- `wp_id`

## 発火単位

前回Actionをcontrollerに保持する。

- 現在Actionが`None`: 保持値を`None`へ戻し、ログを出さない。
- 現在Actionが前回と同じ: 継続中なのでログを出さない。
- 現在Actionが前回と異なる: 新規イベントとして一度ログを出す。

Actionの選択後、副作用を実行する前に記録するため、entry rejectの早期returnも取得できる。

## 互換性

- ログは既存の`debug_log_enabled`に従う。
- Action選択とcontrollerの副作用は変更しない。
- config schema、topic、service、launchは変更しない。
