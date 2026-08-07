# Tasklist

- [x] 最新ログと現行コードの失敗経路を照合する
- [x] forward completion を latched state にする
- [x] SafeSeparation の `target clear ahead` 復帰を latch 中は抑止する
- [x] 初回 completion 距離 budget に有限 extension を含める
- [x] pre-arm を Mission scoped にする
- [x] pre-arm に時間・距離上限と retry cooldown を追加する
- [x] core regression test を追加する
- [x] Docker 内で build/test を実行する
- [x] 動的確認項目を記録する

## 動的確認項目

- `side-by-side completion admitted` 後、`forward_commit=1/.../latched=1` が rear-clear まで継続すること
- 同区間で `SafeSeparation ... target clear ahead` による FollowPrepare が発生しないこと
- hard guard 喪失時は `SafeSeparation abort` で Recovery へ入ること
- Mission の side が変化した周期に `entry_stable` と `prearm_window` が 0 から再開すること
- Mission が消えた周期は `prearm=0` となること
- 上限到達時だけ `V2X overtake entry pre-arm timed out` が出て、0.75 秒後に再評価されること
- `Pass -> Return -> Idle` の完遂数と `Pass -> FollowPrepare` 回数を比較すること
