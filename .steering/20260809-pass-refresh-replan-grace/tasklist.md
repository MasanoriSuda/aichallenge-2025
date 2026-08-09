# Tasklist

- [x] `20260809-220122/d1`の失敗経路を確認
- [x] 変更境界を文書化
- [x] coreのreplan grace判定を実装
- [x] controller状態と再試行を実装
- [x] 単体テストを追加
- [x] `make autoware-build`
- [x] package test（25件、全体集計958 tests、failure 0）
- [x] `git diff --check`とユーザー変更非干渉を確認

## 動的確認

- [ ] `Pass refresh replan grace started`後も加速度指令がFollow制動へ落ちない
- [ ] grace中にrefresh成功して`Pass -> Return`へ到達する
- [ ] hard faultではgraceを使わず従来どおり離脱する
- [ ] 期限切れ時にSafeSeparationへ移行する
