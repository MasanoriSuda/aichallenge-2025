# Tasklist

- [x] 最新 HEAD と既存 forward completion を確認
- [x] 最新ログで side-by-side commit 後の失敗経路を確認
- [x] current-side predicted overlap を alternate Mission 評価へ接続
- [x] alternate Mission 成立時のみ atomic replacement を許可
- [x] forward completion に predicted sweep guard を追加
- [x] rear-clear 到達距離 admission を追加
- [x] entry pre-armを採用Missionのclosing speedへ一致
- [x] debug log を更新
- [x] pure policy test を追加・更新
- [x] `docs/spec/mpc-integration.md` を更新
- [x] Docker 内ビルド・対象テスト
- [ ] 実走確認項目を記録

## 実走確認項目

- current-side predicted overlap 後に alternate side opportunity/replace が出るか
- forward completion 開始数、rear-clear 完遂数、距離不足拒否数
- Pass から EmergencyBrake / wall Recovery へ落ちる回数
- `Pass -> Return -> Idle` 完遂率
