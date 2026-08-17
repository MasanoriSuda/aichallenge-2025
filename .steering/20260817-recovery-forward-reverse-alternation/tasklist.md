# Tasklist

- [x] 最新runと既存方向切替処理を照合する
- [x] requirements/designを作成する
- [x] Forward失敗trackerをpure C++ coreへ追加する
- [x] controllerのaggressive retryへ統合する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] 変更をコミットする

## 実走確認（ユーザー実施）

- [ ] Forward失敗cycleがログで累積される
- [ ] 設定回数到達後に`next_direction=Reverse`が出る
- [ ] forced Reverse中はForward候補へ戻らない
- [ ] 壁接触セルが減少し、Recoveryから通常走行へ復帰する
