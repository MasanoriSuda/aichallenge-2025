# Tasklist

- [x] 現行のlow-speed専用早期returnを特定する
- [x] 動的障害物Cruise所有権の純粋判定を追加する
- [x] 停止・低速候補を通常front tactical targetへ昇格する
- [x] 新規legacy low-speed entryを通常Mission評価の後ろへ退避する
- [x] ローカル・cloud設定へロールバック可能なflagを追加する
- [x] 所有権判定の単体テストを追加する
- [x] `test_v2x_overtake_core`を実行する（689 tests passed）
- [x] `make autoware-build`を実行する（25 packages passed）
- [ ] 実走で停止・低速車の初動と完遂を確認する

## Definition of Done

- 単体テストとビルドが成功する。
- ユーザーの既存設定変更をコミットしない。
- 実走時に通常MPCC-lite Missionがlow-speed専用分岐より先に評価される。
