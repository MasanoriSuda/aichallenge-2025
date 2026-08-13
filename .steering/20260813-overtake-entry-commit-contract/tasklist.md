# Tasklist

- [x] 最新ログと現行entry/MPCC-lite接続を照合する。
- [x] 監視距離と新規entryコミット距離を分離する。
- [x] pre-arm走行距離上限を15 mへ変更する。
- [x] FSMの実行可能Missionを有限rear-clear必須へ変更する。
- [x] MPCC-liteの新規entry権限を完全Mission必須へ変更する。
- [x] 通常設定とcloud設定を同期する。
- [x] `v2x_overtake_core`を含むパッケージ単体テストを通す（25/25）。
- [x] `multi_purpose_mpc_ros`を含むAutoware workspaceをビルドする。
- [ ] `make dev2`で動的効果を確認する（ユーザー実施）。

## Definition of Done

- 不完全なMPCC-lite prefixが新規横Missionを開始しない。
- 15 mより遠方の前車は観測・準備対象のままで、横追い越しは開始しない。
- 既存インターフェース契約を変更しない。
- 対象単体テストとパッケージビルドが成功する。
