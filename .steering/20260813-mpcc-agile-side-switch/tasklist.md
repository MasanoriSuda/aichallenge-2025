# Tasklist

- [x] 最新ログと現行のMPCC/side-replan二重ゲートを照合する。
- [x] MPCC反対側勝者の専用debounceとauthorityを実装する。
- [x] no-returnを実際のside-by-side進行に合わせる。
- [x] no-return前の複数cross-side置換を許可する。
- [x] 通常設定とcloud設定を同期する。
- [x] 単体テストを追加・更新する。
- [x] `multi_purpose_mpc_ros`をビルド・テストする。
- [ ] `make dev2`で動的効果を確認する（ユーザー実施）。

## Definition of Done

- MPCC-liteの完全な反対側Missionが、短い安定確認後に実行へ反映される。
- targetが前方にいる間は最大3回まで左右再選択できる。
- side-by-side/no-return後は選択側を維持する。
- hard guardとROSインターフェース契約を維持する。
