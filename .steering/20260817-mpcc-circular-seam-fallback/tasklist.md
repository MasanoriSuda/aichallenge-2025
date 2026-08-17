# Tasklist

- [x] 実走ログと循環境界失敗を照合する
- [x] requirements/designを作成する
- [x] progress stage距離正規化helperを追加する
- [x] progress preparationをtransactionalに分離する
- [x] legacy MPC縮退と周回wrap resetを追加する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] 変更をコミットする

## 実走確認（ユーザー実施）

- [ ] `progress stage distance normalized`が周回境界で記録される
- [ ] `progress MPCC rejected invalid progress reference`が0件
- [ ] Overtake Return後にdeceleration fallbackが連続しない
- [ ] 周回境界通過後も通常速度へ復帰する
