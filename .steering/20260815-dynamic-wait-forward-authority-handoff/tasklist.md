# Tasklist

- [x] 直近走行のDynamicMissionWait、SafetyBrake、Pass速度capの因果を照合
- [x] forward authorityのfail-closed pure coreを追加
- [x] Behavior SafetyBrake仲裁へforward authorityを接続
- [x] fresh same-side Pass continuationへfront-cap状態を引き継ぐ
- [x] pure core単体テストを追加
- [x] package build（`make autoware-build`）
- [x] package test（25/25 passed、集計1145 tests / 0 failures）
- [x] 関連ファイルだけをcommit
- [ ] `make dev2`で動的効果確認（ユーザー実施）

## 動的合格条件

- DynamicMissionWait full prefix成立後に縦距離だけでSafetyBrakeへ落ちない。
- fresh same-side Pass再開ログで`front_cap_handoff=1`となる。
- 再開直後にclosing speedが0.5 m/sへ不必要に戻らない。
- 壁接触、actual overlap、予測不成立時の既存guardが維持される。
