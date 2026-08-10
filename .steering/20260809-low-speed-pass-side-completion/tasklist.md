# Tasklist

- [x] 最新ログと front-to-side 投影境界を照合する
- [x] corridor stop 判定を phase / vehicle classification 対応へ変更する
- [x] 保持 pass target の base-bounds / static-wall preflight を追加する
- [x] 保持 Pass の状態ログを追加する
- [x] core 単体テストを追加・更新する
- [x] `multi_purpose_mpc_ros` をビルドする
- [x] 実走確認項目を記録する

## Definition of Done

- Pass 中に target が side / clearance 帯へ移り live planner が inactive になっても、保持
  経路が再検証済みなら direct Pass を継続する。
- Shift、front 残存、live infeasible、保持経路の壁不成立では停止する。
- 車両群 clear 後は既存 Rejoin と MPC handoff へ移行する。
- core 単体テストとパッケージビルドが成功する。

## 実走確認

- `Low-speed pass retained for side completion` が出ること
- 同じ event で `live vehicle corridor unavailable` が出ないこと
- 対象車 rear-clear 後に Rejoin / MPC handoff へ進むこと
- 保持経路が壁不成立なら従来どおり停止すること
- wall contact、Recovery、Reverse が増えていないこと

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- 追加・更新した corridor 判定テスト: 2 / 2 成功
- `test_v2x_overtake_core`: 437 / 437 成功
