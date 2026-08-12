# Tasklist

- [x] 最新ログから逸走経路を特定する
- [x] Proレビューと直接原因を照合する
- [x] direct-control入口の減速成立性resolverを追加する
- [x] 高速時にMPC経路追従へフォールバックする
- [x] direct操舵範囲を基準曲率中心へ変更する
- [x] pure core単体テストを追加する
- [x] 対象packageをビルド・テストする
- [x] 次回試走の確認項目を記録する

## 検証結果

- `v2x_overtake_core.cpp` のhost側構文チェック: 成功
- `git diff --check`: 成功
- Docker内 `colcon build`: 25 package成功
- Docker内 `colcon test --packages-select multi_purpose_mpc_ros`: 25/25成功
- 追加した入口・操舵境界テストの明示実行: 5/5成功

## 次回試走の確認項目

`make dev2` で停止・低速車両へ高速接近する場面を確認する。

- 約9.8 m/s、前方約9.2 mでは
  `Low-speed direct control deferred to MPC` が一度だけ出ること
- 同じ条件で直後に
  `Low-speed pass shift control entered` が出ないこと
- MPCの経路追従で減速・横移動し、約2秒後の
  `wall clearance margin violated` とReverse Recoveryを再現しないこと
- 十分に低速化した後は、必要に応じてdirect-controlへ移行できること
- 通常OvertakeのShiftOut / Pass / Return成功率を悪化させないこと
