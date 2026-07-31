# Committed ShiftOut speed continuity 性能修正要件

## 目的

低速・停止車両に対する通常 Overtake で、車体横離隔をすでに確保した後も
ShiftOut の phase 完了待ちだけで前車速度付近まで失速する現象を減らす。

## 背景

`output/20260731-233748/d1/autoware.log` では、P1 は ShiftOut 中に横へ出ているが
Pass へ遷移せず、既存の committed Pass speed floor が一度も有効になっていない。
この間、停止しかけた対象車に対する closing-speed cap により参照速度が約 1.5 m/s
まで低下し、抜き切る前に Recovery へ遷移している。

## 性能変更

既存の committed Pass speed floor を、次をすべて満たす ShiftOut にも適用する。

1. ShiftOut phase である。
2. locked target の現在観測が有効である。
3. locked target との実寸ベースの完全な横離隔が成立している。
4. front-cap reapply 用の横離隔閾値以上を維持している。
5. 実行経路が物理的に成立している。
6. 実壁接触がない。
7. target が低速判定閾値以下である。

## 変更しない安全条件

- ShiftOut の front-speed cap release 条件は変更しない。
- Pass overlap latch、横衝突判定、左右選択、Recovery 条件は変更しない。
- 横離隔成立前は従来の adaptive closing-speed cap を維持する。
- target loss、経路不成立、実壁接触、低速対象外では floor を使わない。
- SafetyBrake、wall、curvature、MPC hard speed bounds は速度 floor より優先する。
- ROS 2 topic、message、launch、評価インターフェースは変更しない。

## 対象外

- 停止した P2 の競走復帰
- Recovery のリバース速度変更
- pass side 選択、相手予測、経路生成の変更
- 設定値の一律なアグレッシブ化

## Definition of Done

- 条件を満たす低速 target の ShiftOut で既存 3.0 m/s reference floor が有効になる。
- front cap は ShiftOut 中に release されない。
- 各安全条件で floor が無効になることを単体テストで確認する。
- `make autoware-build` が成功する。
- `multi_purpose_mpc_ros` のテストが成功する。
