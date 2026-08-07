# Tasklist

- [x] 現行ログと候補選択経路を照合する
- [x] 要件・設計を作成する
- [x] コース役割評価を core へ追加する
- [x] Mission candidate と左右ランキングへ反映する
- [x] 設定読み込み・起動ログを追加する
- [x] 選択理由ログを追加する
- [x] 単体テストを追加する
- [x] build/test を実行する
- [x] 実走確認項目を記録する

## Definition of Done

- 直線から開いたインへ入って出口外側になる候補を選べる。
- 外側開始でも曲率反転前に抜き切れる候補は維持される。
- rear-clear 前に外側から内側へ反転する候補より、切替不要候補を選ぶ。
- tight/blocked なイン候補は既存 hard gate で選ばれない。
- 両側不成立時に追い越しを強制しない。
- `multi_purpose_mpc_ros` の対象テストが成功する。

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 tests、880 assertions、失敗0
- `git diff --check`: 成功

## 実走確認

`make dev2` で次を確認する。

- 起動ログに `rear_clear_side=enabled/reserve=2.00 m` が出る。
- 候補ログの `entry_role`、`rear_clear_role`、`full_track_transition` を比較する。
- 両側成立時に `full_track_transition=1` より `0` の候補が選ばれる。
- 入口インから出口外へつながる場合に `inner_to_outer=1` が出る。
- `Pass -> Return -> Idle`、rear-clear所要時間、SafeSeparation、接触を現行runと比較する。

動的効果確認は未実施。速度・壁余裕・Recovery条件は今回変更していない。
