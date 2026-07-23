# オーバーテイク根本原因修正 タスクリスト

## 調査・設計

- [x] 最新 dev2 ログから失敗時系列を特定する
- [x] Follow cap、ShiftOut、Pass、Start reset のコード経路を特定する
- [x] topic / service / evaluation 契約に変更が不要であることを確認する
- [x] 最小実装案と閾値を確定する

## 実装

- [x] Start で V2X tracking を保持する
- [x] ShiftOut / Pass 用 moving-front clearance 仲裁を追加する
- [x] close-follow入口とstage実行可能速度による完遂距離を揃える
- [x] curve entry例外を実測速度差と近距離entry範囲で限定する
- [x] MPC 統合仕様を更新する

## テスト

- [x] 状態別 clearance cap の単体テスト
- [x] 入口・完遂距離・curve entry例外の単体テスト
- [x] 既存 `test_v2x_overtake_core` の回帰確認（599 tests pass）
- [x] 対象パッケージの build / test
- [x] interface 契約差分の確認
- [x] dev2 で動的確認（速度cap解除まで確認、`Pass -> Return`は未達）

## Definition of Done

- [x] d1 を対象速度未満へ落とす Overtake 中の clearance cap が再現しない
- [x] Ready 中の V2X 履歴が同一セッション Start で失われない
- [x] 負の速度差または遠方targetではcurve entryせず、近距離の正速度差で開始する
- [ ] locked targetを後方へ抜いてPassからReturnへ遷移する
- [x] 既存 fail-safe と ROS 2 / 評価インターフェースを維持する
- [x] 実行した検証と未完遂理由を報告する
