# 追い越しRecovery急減速調整 タスクリスト

## 調査・設計

- [x] `20260724-231447`のMCAPとログを時刻同期する
- [x] 急減速前に速度指令が低下することを確認する
- [x] Recovery 3.0 m/s制限の設定・適用経路を特定する
- [x] SafetyStop、Stuck Recovery、V2X欠損が主因でないことを確認する
- [x] 既存ステアリングとの作業境界を確認する

## 設定

- [x] Recovery速度を3.0から5.0 m/sへ変更する
- [x] 制限enabled、wall、横加速度、Emergency設定が不変であることを確認する
- [x] 対象設定以外に参加者制御差分がないことを確認する

## 静的検証

- [x] `make autoware-build`
- [x] `git diff --check`

## 動的検証

- [x] dev2を同条件で起動する
- [x] d1/d2を2周以上走行する
- [x] d1/d2の5トピックMCAP生成を確認する
- [x] Recovery開始前後のcommand/actual/acceleration/V2X車間を抽出する
- [x] Recovery回数、理由、最低速度、最小加速度を比較する
- [x] wall/contact/stuckとlap timeを比較する
- [x] 追い越し完遂有無を確認する

## 完了条件

- [x] `results.md`へ比較値と採用判定を記録する
- [x] 設定だけで改善可能な範囲と、ソース変更が必要な残課題を分離する
