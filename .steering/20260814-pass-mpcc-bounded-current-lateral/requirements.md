# Requirements

## 背景

`output/20260814-211535` では、FollowPrepare から Pass へ戻す phase-aware
replacement 自体は動作した。一方、失敗した2件では現在横位置から約 1.45--1.47 m
離れた goal を選び直し、壁接触または target overlap から Recovery へ移行した。

## 目的

- Pass由来の同側rolling replanでは、現在の安全な横位置を第一候補にする。
- 同側継続を理由に大きな横移動を再発行しない。
- rear-clearへ向かう縦進捗を優先し、必要な横補正だけを許す。
- 重いpreflightへ渡すgoal数を早期に削り、制御callbackの計算負荷を下げる。

## 制約

- ROS 2 topic、message、service、launch entry、評価結果schemaは変更しない。
- 新規entry、cross-side replan、Recoveryの既存判定は変更しない。
- wall、target footprint、横加速度、速度のpreflightは省略しない。
- `aichallenge/result-summary.json` の既存ユーザー変更には触れない。

## Definition of Done

- rolling same-side assessmentに現在横位置を含める。
- 同assessmentの横補正を設定値以内に限定する。
- 上限超過goalを重いpreflight前に棄却する。
- 左右対称性と実ログ相当の大補正棄却を単体テストする。
- 対象packageをビルドし、単体テストが通る。

