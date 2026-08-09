# Requirements

## Purpose

追い越し中の他車予測が短時間の生速度外挿に偏り、実行可能な Pass を
一時的な予測変化で失効させる問題を改善する。

## Scope

- V2X の位置差分速度を平滑化し、縦加速度を有界に推定する。
- 相手横速度を長時間直線外挿せず、時間とともに減衰させる。
- Entry と runtime の rear-clear rollout で同じ相手運動モデルを使う。
- 失効した固定 Mission は再開しない。
- 動的待機中に同じ側の新しい完全 preflight 済み Mission が成立した場合、
  古い generation を新しい generation へ置換できるようにする。
- wall contact、実車体 overlap、target 不連続などの hard fault は従来どおり
  Recovery とする。

## Constraints

- ROS 2 topic / service / message 契約は変更しない。
- V2X message に速度・加速度フィールドを追加しない。
- 既存の Mission generation invalidation を撤回しない。
- 壁・車体・横加速度の hard constraint を緩和しない。
- 追い越し速度・加速度上限は変更しない。

## Definition of Done

- 平滑化・減衰予測を pure core の単体テストで確認する。
- 失効した Mission が同一 generation のまま再開されない。
- fresh same-side Mission だけが新しい generation として置換される。
- `multi_purpose_mpc_ros` の build / test が通る。

