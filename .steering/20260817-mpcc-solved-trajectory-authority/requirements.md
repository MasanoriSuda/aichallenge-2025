# Requirements

## 目的

progress-contouring MPCC が実際に解いた可行軌道を、次周期の追い越し実行判定へ
戻す。名目 Mission 経路だけを見た壁予告によって、壁・target corridor 内にある
MPCC 解まで捨てて `FollowPrepare` / `Recovery` へ落ちる不整合を減らす。

## 要件

- MPCC 解から stage ごとの `[e_y, s]` を抽出し、同時に適用した横境界を記録する。
- 同一 target、Mission generation、side、phase の短時間の解だけを再利用する。
- 現在の進捗に合わせて解軌道を再整列する。
- 現在の静的壁 footprint で物理再検証できた解だけを実行 authority とする。
- authority が有効な場合、名目経路由来の soft な runtime wall preplan warning を
  抑制し、解軌道を次の receding-horizon warm-start に使う。
- 現在車体の壁接触・hard wall margin 違反、map 不明、target 不連続、緊急制動、
  solver failure は従来どおり authority を無効化する。
- legacy MPC、Recovery、ROS 2 interface、評価 schema は変更しない。
- `config.yaml` と `result-summary.json` の既存ユーザー変更は変更・stageしない。

## Definition of Done

- 解軌道抽出の入力検証と境界検証に単体テストがある。
- 新鮮でcontext一致する解だけが再利用される。
- soft warning抑制時もhard wall guardが維持される。
- package build/testが成功する。
- 今回の変更だけをコミットする。
