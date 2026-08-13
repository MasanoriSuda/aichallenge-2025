# Requirements

## 背景

`output/20260814-000850` では MPCC-lite が実行中 Mission と反対側の、壁・車体・横加速度制約を満たす短期軌道を選んでいる。しかし実行権は rear-clear と Return まで成立した完全 Mission に限定されているため、候補が `hard=1` でも `authority=none` のままになり、実際の左右切替は 0 回だった。

また、左右評価周期 0.15 s に対して切替安定時間が 0.10 s のため、1 回だけ明確に優位な候補が得られても次回評価で消えると commit できない。

## 目的

- 壁・対象車・body-clear を満たす MPCC-lite の receding prefix に、no-return 前の限定的な実行権を与える。
- 新しい解が一時的に得られない場合は、実行中の直近 feasible trajectory を維持し、Follow/Return へ即時に落とさない。
- 現在側より十分大きく優位な反対側候補は、1 回の評価でも早期 commit できるようにする。

## 制約

- actual wall contact、runtime hard fault、target continuity 喪失、body overlap は緩和しない。
- no-return 後、SafeSeparation 中、rear-clear 後の左右切替は許可しない。
- ROS topic/service、Domain、提出インターフェースは変更しない。
- 完全 Mission の従来 admission は維持する。

## 完了条件

- prefix admission と authority の単体テストが追加される。
- `authority=replace` と prefix/full の区別をログから確認できる。
- 通常の marginal な候補は debounce、明確な候補は即時 commit となる。
- package build と test が成功する。
