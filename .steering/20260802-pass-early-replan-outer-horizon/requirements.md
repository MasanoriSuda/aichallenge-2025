# Requirements

## 背景

`output/20260802-212414` では `ShiftOut -> Pass` が8回成立した一方、
same-side extensionは7回要求して成功0回、`Pass -> Recovery` が6回だった。
extension失敗の6/7は `dynamic rear-clear Pass distance exceeds budget` であり、
検証済みPass終端まで約3 mになってから最大12 mのreplacementを作る現行判断では遅い。

また、候補探索の速度rolloutは基準trajectoryの曲率と通常走行用 `ay_max` を使う一方、
Overtake実行時は6.0 m/s²の横加速度guardで棄却される。固定したFrenet左右がヘアピン途中で
実質的なイン側へ変わる場合や、イン側オフセットで曲率が増える場合を、rear-clearまでの
候補経路評価へ反映できていない。

## 目的

- rear-clear予測に対してreplacement可能距離が不足する前に、同側再計画を要求する。
- replacementは必要なrear-clear距離を収める範囲で、absolute Pass上限まで検証可能にする。
- 外まくりとして開始する候補は、rear-clearまで同じ側が実質インへ反転しないことを確認する。
- オフセット経路の曲率とOvertake横加速度上限を速度rolloutへ反映する。
- 実行時guardで初めて失速・棄却される候補を、ShiftOut前または同側replacement時に除外する。

## 制約

- ROS 2 topic、message、service、launch、提出物の契約は変更しない。
- 変更は `aichallenge_submit/multi_purpose_mpc_ros` と本steeringへ閉じる。
- target IDとpass sideをextension中に変更しない。
- intentionally selected inside attackを一律禁止しない。外まくりとして確定したmissionだけ、
  rear-clear前のinside反転を禁止する。
- absolute Pass距離32 m、absolute time 10秒、extension回数1回は維持する。
- wall、current-footprint overlap、Emergency、solver guardは緩和しない。
- 生成済み `aichallenge/result-summary.json` のユーザー変更は触らない。

## Acceptance criteria

- Pass終端3 m手前だけでなく、rear-clear予測とreplacement距離から早期再計画できる。
- 12 mを超えるrear-clear replacementも、32 m absolute上限内ならfull static preflightされる。
- 外まくりmissionがrear-clear前に実質インへ変わる候補を理由付きで棄却する。
- kinematic rolloutがmission lateral offset後の曲率と6.0 m/s²上限を使う。
- inside offsetによる速度低下でrear-clear不能な候補をmission採用前に棄却する。
- 追加単体テスト、既存単体テスト、対象package buildが成功する。

