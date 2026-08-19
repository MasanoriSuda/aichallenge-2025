# Requirements

## Background

`output/20260819-223132/d1/autoware.log` では、Follow 中の動的障害物回避で
GapPlanner と reachable bridge が成立しているにもかかわらず、要求横移動が
`v2x_dynamic_obstacle_lateral_escape_min_shift` の 0.10 m 未満という理由だけで
authority が 34 回棄却されていた。棄却後は検証済み planner 出力も消去されるため、
走行ラインが既に安全回廊内にある場面でも Follow 速度制限へ戻る。

同じ走行では Mission の実差し替えが 2 回発生した一方、runtime-failover trace は
全 69 行が `action=hold` だった。差し替え処理が wait resolver より前に実行される
経路を trace が覆っていない。また非同期評価の自由文 reason が交互に変わり、
同じ gate 状態を過剰に再出力していた。

## Requirements

- 物理回廊と到達可能性 bridge が検証済みの場合に限り、0.10 m 未満の同側横移動、
  または横移動不要な pass-through を lateral authority として許可する。
- EmergencyBrake、solver recovery、planner infeasible、bounds 非所有、逆方向shiftの
  hard gate は従来どおり fail closed とする。
- pass-through を採用しても、有効な tracking solve が1回成立するまでは通常Followの
  速度制限を解除しない。
- runtime-failover trace に、実際の Mission 差し替え結果と差し替え元を残す。
- 自由文 reason の変動だけでは trace を再出力せず、固定 gate・action・source の
  変化と低頻度 heartbeat で記録する。
- ROS 2 topic/service/message、評価成果物schema、既存設定値は変更しない。
- ユーザー所有の `aichallenge/result-summary.json` は変更・コミットしない。

## Definition of Done

- pass-through の許可・不許可と hard gate 優先を単体テストで固定する。
- runtime trace の gate分類、reason-only変化の抑制、action/source変化の即時出力を
  単体テストで固定する。
- `multi_purpose_mpc_ros` が build/test を通る。
- 次回試走で `accepted-pass-through`、`follow_cap_suppressed`、
  `runtime-failover action/source` をログから集計できる。
