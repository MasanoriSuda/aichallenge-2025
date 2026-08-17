# Design

## 直線での side 順位

現在位置がカーブとして分類されていない間は、18 m 先の `lookahead_inner_side` を minimum-motion の inner preference に使用しない。完全または Progressive Entry として実行可能な左右候補がそろった場合は、現在の target-to-wall 開き幅に 0.25 m 以上の差があれば広い側を優先する。

将来形状は無視しない。full-track transition、rear-clear、target/wall interaction、ShiftOut/Pass/Return preflight は引き続き hard admission として先に評価する。現在幅の比較対象になるのは、それらの admission を通過した候補だけである。

## Progressive Entry の elastic target clearance

初期の局所 ShiftOut/body-clear prefix では、ロバスト target center separation が壁境界に収まらない場合のみ、実寸 center separation で再評価する。適用範囲は次に限定する。

- 新規 Progressive Entry
- MPCC-lite shadow/receding prefix
- Progressive Entry の短距離 continuation preflight

完全な rear-clear/Return Mission は従来どおりロバスト離隔を要求する。したがって実寸境界まで緩和された経路は短い prefix としてのみ実行され、rolling replan が継続可否を更新する。

## 診断

候補探索集計へ以下を追加する。

- initial entry preflight の棄却回数
- 最初の棄却理由
- 実寸離隔へ段階的に緩和した候補数
- 左右それぞれの現在開き幅

40 Hz の追加単発ログは出さず、既存の抑制された V2X debug/reason に載せる。
