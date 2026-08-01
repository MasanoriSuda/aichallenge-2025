# Design

## Forward escape

minimum-motion Passで次を満たす場合を`side-by-side escape`とする。

- target seen / no position jump / no actual wall contact
- current body footprints separated
- `target_s <= 0.5 * (ego_length + target_length)`
- Passの横目標へ到達済み

この状態は、targetが横から進路へ寄る予測だけを理由とした減速より、現在の非重複を
維持して前へ抜け切る方が縦方向重複時間を短くする。したがって予測sweepが重複でも
front-cap解除を取得・保持できる。

## Predicted-overlap confirmation

targetが車体縦範囲より前に残る場合は予測sweepを維持する。ただし、既に解除済みの
capを単一V2X観測で再適用せず、連続`0.25 s`の予測重複を要求する。

- raw overlap開始: confirmation timer開始
- raw clear / escape / phase離脱: timer reset
- confirmation未完了: 既存解除を保持
- confirmation完了: front-cap再適用
- cap未解除の初回判定: raw overlapを即座に拒否し、猶予で解除しない

## Diagnostics

front-cap遷移・周期ログへ以下を追加する。

- side-by-side escape active
- raw predicted overlap
- predicted-overlap confirmed
- confirmation elapsed / configured duration
- body longitudinal clearance

## Configuration

`v2x_overtake_pass_predicted_overlap_confirm_sec: 0.25`を追加する。0秒なら連続確認を
無効化して従来の即時再適用へ戻せる。
