# Requirements

## Goal

20260822-045834 の走行で確認した、追い越し／Dynamic Escape の実行契約不整合を局所修正する。

## Defects in scope

1. wall-safe replacement を採用した直後、同じ Dynamic Escape attempt が次周期に新規 exit として再入場する。
2. ShiftOut 開始周期の最終 physical wall 判定が失敗した際、まだ当該 Mission の指令を一度も publish していないのに DynamicMissionWait へ進む。
3. 0.4 m 境界で数 mm の離散化差があるだけの経路と、0.1～0.3 m 級の本当の余裕不足を同じ理由で棄却している。
4. ログだけでは replacement の attempt 継続、公開済み指令、境界許容の適用有無を識別しにくい。

## Constraints

- ROS topic/service/message、Domain、提出物の契約は変更しない。
- 物理接触、out-of-map、明確な壁余裕不足は許容しない。
- Dynamic Escape の retained solution は既存の短時間 lease を超えて延命しない。
- `aichallenge/result-summary.json` の既存ユーザー変更には触れない。
