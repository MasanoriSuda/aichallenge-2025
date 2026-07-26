# Requirements

## 目的

レース中の事故・接触後、後方に退避余地がなくなったにもかかわらず
Reverse-only状態が残り、前進退避を評価せずSafeStopを繰り返す現象を解消する。

## 確認事象

`output/20260726-093622/d3/autoware.log` では、接触後のRecoveryが
`static=invalid_grid` となり、約487秒間に673回のaggressive retryを
繰り返したが、`rejoin_complete`へ到達しなかった。

同時刻のrosbagから自己位置を地図座標へ照合すると、車体は地図上端から
約6.8 m内側にあり、実際の地図外ではなかった。`checked=0`であることから、
`invalid_grid`は候補未評価時に残る診断値の初期値だった。

## 要求

1. 後壁が明示的に検出された場合、過去のsolver fallback由来のReverse固定より
   前進退避を優先できること。
2. 前進退避候補は、既存の静的rolloutとV2X corridorで安全確認すること。
3. 候補未評価と、実際の静的地図不正をログ上で区別できること。
4. Boost、ギア確認、移動距離、速度上限など既存のRecovery安全ゲートを維持すること。
5. simulation-only制約を維持し、実車Recoveryを有効化しないこと。
6. ROS topic/service/message、評価JSONの契約を変更しないこと。

## 対象外

- 実走による効果確認
- occupancy gridの範囲・セル値・地図境界の変更
- 通常のMPC走行経路や追い越し判定の緩和
- 静的地図もV2Xも利用できない状態での盲目的な発進
