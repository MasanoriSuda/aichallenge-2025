# Design

## 観測した欠陥

`20260819-230226`では、Pass中にpredicted overlapがconfirm済みでも
`attack_hold=1`、`danger_suppress=1`が継続した。alternate sideは利用不能で、
no-return後だったため横方向の逃げを作らず、実車体中心距離が約1.464 mまで縮んだ。
車幅1.45 m同士の中心分離境界1.45 mにほぼ一致し、接触相当である。

## 方針

既存のattack modeを全面的に無効化せず、以下をすべて満たす場合だけ
`pre-contact squeeze response`を有効化する。

- minimum-motion Pass中
- front capを一度release済み
- locked targetが連続観測され、現在車体は非重複
- footprint予測が有効
- predicted sweepが非分離
- 既存confirmation時間を超えて重複が確定
- actual contactのContactContinuationではない

有効時の優先順位は次の通り。

1. frozen pass sideを維持したまま、targetから離れる方向へ小さな横biasを加える
2. biasを全horizonの壁境界でclampする
3. attack holdとfront danger suppressionを解除する
4. front capを再適用し、横余裕がない場合は速度側をfallbackとする

予測重複の確定後に現在車体が重なり始めた場合は、current-overlap確定の短い窓の間だけ
同じresponseを保持する。この引き渡しにより、ContactContinuationが所有する直前の1〜2周期で
旧来のcurrent-overlap attack graceへ戻り、全速前進が復活する穴を防ぐ。重複確定後はresponseを
終了し、ContactContinuationが成立しない場合は従来どおりhard fault側に倒す。

## ログ設計

状態遷移時だけ次のイベントを出す。

`OvertakeLine pre-contact squeeze response entered|ended`

項目はtarget、side、target_s、relative lateral、予測重複時間、requested/applied bias、
wall-limited、front-cap、danger suppression、actionとする。周期debugにも同じ状態とbiasを
含めるが、新しい毎周期ログは追加しない。

## 非対象

- 反対sideへの切り返し
- 相手予測モデルの変更
- actual contact後の押し出し戦略
- Recovery、Reverse、自己位置推定の変更
