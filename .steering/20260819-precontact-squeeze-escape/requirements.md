# Requirements

## 目的

Pass中、locked targetとの将来footprint重複が連続確定しているにもかかわらず、
attack holdとfront danger suppressionが前進を維持し、横から挟まれて接触する事象を防ぐ。

## 変更範囲

- confirmed predicted overlapを接触前squeezeとして分類する純粋ロジック
- 同じpass side内でのwall-bounded lateral escape
- escape中のfront-cap再適用とfront danger suppression解除
- 遷移単位の診断ログと既存debugログの項目追加
- core単体テスト、config、起動時設定ログ

## 制約

- actual contact後のContactContinuationは維持する
- 一周期だけの予測重複には反応せず、既存confirmation時間を共有する
- sideを横断しない。no-return契約を維持する
- 壁余裕がなければ横移動をclampし、速度制限をfallbackとする
- ROS 2 topic/service、評価schema、提出インターフェースは変更しない
- `aichallenge/result-summary.json`の既存変更は対象外とする

## Definition of Done

- squeeze active時にattack holdが解除され、front capが再適用される
- squeeze active時にlocked target向けfront danger suppressionが解除される
- wall bounds内でpass side方向へ追加の分離biasが適用される
- entered/endedとrequested/applied bias、wall limit、fallback理由をログで判別できる
- package build/testが成功する
