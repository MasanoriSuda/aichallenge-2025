# Requirements

## 目的

横並びまでコミットした追い越しで、対象車が既に自車後方へ移り、自車が実測で前進しているにもかかわらず、予測 footprint の一時的な重複だけで `SafeSeparation` を中断しない。

## 対象事象

- `output/20260810-201802/d1/autoware.log`
- `target_s=-1.37 m`
- SafeSeparation 内の前進量 `3.32 m`
- 最終進捗から `0.15 s`
- 現在車体は非重複、execution corridor も非block
- 予測 sweep の不成立だけで `Pass -> Recovery`

## 制約

- 実車体の危険、壁接触・壁観測欠損、pass-side intrusion、EmergencyBrake、solver recoveryは緩和しない。
- 対象ID、pass side、mission generationを維持する。
- 実測進捗が古くなった場合は従来どおり中断する。
- 絶対Pass時間・距離上限は変更しない。

