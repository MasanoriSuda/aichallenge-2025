# 要件

## 目的

OvertakeLine が検証済み Pass を実行している最中に、Behavior の新規 Entry 条件が再適用されて Follow へ落ちる現象を止める。

## 対象事象

`output/20260802-122031/d1/autoware.log` では Pass 中にも以下の理由で `Overtake -> Follow` が発生していた。

- mission candidate search rejected
- continuation front distance
- hard curve blocked
- guard lateral acceleration
- gap unavailable / retry cooldown

これらは新規候補の採否には必要だが、固定済みPassの速度・状態所有権を解除する条件としては重複している。壁・live corridor・solverはOvertakeLineの実行時判定が別途監視している。

## 維持するHard abort

- locked target消失・position jump・course progress不連続
- 実車体footprint重複
- locked targetのpass側侵入
- 明示的な追い越し禁止waypoint
- 抑制されていないEmergency front risk
- solver recovery要求
- OvertakeLine側の壁・live corridor・横加速度・solver判定

## 完了条件

- 検証済みPass中はEntry候補の棄却だけでFollowへ落ちない。
- Hard abortは従来どおり有効。
- 既存ログ周期を増やさず、owner適用の有無を確認できる。
- build/testが成功する。
