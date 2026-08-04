# 設計

## 1. 車体間余裕

`v2x_vehicle_radius=1.45`は2台の半幅合計であり、車体寸法として正しいため変更しない。
代わりに以下を変更する。

- minimum target separation: 1.50 -> 1.55 m
- front-cap release: 1.50 -> 1.55 m
- front-cap reapply: 1.45 -> 1.50 m
- current overlap confirm: 0.10 -> 0.05 s

## 2. clearance-buffered minimum motion

動的・静的に検証済みのpass corridor `[lower, upper]`に対して、車両側境界から
選択side方向へ最大`preferred_clearance_buffer`だけ内側へ縮めた推奨区間を作る。

- left pass: `[lower + buffer, upper]`
- right pass: `[lower, upper - buffer]`
- `buffer <= corridor_width / 2`

この推奨区間へ基準レーシングラインをclampする。広いcorridorでは追加余裕を使い、
狭いcorridorでは中央までに制限して壁側余裕を消費し切らない。最終的な
target separation、wall、横加速度、full mission preflightは既存hard guardで確認する。

## 3. transient predicted overlap

既存の0.25秒confirmation clockをPass continuation preflightでも共有する。
予測sweepが一度重なっただけではcenter-separation/outer-role制約を復活させず、
confirmed overlapで初めてsame-side lateral replanを要求する。

## 4. side-by-side forward escape

SafeSeparationへ入った後でも、次を全て満たすときはforward escapeを有効にする。

- committed Pass attack modeが有効
- minimum-motion corridorを保持
- front cap解除済み
- current footprintが非重複
- target観測が連続しposition jumpなし
- target longitudinalが0.75 m以下（ほぼ横並びから後方）
- current short horizonがhard-safe

速度参照は`max(current ego speed, target speed + separation delta)`とし、v_maxで上限を
取る。これによりSafeSeparationへ遷移した瞬間の減速を防ぐ。上記条件を外れた場合は
従来どおりtarget aheadならRecoverBehind方向の速度参照を使う。

## 影響範囲

- `v2x_overtake_core`: minimum-motion goal、Pass continuation policy、SafeSeparation policy
- `mpc_controller_cpp`: config読込、policy入力、起動/debugログ
- `config.yaml`: シミュレーション用A/B既定値
- ROSインターフェース、評価基盤、trajectory/localizationは変更しない。

