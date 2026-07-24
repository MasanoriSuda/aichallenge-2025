# 追い越し残事象修正 設計

## 調査結果

| 出力 | 主な観測 | 判定 |
|---|---|---|
| `20260724-070818` | hold中70/70周期が`acceleration=-1.35 m/s²`。command 6.424→3.661 m/s、実速度6.470→3.634 m/s | bounded corridor holdと2.0 m/s no-gap hard limitの所有権不整合 |
| `20260724-073134` | 最長holdで最大減速度は8/40周期。command 6.477→6.466 m/s、実速度6.550→6.245 m/s | no-gap重複制限の除去で2.0 m/sへの崩落を解消 |
| `20260724-083221` | 7.191 m/sでraw steering -0.445 rad、最終出力-0.667 rad。壁接触後、約1.003 sで7.191→0.999 m/s | 停止車誤判定、横計画競合、direct操舵ラッチによる物理衝突が急失速の主因 |
| `20260724-085142` | 停止車確認は最大1/3。LowSpeed direct、wall/contact/stuckなし。lap 1は70.566 s | 3 distinct valid sample確認で既知の誤起動連鎖を遮断 |

### no-gap急減速

live execution corridor欠落は既存の最大2秒holdにより追い越し継続可能と判定されていた。
一方、後段のno-gap速度制限はholdの成立を参照せず、plannerの2.0 m/s hard limitを
残していた。corridorがvalidへ戻った直後に指令が加速へ反転することから、
この減速はsolver failureや横加速度制限ではなく、縦速度制限の競合である。

### target loss誤分類

自車失速後に対象との距離が増え、対象が24 mのbounded探索窓を出る時刻と投影失敗が
一致した。現行判定は進捗連続性制約付き投影の失敗を一律に
`course progress discontinuity`としていたため、通常の探索範囲外lossまで誤分類した。
Recovery自体は、rear-clear未確認の対象を見失った場合の正しい安全動作である。

### `083221`の急失速因果列

1. `d2`は実際には約5.288→5.497 m/sで走行していた。
2. `d1`側trackerは初回または同時刻観測で速度差分を計算できず、初期値0 m/sを
   停止車観測として扱った。
3. LowSpeedAvoidanceが右側targetでdirect controlを開始した。
4. behaviorがFollowへ戻った後、OvertakeLineは左側ShiftOutを開始したが、
   以前のdirect controlが出力経路に残った。
5. direct controlは「low-speed」の固定phase速度を前提にしており、実速度約7.19 m/s
   に対する横加速度制限がなかった。大操舵で壁へ接触し、制動設定値を大きく超える
   約-6.17 m/s²の物理的な失速となった。

したがって、5 m Followの不足や通常の`a_min=-1.35 m/s²`だけでは、
この急失速量を説明できない。主因は壁接触である。

## 修正設計

### 1. bounded corridor holdとno-gap制限の整合

`GapPlannerNoGapVelocityLimitRequest`へ
`transient_execution_corridor_hold`を渡す。no-gap制限を抑止するのは、
横離隔済みcommitted Pass、またはhard failureを除外した既存pure helperが認定した
bounded holdだけとする。期限切れ、禁止区間、position jump、cooldown、
EmergencyBrakeでは従来のfail closed動作へ戻る。

### 2. 診断専用のcommon-course再投影

進捗連続性制約付き投影が失敗したlocked targetだけ、同じlookahead、
cross-track、方向条件から進捗連続性制約だけを外して再投影する。

- 制約なし投影が成功: 真のcourse progress discontinuity
- 制約なし投影も失敗: lookahead外などの通常target loss

制約なし結果は診断専用であり、追跡値やfront判定には採用しない。

### 3. Passのfail-closed化

- front cap解除には履歴latchではなく現在の物理的横離隔を使い、必要離隔を1.50 mとする。
- committed Passは、対象前方距離が0.5 m改善するたびに走行距離budgetを更新する。
  改善なしに32 m進んだ場合はRecoveryへ移行する。
- raw odometryによる実車体footprintを静的壁gridに照合する。ShiftOut/Pass中に
  検証不能、map外または接触ならRecoveryへ移行する。
- 静的壁clamp後にも必要横加速度を再計算し、上限を超えるtargetはRecoveryとする。
- Recoveryの横targetにも同じ静的壁horizonを適用する。

### 4. 停止車候補の時系列確認

`TrackedVehicle`へ速度観測の有効性を持たせ、以前の異なる時刻の観測から
速度を計算できた場合だけ停止車候補にする。

候補は同一IDのdistinctな受信時刻を3回連続確認して確定する。同一受信sampleを
複数制御周期で読む場合はcountを進めない。候補消失、ID変更、無効時刻、逆行時刻、
専用maximum gap超過ではリセットする。confirmationの最大間隔はstall watchdogの
観測間隔から分離し、低頻度V2Xにも対応可能な独立parameterとする。

### 5. 横計画の単一所有者化

LowSpeed direct controlがactiveまたはhandoff中は、OvertakeLineが横targetを
生成しない。direct開始時にpass側targetを一度だけlatchし、作動中の別候補で
反対側へ上書きしない。direct終了・外部Recovery・session resetでは関連stateを
一括resetする。

### 6. LowSpeed direct controlの安全上限

- measured actual speed、wheelbase、`max_lateral_accel`、最終steering gainから
  許容controller steeringを計算する。
- unconstrained target、direct内rate-limit後、node側の外部filter/rate-limit後の
  publish値を同じ上限でclipする。
- phase速度は`v_max`だけでなく、behaviorの有限な`desired_velocity`と
  `target_velocity_limit`の最小値に制限する。
- actual poseの物理footprintとclearance footprintを静的壁gridで検証する。
  検証不能、map外、接触、clearance違反では速度0、操舵0へfail closedし、
  legacy Boostを無効化して加速度low-passなしの`a_min`を適用する。
  危険なMPC/OvertakeLineへのfallbackは行わない。
- runtimeのsteering gain更新はMPC内部の上限計算にも同時反映する。

## 効果確認設計

1. pure helperでhold/no-gap、投影分類、停止車distinct sample、所有権、
   actual-speed操舵cap、Pass進捗watchdog、静的壁abort条件を単体テストする。
2. Docker環境で対象package testとbuildを実行する。
3. 同一dev2で既知の誤起動箇所まで走行し、LowSpeed confirmation count、
   direct entry、OvertakeLine phase、steering、wall/contact/stuckを比較する。
4. 正常な停止車が存在するシナリオでは、3観測後のdirect開始、速度・操舵cap、
   wall stopを別途動的確認する。

## インターフェース影響

変更は参加者package内部の判定、parameter、診断ログに限定する。
`/control/command/control_cmd`の型・役割、V2X topic、Domain、launch、
result JSON、`output/latest`契約は変更しない。
