# Design

## 観測した失敗

最新走行では SafeSeparation が tactical alternate reselection を許可し、fresh alternate Mission も保持していたが、`try_last_feasible_maneuver` が履歴上の no-return latch を再評価して `BlockedByNoReturn` とした。その後は Recovery へ遷移した。

## 方針

### 1. Tactical no-return re-arm

通常の no-return latch は単調のまま維持する。例外として、SafeSeparation の専用判定が次を全て確認した場合だけ、alternate 再選択要求に `tactical_no_return_rearmed` を付ける。

- target identity/progress が連続
- target が設定距離以上前方
- 現在車体が非重複
- footprint prediction が有効かつ sweep 非重複
- corridor 非閉塞
- wall / emergency / solver hard fault なし
- rear-clear 前
- cross-side replacement 未使用

このフラグは候補選択と cross-side commit の両方へ明示的に渡す。SafeSeparation 中の一般的な side change は引き続き拒否する。

### 2. Speed-preserving soft disengagement

SafeSeparation の local time/distance または short-horizon 軟失敗で、次善 Mission も dynamic wait も成立しない場合、次を満たせば Recovery を経由せず OvertakeLine を Idle に戻す。

- hard fault なし
- target 連続
- target が設定距離以上前方
- 現在車体と予測 sweep が非重複
- corridor 非閉塞
- rear-clear ではない

失敗した side には cooldown を設定する。これにより通常 Follow の速度仲裁へ戻りつつ、反対側は次周期以降の新規候補として評価できる。

## 安全境界

- 実 overlap、予測不成立、corridor block、壁接触、EmergencyBrake、solver recovery は Recovery のまま。
- tactical re-arm 後も完全 preflight、rear-clear rollout、時間・距離 budget、最低速度、壁余裕を再確認する。
- Mission 中の cross-side replacement は最大1回という既存制限を維持する。

## 効果確認

- `SafeSeparation tactical alternate reselect` が `BlockedByNoReturn` ではなく accepted になること。
- accepted 後に `ShiftOut` へ戻り、別側で Pass を継続すること。
- alternate 不成立かつ軟失敗・物理 clear 時は `Recovery` ではなく `Idle` へ戻ること。
- 接触、壁詰まり、EmergencyBrake では従来どおり Recovery へ入ること。

