# Design

## 現行不整合

cross-side replacementの候補rolloutは、曲率による一時的な速度低下とrear-clear時の終端速度を別々に予測している。一方、Admissionは前車速度を全horizonの最低速度に設定していた。

このため、既に自車が前車より遅いヘアピンでは、候補が現在速度を維持し、終端では十分なclosing speedを回復しても、一時最低速度だけで棄却される。

## 変更

Admission要求を次の二条件へ分離したまま、horizon最低速度だけを現在状態に合わせる。

```text
minimum_horizon_speed = min(current_ego_speed, target_speed)
minimum_rear_clear_speed = target_speed + terminal_closing_speed
```

これにより、候補は現在より余計に失速しないことと、rear-clear時に前車より速いことの両方を要求される。

## 継続処理

現行には以下が既に存在するため、新規`hold_alongside`状態は追加しない。

- cross-side replacement棄却時の旧Mission保持
- committed forward completion
- SafeSeparationによるrear-clearまでの前進所有
- last-feasible current/alternate Missionの再利用
- soft failure時のdynamic Mission wait

## 安全境界

- hard wall/contact/solver faultは従来どおり継続を許可しない。
- 絶対時間・距離budgetは緩和しない。
- rear-clear終端速度は緩和しない。
- no-return後の無条件な左右切替は許可しない。

## 動的確認

- `minimum_speed_insufficient`件数
- `required_min_v`と`ego_v`、`predicted min_v`
- `opponent side PassPlan replaced`の発生
- `Pass -> Return`完遂数
- wall/solver Recoveryと接触の増減

