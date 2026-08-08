# Design

## 1. Follow deliberate-stop判定

stuck recoveryへ渡す`deliberate_stop`を、behavior名ではなく実際の縦制御所有権から決める。

```text
follow_is_deliberate =
  Follow && has_front_vehicle && (
    follow_speed_limit_active ||
    moving_front_clearance_limit_active ||
    finite target/desired velocity below path forward demand)
```

SafetyBrakeとLowSpeedAvoidanceは従来どおり常にdeliberate stopとする。これにより、前車が
逸走して`Follow`表示がstate holdで残っていても、`limit=inf`ならsolver fallback/no-progressの
観測を継続し、既存の2秒wall evidenceまたは3秒evidence-free Recoveryへ早く渡せる。

## 2. Rearward-progress time grace

SafeSeparationの絶対距離上限は維持する。絶対／局所時間上限へ到達した場合のみ、次の全条件で
時間Abortを抑制する。

- 設定が有効
- forward escapeがactive
- locked targetがego後方
- current short horizonがsafe
- 現在の局所枠で前方進捗がfresh
- 絶対距離上限未到達

この間は同じpass sideと通常コース速度参照を維持する。fresh progressが失われれば次周期で
従来の時間Abortへ戻るため、衝突ペナルティや低加速の最中に進んでいる車だけを救済し、
完全停止を無期限に延命しない。

## 3. 設定と観測性

- `v2x_overtake_safe_separation_rearward_progress_time_grace_enabled: true`
- SafeSeparation理由へ`rearward progress time grace`を追加する。
- 既存の`Stuck detector`ログでは、非制限Follow時に`reject=deliberate_stop`ではなく
  solver fallback継続時間が増えることを確認する。

## 4. 期待する動的効果

- `limit=inf / follow_cap=0`で停止した場合、前車がfront集合から消えるまで待たずRecoveryへ入る。
- targetが後方へ移ったPassで`local time limit`だけを理由にRecoveryへ落ちない。
- 前方障害に対するFollow/SafetyBrakeと絶対走行距離のhard boundは変化しない。
