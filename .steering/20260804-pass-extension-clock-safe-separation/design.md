# Design

## 1. Pass extension clock normalization

制御周期へ渡される`now_sec`はROS時刻で、`live_prediction_timing.expiry_sec`も同じ基準で
計算される。一方、延長計画の計算時間計測には単調増加する`steady_clock`を使う。

延長計画開始時に次を保存する。

```text
planner_ros_sec = now_sec
planner_steady = steady_clock::now()
```

commit直前は、steady clockの経過時間だけをROS時刻へ加算する。

```text
planning_elapsed = steady_now - planner_steady
commit_ros_sec = planner_ros_sec + planning_elapsed
```

これにより、時刻ジャンプの影響を受けずに計算時間を測りつつ、commit時刻と予測期限を
同じROS時刻基準で比較できる。`planner_result_max_age`の0.10秒上限も維持する。

## 2. SafeSeparation front-clear confirmation

SafeSeparation中に、`target_longitudinal >= front_clear_distance`が初めて成立した時刻を保存する。
不成立へ戻った場合は保存時刻をリセットする。

pure resolverへ次を渡す。

- `front_clear_elapsed_sec`
- `front_clear_confirm_sec`

遷移は次とする。

```text
rear-clear confirmed and Return corridor available -> Return
short horizon unsafe                             -> Abort
front clear continuously for confirm_sec         -> RecoverBehind
time/distance bound reached                      -> Abort
otherwise                                        -> KeepSameSide
```

既定確認時間は0.25秒とする。これは20 Hz V2Xで約5観測に相当し、単周期チャタリングを
除去しながら3秒/8 mの安全上限より十分短い。

## 3. Configuration and logging

追加設定:

```yaml
v2x_overtake_safe_separation_front_clear_confirm_sec: 0.25
```

起動ログへ確認時間を表示する。毎周期ログは追加せず、既存の状態遷移ログを使う。

## 4. Compatibility

- 変更対象は`multi_purpose_mpc_ros`内部と設定・単体テストのみ。
- topic/service/message型、launch entry、Domain、result JSONは変更しない。
- 設定未記載時も既定0.25秒で動作する。
