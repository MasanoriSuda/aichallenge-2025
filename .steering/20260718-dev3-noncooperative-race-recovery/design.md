# Design

## Scope

変更対象は`multi_purpose_mpc_ros`のスタック復旧FSM、ROS/AWSIMアダプタ設定、単体テスト、
`docs/spec/mpc-integration.md`とする。実車ノードや評価基盤は変更しない。

## Configuration

`stuck_recovery.maneuver`へ以下を追加する。

- `aggressive_sim_recovery_enabled`: シミュレーション予選向け永久停止解除。
- `aggressive_retry_delay_sec`: 回復可能な`SAFE_STOP`から再評価するまでの停止時間。

設定読み込み時に、積極復旧と`simulation_only: false`の組み合わせを拒否する。

初回実走で短い切り返しだけを反復してrejoinへ入れないことが分かったため、
`stuck_recovery.rejoin.aggressive_force_after_retries`も追加する。指定回数以上失敗し、現在footprintが
非接触でfeedback steeringが有効な場合に限り、短いforward sweepの衝突予測をsimulation-onlyで
緩和して`LOW_SPEED_REJOIN`へ進む。0は無効とする。

Reverseは運営`main`のteleop入力に合わせ、以下をA/B実験値とする。

- 駆動: `reverse_acceleration_sign: -1.0`
- 停止: `reverse_stop_acceleration_mps2: +0.8`

## State machine

### Solver-independent rejoin

積極復旧中の`LOW_SPEED_REJOIN`は通常MPCのsolver healthを必要としない。この状態の制御出力は
既にrejoin feedback、静的forward sweep、低速制限を用いるため、通常MPCの最適解を直接使用しない。

### Recoverable terminal retry

以下の種類をシミュレーション競争上の回復可能停止として扱う。

- clearance/候補/試行上限
- gear reportの一時欠損・不一致
- 接触悪化または改善不足
- forward/reverse maneuver limit
- rejoin timeout/path blocked/unsafe
- solver unsafe

待機時間後にattempt/step budgetをリセットして`STOP_AND_CONFIRM`へ戻す。ROSアダプタも
episode距離、候補固定、接触基準をリセットし、同じ失敗済み操作の即時再利用を防ぐ。

次はハード停止のままとする。

- invalid/non-finite input
- non-monotonic time
- odometry unsafe
- control interrupted

## Compatibility

- 既定値は積極復旧無効とし、純粋coreの従来fail-closed挙動を維持する。
- checked-in dev3設定だけ積極復旧を有効化する。
- V2X topic契約や他車からの応答は追加しない。
