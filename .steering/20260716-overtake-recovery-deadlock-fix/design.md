# Overtake Recovery Deadlock Fix Design

作成日: 2026-07-16
更新日: 2026-07-16
状態: Complete

## 原因

solver failure Recoveryでは速度上限を`min(configured_limit, current_speed)`としていた。この上限は減速に追従して単調に低下し、0 m/sで再加速不能になる。

同時にphase距離を`current_speed * phase_elapsed`で近似しており、停止中は完了できない一方、長時間後の微小速度だけで突然距離条件を満たし得る。

## 設計方針

### 1. Pure recovery policy

ROS非依存の`v2x_overtake_core`へ次を追加する。

```cpp
enum class RecoveryExitReason
{
  Active,
  DistanceComplete,
  LateralComplete,
  Stalled,
  TimedOut,
  InvalidObservation,
};

struct RecoveryPolicyRequest
{
  double configured_velocity_limit_mps;
  double elapsed_sec;
  double traveled_distance_m;
  double target_distance_m;
  double lateral_error_m;
  double lateral_completion_m;
  double stalled_sec;
  double stall_timeout_sec;
  double timeout_sec;
};
```

policyは現在速度をvelocity limitへ使用せず、設定上限と終了理由だけを返す。

距離積算もpure helperにする。

```cpp
distance += max(0.0, speed_mps) * dt;
```

非finite、負dt、最大観測gap超過では距離を増やさない。

solver cooldownの期限更新とactive境界判定もpure helperにし、既存期限を短縮せず、期限ちょうどで解除する。

### 2. OvertakeLine state

`OvertakeLineState`に次を追加する。

- `phase_traveled_m`
- `phase_last_update_sec`
- `recovery_stall_since_sec`

phase遷移でprogress stateを初期化する。Recovery中は毎周期progressとstall連続時間を更新し、pure policyの結果で終了する。solver failure起因かどうかは既存の`overtake_solver_recovery_active_`で保持する。

### 3. Recovery termination

優先順位:

```text
SafetyBrake / EmergencyBrake
  > invalid observation
  > lateral complete
  > distance complete
  > stall timeout
  > total timeout
  > active recovery
```

SafetyBrake/EmergencyBrakeは既存の先頭分岐でline stateをresetし、V2Xが0速度を所有する。

stall/timeoutはline stateだけをIdleへ戻す。通常trajectoryは既にbase lineを0 m横偏差へ戻すため、Recoveryの目標と方向が一致し、急な逆方向目標を作らない。

### 4. Solver cooldown

MPCに`overtake_solver_cooldown_until_sec_`を保持する。solver failure起因Recoveryが終了または安全状態でresetされた場合に`now + cooldown`を設定し、`behavior_overtake`のline開始条件へ加える。

cooldownは`Overtake`状態で生成される明示的line、gap planner、fallback lateral targetを抑止し、SafetyBrake、Follow、Cruise、通常control commandは変更しない。

### 5. Logging

既存OvertakeLine debugへ次を追加する。

```text
elapsed, traveled, stalled, velocity_limit, solver_cooldown
```

終了時は`reason=recovery stalled`、`recovery timeout`等を既存phase transition logへ出す。

## 変更対象

| ファイル | 変更 |
|---|---|
| `include/.../v2x_overtake_core.hpp` | pure距離積算・Recovery policy |
| `src/v2x_overtake_core.cpp` | policy実装とvalidation |
| `src/mpc_controller_cpp.cpp` | state統合、固定速度上限、stall/timeout/cooldown |
| `config/config.yaml` | 暫定設定追加 |
| `test/test_v2x_overtake_core.cpp` | 0速度、積分、timeout、安全優先test |
| `docs/spec/mpc-integration.md` | Recoveryのfail-open条件と設定を記録 |

ROS interfaceとpackage依存は変更しない。

## Runtime verification

baselineと同じ`make dev3`で次を確認する。

- solver failure Recoveryが発生した場合、最大5秒以内にIdleへ戻る。
- Recovery中に速度上限が0へ固定されない。
- P2の0 m/s継続が1秒stall判定を大幅に超えない。
- P1のSafetyBrakeはP2再発進後に解除される。
- collision、wall contact、odometry/control失効を増やさない。

### 実施結果

run: `output/20260716-224008`

- 発進後約128秒の1秒周期V2X debugで、P1/P2/P3はいずれも`ego <= 0.15 m/s`の連続sampleなし。
- P1 Recoveryは3.075秒で`distance complete`、P2 Recoveryは0.323秒、0.325秒、1.400秒で完了した。
- Recovery debugの速度上限は`v_limit=3.00`を維持した。
- AWSIM logにcollision、wall contact、fatal、exceptionなし。
- solver failure起因Recoveryはこのrunでは発生しなかったため、2秒cooldownはpure unit testで期限更新、期限非短縮、境界解除を確認した。
- 起動直後のodometry未受信failsafeは各domainで1回だけ発生し、走行中の再発はない。

package全体testは17 CTest中16成功。失敗した既存`PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`は、作業前から変更済みの`traj_mincurv.csv`が終端重複を持たない一方、testが1点削除を固定期待しているためで、本修正では軌道とfixtureを変更していない。対象`test_v2x_overtake_core`は22/22成功。
