# AWSIM Boost Motion Trigger Tasklist

作成日: 2026-07-16
更新日: 2026-07-16
状態: Implemented / dev3 verified

## Definition of Done

- BoostがReady静止中やGrounded中に発動しない。
- d1〜d3で実前進開始から0.25秒以内にpulseが1回だけ出る。
- 遅延Startを待たず、走行途中で遅れてBoostしない。
- SafetyBrake、solver fallback、forced stop、Reverse/recovery中に発動しない。
- status異常・stale・残数なし・Boost中では発動しない。
- timeout後、確認失敗後、duplicate stateで再送しない。
- StartEnteredの既存利用者とROS interface契約を変更しない。
- build、unit test、dev3 runtimeが成功する。性能A/Bはfollow-upとして分離する。
- 仕様書と実装が一致する。

## 0. Baseline

- [x] 現行guardが `start_seen_` をpulse必須条件にしていることを確認する。
- [x] pulse publishが正常command publish後に呼ばれることを確認する。
- [x] run `output/20260716-083853` のReady、motion、Start、pulse時刻を比較する。
- [x] 現行pulseが発車からd1=7.93秒、d2=6.95秒、d3=6.23秒遅いことを確認する。
- [x] 現行pulse high/low、remaining減少、確認処理自体は成功していることを確認する。

## 1. Specification and compatibility

- [x] `trigger` enumとconfig未指定時の互換既定値を確定する。
- [x] motion threshold、max trigger speed、timeoutの初期値を確定する。
- [x] Ready欠落時のStart fallback条件を確定する。
- [x] V2X SafetyBrakeとMPC solver fallbackをBoost inhibitへ明示的に含める。
- [x] `StartEntered` eventの既存利用箇所を列挙し、意味を変更しないことを確認する。
- [x] topic/type/Domain/submit/result契約に変更がないことを確認する。

## 2. Guard state machine

- [x] `Trigger::{AwsimStart, FirstForwardMotion}` を追加する。
- [x] Readyで `AwaitingMotion` へ進むがpulseしない状態を追加する。
- [x] signed forward speedのmotion edgeを1回だけlatchする。
- [x] motion検出時刻をsteady clockで保持する。
- [x] trigger timeoutとmax trigger speed超過で `LaunchExpiredSpent` へ進める。
- [x] duplicate Ready / Startでmotion epochやtimeoutを延長しない。
- [x] low-speed Start fallbackを実装する。
- [x] Finish -> Spawnedだけ次sessionへrearmする既存契約を維持する。
- [x] confirmation/no-retry状態を退行させない。
- [x] NaN / Inf / clock rollbackをfail-closedにする。

## 3. MPC node integration

- [x] odometryからsigned forward speedを取得してguardへ渡す。
- [x] normal control command publish成功後にBoost評価する順序を維持する。
- [x] enable_control、forced stop、operator stopをcontextへ渡す。
- [x] `mpc_fallback_active` をBoost inhibitへ含める。
- [x] V2X最終stateがSafetyBrakeならBoostをinhibitする。
- [x] stuck recovery / Reverse / recovery inhibitをcontextへ含める。
- [x] Ready stateをBoost guardへ渡すがStartEnteredを生成しない。
- [x] start-grid grace、domain speed window、manual reset分岐を変更しない。

## 4. Config and validation

- [x] `trigger: first_forward_motion` をconfigへ追加する。
- [x] `motion_speed_threshold_mps` を追加する。
- [x] `max_trigger_speed_mps` を追加する。
- [x] `motion_trigger_timeout_sec` を追加する。
- [x] unknown trigger、negative/nonfinite、threshold逆転を起動時に拒否する。
- [x] `trigger: awsim_start` rollbackを維持する。
- [x] `enabled: false` とdomain override無効化を維持する。

## 5. Diagnostics

- [x] Ready motion-watch準備を状態変化時に記録する。
- [x] motion速度と同一clockのReady/motion時刻を記録する。
- [x] motionからpulseまでの遅延を記録する。
- [x] timeout/max speedによる見送りを1回だけ記録する。
- [x] SafetyBrake/fallback/recovery/statusによるblock理由を区別する。
- [x] 40 Hzの同一ログ連打がないことを確認する。

## 6. Unit tests

- [x] Ready前はpulseなし。
- [x] Ready静止中はpulseなし。
- [x] threshold未満はpulseなし。
- [x] threshold到達の正常周期で1回だけpulse。
- [x] motion後の短いstatus遅延はwindow内でpulse。
- [x] timeout後はpulseなし・再送なし。
- [x] max trigger speed超過後はpulseなし。
- [x] Ready欠落 + low-speed Start fallback。
- [x] 高速の遅延Startではpulseなし。
- [x] SafetyBrake中はpulseなし。
- [x] solver fallback / forced stop中はpulseなし。
- [x] Reverse/recovery中はpulseなし。
- [x] stale/invalid status、remaining=0、isBoosting=trueはpulseなし。
- [x] duplicate Ready/Startでwindowを延長しない。
- [x] confirmation/no-retry testを維持する。
- [x] Finish -> Spawned rearm testを維持する。
- [x] `awsim_start` rollback trigger test。

## 7. Build and static verification

- [x] `make autoware-build`
- [ ] `colcon test --packages-select multi_purpose_mpc_ros`
- [x] `test_awsim_boost_start_dash` 全24件成功。
- [x] `git diff --check` 成功（legacyファイル全体のclang-format適用は差分肥大化を避けて未実施）。
- [x] `/awsim/cmd` high/low pulseコードに意図しない差分がないことを確認する。
- [x] `/control/command/control_cmd` のpublisher責務に差分がないことを確認する。

## 8. dev3 runtime verification

- [x] `make down` 後に `make dev3` をクリーン起動する。
- [x] d1〜d3のReady時刻を記録する。
- [x] d1〜d3の初回signed speed >= 0.1 m/s時刻を記録する。
- [x] d1〜d3のpulse high/low時刻と回数を記録する。
- [x] `pulse - first_motion <= 0.25 s` を各車で確認する。
- [x] Ready静止中にpulseがないことを確認する。
- [x] 車両別Startより前にpulseが出ることを確認する。
- [x] remainingが各車で1だけ減ることを確認する。
- [x] duplicate pulseがないことを確認する。
- [ ] initial SafetyBrake中にpulseがないことを確認する。
- [x] collision / wall contactログがないことを確認する。pre-Readyの停止中OSQP fallbackは全車で発車前に解消した。

## 9. A/B and safety regression（follow-up、実装完了のblocker外）

- [ ] 全Domain Boost無効runを同一条件で取得する。
- [ ] first-motionをepochとして0〜10秒の速度・加速度を比較する。
- [ ] 0.1 / 1 / 5 m/s到達時刻を比較する。
- [ ] 多車両車間とSafetyBrake回数を比較する。
- [ ] status staleを模擬し、timeout後の遅延pulseがないことを確認する。
- [ ] `trigger: awsim_start` rollback runを確認する。
- [ ] 単独 `make dev` でも1回だけ発動することを確認する。

## 10. Documentation and completion

- [x] `docs/spec/mpc-integration.md` のStart Dash説明をmotion triggerへ更新する。
- [x] 0.1 / 1.0 / 0.5がローカル暫定値であることを明記する。
- [x] interface文書が変更不要であることを確認する。
- [x] 実行コマンド、run ID、主要時刻をこのtasklistへ追記する。
- [x] requirements/design/tasklistを実装結果に合わせて更新する。

## Rollback check

- [x] `awsim_boost.trigger: awsim_start` で旧タイミングへ戻せる（unit test）。
- [x] `awsim_boost.enabled: false` で公式Boostを完全無効化できる（既存unit test）。
- [x] rollbackにtrajectory、domain速度、legacy Boost有効化を混ぜない。

## 実行結果

検証run: `output/20260716-090830`

| Domain | Ready [s] | first motion / speed | pulse [s] | motion→pulse | pulse→Start | count / confirmed |
|---|---:|---:|---:|---:|---:|---:|
| d1 | 1784160535.5916 | 1784160535.9528 / 0.105 m/s | 1784160535.9529 | 0.0001 s | 6.2136 s前 | 1 / remaining=1 |
| d2 | 1784160535.5815 | 1784160535.9543 / 0.123 m/s | 1784160535.9543 | 0.0001 s | 5.3555 s前 | 1 / remaining=1 |
| d3 | 1784160535.5778 | 1784160535.8918 / 0.108 m/s | 1784160535.8919 | 0.0001 s | 4.8520 s前 | 1 / remaining=1 |

- 3台のpulse時刻差は約0.062秒。
- Ready静止中のpulseなし。全車でmotion検出と同一control cycleにpulse送信。
- 後着の`Start`後もpulse countは各車1のままで、遅延再送なし。
- `make autoware-build`: 25 packages成功。
- `test_awsim_boost_start_dash`: 24/24成功。
- runにinitial SafetyBrakeログはなく、そのruntimeケースはunit testで確認した。

## 作業上の注意

- 既存 `.steering/20260711-awsim-boost-start-dash/` は初回実装の履歴として変更しない。
- `output/`、`aichallenge/result-summary.json`、trajectory CSVは編集対象にしない。
- 実装は `aichallenge_submit/multi_purpose_mpc_ros` に閉じる。
