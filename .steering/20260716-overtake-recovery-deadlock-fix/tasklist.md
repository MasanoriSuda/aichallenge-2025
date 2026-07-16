# Overtake Recovery Deadlock Fix Tasklist

作成日: 2026-07-16
更新日: 2026-07-16
状態: Complete

## Definition of Done

- [x] Recovery速度上限が現在速度に追従して0にならない。
- [x] phase距離を周期積算する。
- [x] stall 1秒、全体5秒のtimeoutを実装する。
- [x] solver failure後2秒のline cooldownを実装する。
- [x] SafetyBrake優先を維持する。
- [x] unit test、build、dev3 runtimeが成功する。
- [x] P1/P2の約48秒停止が再発しない。

## 1. Baseline

- [x] run `output/20260716-215126`の停止区間を特定する。
- [x] P2のsolver failure Recoveryが起点であることを確認する。
- [x] P1はP2に対するSafetyBrakeで停止したことを確認する。
- [x] trajectoryの速度0が原因でないことを確認する。

## 2. Steering

- [x] requirementsを作成する。
- [x] designを作成する。
- [x] tasklistを作成する。
- [x] topic/type/Domain契約を変更しない方針を確認する。

## 3. Pure core

- [x] forward distance integration helperを追加する。
- [x] Recovery policyと終了理由を追加する。
- [x] 0 m/sでも設定velocity limitを返す。
- [x] invalid config/inputをfail-closedで扱う。

## 4. MPC integration

- [x] phase progress stateを追加する。
- [x] phase遷移時に距離/stall stateをresetする。
- [x] ShiftOut/Return/Recoveryの距離判定を積算値へ変更する。
- [x] Recovery current-speed clampを削除する。
- [x] stall/timeoutでRecoveryをIdleへ戻す。
- [x] solver cooldownをline開始条件へ追加する。
- [x] SafetyBrake/EmergencyBrakeの既存優先順位を維持する。
- [x] debug logへelapsed/distance/stall/limitを追加する。

## 5. Config

- [x] stall speedを追加する。
- [x] stall timeoutを追加する。
- [x] total timeoutを追加する。
- [x] max observation gapを追加する。
- [x] solver cooldownを追加する。
- [x] finite/range validationを追加する。

## 6. Unit tests

- [x] 可変速度列の積算距離を検証する。
- [x] 最終速度×全経過時間にならないことを検証する。
- [x] observation gap/rollbackを積算しないことを検証する。
- [x] 0速度でもRecovery velocity limitが3.0 m/sであることを検証する。
- [x] lateral/distance完了を検証する。
- [x] stall/total timeoutを検証する。
- [x] invalid設定を拒否する。

## 7. Verification

- [x] `git diff --check`
- [x] `test_v2x_overtake_core`
- [x] `make autoware-build`
- [x] `make down && make dev3`
- [x] P1/P2停止継続時間をログから計測する。
- [x] Recovery exit reasonとcooldownを確認する。
- [x] collision/wall/control fail-safeを確認する。

## 8. Documentation

- [x] `docs/spec/mpc-integration.md`を更新する。
- [x] 暫定値とrollbackを記録する。
- [x] runtime run IDと結果を追記する。

## Verification result

- run: `output/20260716-224008`
- P1/P2/P3: 発進後約128秒、1秒周期sampleで`ego <= 0.15 m/s`の連続区間0秒。
- P1 Recovery: 3.075秒、`distance complete`。
- P2 Recovery: 0.323秒、0.325秒、1.400秒で`lateral complete`または`distance complete`。
- Recovery speed limit: debug上で`3.00 m/s`を維持。
- solver cooldown: runtime triggerなし。pure unit testで2秒期限、非短縮、境界解除を確認。
- AWSIM: collision/wall/fatal/exceptionなし。
- package test: 17 CTest中16成功。作業前変更のtrajectory終端と既存fixtureの不一致による`test_path_core` 1件だけ失敗。
- relevant unit: `test_v2x_overtake_core` 22/22成功。

## Rollback

- `v2x_overtake_line_enabled: false`で明示的lineを無効化できる。
- P1/P2別trajectory、Boost、stuck recovery設定はrollbackへ混ぜない。
