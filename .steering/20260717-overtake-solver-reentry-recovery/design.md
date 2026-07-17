# 追い越しSolver復帰ゲート Design

作成日: 2026-07-17
状態: Experiment Complete（gate Pass / dev3全体Partial）

## 原因

現在のsolver fallbackは最後の有効操舵を固定する。D2ではRecovery遷移時の右操舵`-0.179 rad`を
保持したまま約3 m/sから減速し、solverが復旧できるcorridorからさらに離れた。また既存の再進入抑制は
2秒の時刻条件だけなので、連続failure中でも期限後にOvertakeLineが再開される。

## 設計

### Recovery fallback steering

OvertakeLine Recoveryまたはsolver-abort episodeでは、内部操舵を1周期あたり
`steer_rate_max * Ts`以内で0へ近づける。速度は既存の`a_min`による減速を維持する。通常走行中の
solver fallbackは今回のスコープ外とし、従来どおり最後の操舵を保持する。

### Solver-health re-entry gate

Recovery中のsolver fallbackまたはShiftOut / Passのfailure threshold到達をepisode失敗とする。
episode終了時にgateをarmし、以下を同時に満たした場合だけ解除する。

- `v2x_overtake_solver_cooldown_sec`経過済み
- 通常MPCが`v2x_overtake_solver_recovery_success_cycles`回連続成功

gate中の1回のfailureで成功カウントを0へ戻す。gate中はOvertakeLine、overtake side target、gap
plannerによる追い越し横目標を使用せず、通常trajectoryをsolver復旧判定に使う。

### 初期値

`v2x_overtake_solver_recovery_success_cycles: 20`とする。40 Hz制御で0.5秒の連続成功に相当する。

## 判定

- Pass: D2がsolverを復旧してWP60以上へ進み、再進入ループと3台停止がない。
- Partial: solver未復旧の安全停止を維持し、再進入ループと他車巻き込みはない。
- Fail: 連続failure中に再進入する、壁接触する、または3台停止が再発する。

## Rollback

追加したre-entry gateとRecovery中立復帰を外し、既存の時間cooldownだけへ戻す。

## 実験判定

solver-health re-entry gateは採用する。実走でfailure threshold、Recovery、gate arm、cooldown、20連続
成功、gate releaseの順序を確認し、D2はその後WP220まで走行した。連続failure中に時刻だけで再進入
する経路は閉じた。

Recovery中立復帰はunit testとビルドまで成功したが、実走ではRecoveryへ遷移した次周期にsolverが
復旧したためfallback commandとしては発火しなかった。元run相当の連続failure再現時に追加確認する。

D3のWP222 wall contactはOvertakeLine非作動中の通常走行事象で、本gateとは別ステアリング対象とする。
