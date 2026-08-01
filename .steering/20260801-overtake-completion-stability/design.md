# 設計

## 1. lateral separation前のclosing reserve

現行はShiftOutのclosing speedに0.8 m/sの下限、未latch Passに0.5 m/sを使用する。
横分離が遅れた場合でも正の相対速度を維持するため、moving-front hard distanceへ
到達し、0 m/s SafetyBrakeへ切り替わる。

車体矩形が未分離の間だけ次を適用する。

```text
protected_distance = max(moving_hard_distance, body_longitudinal_clearance)
                     + reserve_distance
distance_budget = max(0, target_longitudinal - protected_distance)
closing_budget = distance_budget / remaining_lateral_execution_time
effective_closing = min(stage_closing, closing_budget)
```

これはtarget速度以下へ落とす処理ではない。距離budgetがなくなった場合もtarget速度に
合わせて横経路を継続し、SafetyBrakeによる完全停止を避ける。車体矩形が分離した後は
既存のcap releaseとPass速度floorへ戻す。

## 2. predicted overlap確認の共有

現行はfront cap再適用だけが0.25秒の連続予測重複を要求し、behavior側の
front-danger抑制は同じ瞬間観測で即解除される。

minimum-motion Pass、cap解除済み、現在車体矩形非重複、target継続観測という
同一guard内で、既存の`pass_predicted_overlap_since_sec`をbehaviorとline policyの
双方から参照する。確認前の予測重複は一時的に抑制できるが、現在矩形重複、確認済み
重複、target jump、actual wall条件は従来どおりfail-closedとする。

## 3. admissionとexecutionのwall整合

`assess_side()`はgap planner用に24 m lookaheadのboundsを既に生成しているが、最後の
`evaluate_overtake_line_entry_preflight()`へは通常MPC horizonだけを渡している。
preflightへplanner horizon/boundsを渡し、固定goalをShiftOut完了後も保持した場合の
static footprintと横加速度を同じlookahead全域で検査する。

実行誤差用のwall marginは0.15 mへ上げる。gap幅0.10 m自体は変更せず、最終preflight
で0.15 mの車体余裕を満たさない候補だけをFollowへ戻す。

## 4. geometry abort後のsolver復帰

OSQP workspaceは各solveで再生成されるため、永続solver stateをresetする処理は存在
しない。ログ上の連続失敗は、wall Recoveryが次周期のSafetyBrakeでIdleへ破棄され、
壁接触姿勢の通常MPCへ即座に戻ったことが原因である。

Recovery中にSafetyBrakeを受けた場合はstateを保持して横経路出力だけ停止する。
これによりsolver fallbackはRecoveryとして最初からsteering neutralizationとre-entry
gateを使用し、SafetyBrake解除後はbounded Recovery horizonを再構築する。

## 効果確認

- `unseparated_reserve=1`の後、hard-distance SafetyBrakeへ入らず横分離できるか。
- predicted overlapが0.25秒未満ならcapとfront-dangerの状態が一致するか。
- preflight reject理由にfull-lookaheadのwall/横加速度失敗が出るか。
- `Recovery -> Idle, reason=safety brake`が0回になるか。
- wall/crash penalty、OSQP連続失敗最大周期、Return完遂率を比較する。
