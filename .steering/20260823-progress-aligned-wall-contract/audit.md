# Audit

## Observed phenomenon

`output/20260823-overtake-physical-reject-replay/d1/autoware.log` では、Overtake canonical fresh shadowの5周期が、solver・lateral row・primal・trajectoryを通過した後に全て `HardWallContact` で棄却された。

## Data flow

```text
reference stage geometry
  -> stage wall corridor at ref_wp_id + stage
  -> fixed lateral box row in legacy MpcProblem
  -> copied to extended 5-state QP
  -> MPCC optimizes independent progress state
  -> physical certificate reconstructs pose at solved progress
  -> different course position is checked against the real wall map
```

## Root-cause statement

今回の問題は、wall corridorがstage indexの属性として固定されている一方、extended MPCCではcourse progressが独立状態として変化できるという定式化上の不整合によって、別地点の壁境界がsolved poseへ適用され、その結果QP可行解が実車体証明で壁接触として表面化している。

## Evidence

- diagnosticのprogress deltaは `-5.970 m`。
- lateral row reserveは正値 `0.118 m`。
- exact footprintは同じstageで2 wall contacts。
- current pose、swept connector、map sampleの棄却は0。
- extended solverにはdynamics再線形化相当はあるが、wall boundsをsolved progressへ再対応する処理はない。

## Remaining uncertainty

progress位置対応を直した後も、reference headingとsolved headingの差によるfootprint mismatchが残る可能性がある。次の同一bag replayで残存rejectの `progress_delta` と `heading_offset` を再評価する。

## First implementation falsification

stage boxをfirst-solve progressへ1回だけ差し替える試作は、根本修正にならなかった。

- replay: `output/20260823-progress-aligned-wall-replay-v2/d1/autoware.log`
- Dynamic Escape profile由来のRTI: 32 windowでattempted/solved、failure 0
- first-solve progress mismatch最大: `17.093 m`
- profile範囲外: 合計290 stage
- Overtake `HardWallContact`: 旧版5件、試作7件

second solveがwall box更新後に別progressへ移れるため、stage boxとprogressの対応が
再び失われる。次の実装はbox差し替えではなく、`e_y` と `theta` を同じwall rowで
結合する。

## Coupled-row implementation

各予測stageについて、first solveの `theta` を含むwall profile区間を選び、次を同じQPへ
追加した。

```text
theta_segment_lower <= theta <= theta_segment_upper
wall_lower_slope * theta + wall_lower_intercept <= e_y
e_y <= wall_upper_slope * theta + wall_upper_intercept
```

profileは現在位置 `progress=0` と既存のN stage physical wall envelopeから作る。
Dynamic EscapeはOvertakeLine horizonを持たないため、同じfootprint/map APIからwall-only
profileを生成する。target/tactical corridorは既存box rowに残し、wall-only rowと混ぜない。

## Dynamic replay acceptance

- replay: `output/20260823-progress-coupled-wall-replay/d1/autoware.log`
- source bag: `output/20260823-214300-stop-authority-replay-v2/d1/rosbag2_autoware`
- recorded control commandは除外し、入力6 topicを1.5倍速で再生。
- `/awsim/state=Start`を明示注入。

Overtake canonical fresh shadow集計:

| 指標 | 一回box差し替え | coupled row |
|---|---:|---:|
| evaluated | 1171 | 821 |
| eligible/context/lateral/primal | 1106 | 794 |
| physical/complete | 1099 | 794 |
| `HardWallContact` | 7 | 0 |

選択topicの最終時刻とcontroller schedulingによりcycle総数が異なるため、絶対件数同士を
速度性能として比較しない。ただしcoupled版は全794 eligible cycleで同じcycleの最終
physical certificateまで通り、対象棄却が再現しなかった。

progress-coupled RTI:

- telemetry window: 32
- valid/feasible/applied/attempted/solved: 32/32/32/32/32
- refinement solve failure: 0
- coupled stage: 640/640
- profile範囲外: 0
- first-solve progress mismatch: 12.698〜16.093 m

最大16 m級の位置差自体は残る。従って改善はprogressをstageへ強制的に戻した結果では
なく、wall rowが最適化されたprogressへ追随した結果である。

退行確認:

- Track/Cruise: 116 eligible / 116 solved。solve failure 0。
- coupled refinement中のOSQP structural rebuild: 初期setup後0。
- callback: 4579 cycle、65 overrun、最大56.426 ms。旧試作は5306 cycle、128 overrun、
  最大67.574 msであり、少なくとも追加rowによる悪化証拠はない。ただしbag replayの
  CPU timingは実走性能の合格証拠にはしない。
- Extended全体には既存のmaximum-iteration/circuit-breakerが11件残る。coupled refinement
  32回は全成功しており、本Slice起因とは分類しない。

## Static validation

- `make autoware-build`: success、25 packages。
- focused `test_mpcc_progress`: 67/67 passed。
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 programs passed。
- `colcon test-result --verbose`: 1697 tests、0 errors、0 failures、0 skipped。
- `build/joycon_contract_guard/package.xml`欠落警告は既存生成物由来。

## Conclusion

根本原因仮説は支持された。固定stage wall boxと可変progressの不整合を、margin調整や
trust region縮小ではなくQP定式化で解消し、最終物理証明を緩めず `HardWallContact` を
0件にした。本Sliceは受入れ可能である。

実走で確認すべき残件は、Overtake/Dynamic Escape中の壁接触、coupled refinement failure、
callback overrun、およびheading差由来の物理棄却である。target obstacleの時刻・progress
対応は別の動的証拠が得られるまで本Sliceへ混ぜない。
