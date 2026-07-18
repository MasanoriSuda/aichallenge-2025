# Design

## 状態遷移

### Forward hazard

`ForwardManeuver -> SafeStop`を`ForwardManeuver -> StopAndReassess`へ変更する。指令は即時
HoldStopで、停止確認後に既存のstatic / V2X / gear gateを再実行する。障害が残れば
`WaitForClear`へ入り、既存timeoutとclear再確認で上限を維持する。

### Side-wall stepwise

stepwise選択をpure helperへ分離する。side escapeが有効でcurrent footprintがclearかつ
wall regionがNone / Left / Right / Mixedなら短いstepを使う。Front / Rearのclear footprintは
従来の方向別候補を維持する。

非stepwise後退のduration limitでも、`attempt_count < max_attempts`ならDriveへ戻して
StopAndReassessへ進む。距離上限とattempt上限は従来どおり独立に適用する。

### Rejoin timeout

`retry_on_timeout=true`かつattemptまたはescape-step budgetが残る場合、LowSpeedRejoinを停止し、
escape確認ラッチを解除してStopAndConfirmへ戻す。ROS adapterはこの遷移を検出し、episode距離、
選択primitive、接触baselineを新しい離脱として初期化する。再joinも失敗してbudgetを使い切れば
SafeStopする。

### Solver fallbackと壁方向

`solver_evidence_free_recovery_enabled`は壁なし復帰経路の許可であり、壁ありsolver fallbackまで
無条件にreverse-onlyへするフラグではない。actuation候補生成前に同じoccupancy mapで壁方向を
分類し、次の条件だけreverse-onlyとする。

- 壁なしでevidence-free solver recoveryが有効。
- `abs(e_psi) >= solver_reverse_only_heading_error_rad`。
- coordinated stopまたは開始済みreverse-only episode。

後壁ありかつ姿勢誤差上限未満なら、既存の短距離Forward候補を選べる。ただし候補はstatic swept
footprint、V2X corridor、速度、距離、時間、gear gateをすべて通過しなければactuationしない。

solver fallbackの資格は、外部maneuverで車両が動くとdetectorの無進捗timerから消える。
solver起因episodeだけこの資格をLowSpeedRejoin完了までcoreで保持し、solverに依存しない
feedback steeringを継続する。非solver起因episodeでは既存のsolver graceとSafeStopを維持する。

### Bidirectional contact escape

Side / Mixed接触ではReverse Straight/Left/Rightに加えてForward Straight/Left/Rightも同じ0.4 m、
同じsteering sampleで評価する。`RequireImprovement`のcontact減少数が最大の候補を採用し、同値は
従来のReverseを優先する。Reverseがstatic上優位だがV2Xで塞がる場合だけ、改善するForward候補を
既存deadlock fallbackとして再評価する。

## 設定

- `rejoin.retry_on_timeout: true`
- `maneuver.max_attempts: 3`

3 attemptsは2025 AWSIM用暫定値。距離、速度、時間、step数、static/V2X gateは変更しない。

## テスト

1. forward hazardがHoldStop / StopAndReassessになり、clear後にCheckClearanceへ戻る。
2. 非stepwise reverse durationが残attemptで再評価される。
3. rejoin timeoutが有効時だけStopAndConfirmへ戻る。
4. clear side wallがstepwise、clear Front / Rearが非stepwiseになる。
5. budget消費後とretry無効時は従来どおりSafeStopする。
6. wall-free solver fallbackはreverse-only、壁あり小姿勢誤差は壁方向選択、大姿勢誤差は
   reverse-onlyを維持する。
7. solver起因episodeはfallback継続中でもLowSpeedRejoin commandを返す。
8. contact中Forward候補もRequireImprovementを満たす場合だけ選択される。
