# Results

## 実装結果

- `WaitAwsimRecovery`進入poseを保存し、待機中の位置・yaw不連続を検出するようにした。
- 外部pose変化を検出した場合、現在poseからreference waypointをglobal再対応する。
- AWSIM待機解除時に、補正前の観測anchor、maneuver距離、contact baseline、選択済みprimitive、direction latch、Forward失敗履歴を張り直す。
- footprintがclearでも、lateral / heading誤差がrejoin許容外ならNormalへ直帰せず、現在snapshotで方向を再評価する。
- MPCの外部maneuver履歴はpose不連続または方向再評価が必要な場合だけresetする。

## 検証結果

- `make autoware-build`: 成功、25 packages
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功、28/28 targets
- `colcon test-result --verbose`: 1264 tests、0 errors、0 failures

## 実走で見るログ

- `AWSIM recovery pose handoff detected`
  - pose / yaw不連続を検出し、waypointを再対応したことを示す。
- `AWSIM recovery pose handoff completed`
  - `aligned=0`ならNormal直帰せず、`direction_reassessment=1`で現在姿勢からRecoveryを継続する。
- 反転後に`awsim_recovery_resolved -> Normal`へ直行せず、`StopAndConfirm -> CheckClearance`へ進むこと。
- 同一incidentで`rejoin_path_blocked`が長時間反復せず、現在poseからForward / Reverse候補が選び直されること。

実走効果確認はユーザー試走で行う。
