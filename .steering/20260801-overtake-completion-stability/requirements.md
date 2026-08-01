# Overtake完遂安定化 要件

## 目的

`output/20260801-182440/d1/autoware.log`で確認された、追い越し開始後の
SafetyBrake中断、壁Recovery、直後のMPC連続失敗を減らし、開始した通常Overtakeを
`Return -> Idle`まで完遂する割合を上げる。

## 対象事象

- `Idle -> ShiftOut` 10回に対して`Return -> Idle`は5回。
- `Pass -> FollowPrepare`が6回あり、いずれもmoving front hard distanceによる
  SafetyBrakeだった。
- OvertakeLineからRecoveryへ7回遷移した。
- wall margin違反後、SafetyBrakeがRecoveryを即Idleへ戻した事象で、OSQP失敗が
  126周期継続した。
- 評価結果はcrash 3回、wall 2回、合計40.57秒のペナルティだった。

## 必須修正

1. 車体矩形がまだ横方向に分離していないShiftOut/Passでは、hard-distanceへ
   突入する前にclosing-speed referenceを残距離から0 m/sまで縮める。
2. 一度front capを解除したminimum-motion Passでは、予測重複の確認時間を
   front-danger抑制とcap再適用で共通利用する。
3. Entry preflightは通常MPC horizonだけでなく、V2X gap plannerが承認した
   lookahead全域を同じwall・横加速度条件で検査する。
4. SafetyBrakeがRecoveryと同時発生してもRecovery missionをIdleへ破棄しない。
   SafetyBrake中は横経路を出力せず、解除後にRecoveryを再開する。
5. 追加wall marginを0.10 mから0.15 mへ上げ、candidate admissionと実行時検査を
   同じ値にする。

## 変更しない条件

- 現在車体矩形が重複している場合やactual wall contactは抑制しない。
- 別車両、target jump、V2X不明、inter-vehicle corridorのSafetyBrakeは維持する。
- OSQP反復数、MPC hard bounds、最大加速度1.0 m/s2は変更しない。
- ROS topic/service/message、launch、評価JSONの契約を変更しない。
- Stuck RecoveryのReverse/Forward距離・速度は変更しない。

## Definition of Done

- unseparated closing reserveと予測重複graceを単体テストで確認できる。
- full-lookahead preflightが使用される。
- Recovery中SafetyBrakeでmission stateが破棄されない。
- `multi_purpose_mpc_ros`のテストとビルドが成功する。
- 試走では`Return -> Idle`完遂率、SafetyBrake時間、Recovery、wall/crash penaltyを
  変更前と比較できる。
