# 追い越しdynamic mission corridor要件

## 目的

`ShiftOut -> Pass -> Return`の固定横profileを採用する前に、同じ時系列で予測した
対象車両のinflated footprint corridorと全区間で整合することを確認する。
採用後はentry判定や左右再選択を再実行せず、OvertakeLineが固定missionを所有する。

## 対象

- gap plannerが生成した時系列free corridorと固定mission profileの照合。
- ShiftOut中の途中点を含む、pass goalの実行可能区間導出。
- 動的corridorが観測不能・不成立の場合のFollow継続。
- 固定mission中のearly side replanおよびentry preflight停止。
- ROS非依存policyの単体テスト。

## 変更しない条件

- ROS topic/service/message、launch、評価JSONの契約を変更しない。
- 速度、加速度、gap幅、壁余裕、ShiftOut/Pass/Return距離を変更しない。
- 実行中のactual footprint、static wall、target continuity、SafetyBrake監視は維持する。
- start-grid breakoutとstopped-vehicle bypassの既存所有権を変更しない。

## Definition of Done

- entry時の固定profileが、対象車両を除外した時系列free corridor内に全区間収まる。
- 最小横移動goalは、ShiftOut途中の必要離隔を満たす最小値まで外側へ補正できる。
- 動的corridorが成立しない候補はOvertakeへcommitせずFollowを継続する。
- `mission_path_frozen=true`以降はside replanとentry preflightを再実行しない。
- `multi_purpose_mpc_ros`のbuild/testと`git diff --check`が成功する。
