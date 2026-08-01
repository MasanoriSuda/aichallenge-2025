# 追い越しruntime局所リファクタリング 要件

## 目的

`output/20260801-202255/d1/autoware.log`で確認された長時間の`FollowPrepare`、
wall Recovery、Pass未完遂を次の性能修正で安全に扱えるよう、
`mpc_controller_cpp.cpp`に混在している追い越しmission、横経路目標、
車体分離geometry、速度reserveの組み立てを純粋policyへ分離する。

## 対象

- Recovery完了後にcommitted pass missionを保持する条件。
- 固定corridor目標とwall/target-separationを合成する横目標policy。
- minimum-motion Passで共有する車体長・横分離・予測重複geometry。
- 車体未分離時のclosing reserve policy。
- 既存`OvertakeLineTransitionAction`との責務境界を明文化する。

## 変更しない条件

- ROS topic/service/message、launch、評価JSONの契約を変更しない。
- YAML parameter、既定値、速度・加速度・壁余裕を変更しない。
- FSMの遷移条件、mission保持時間、side選択、Recovery条件を変更しない。
- ログ文字列と主要debug fieldを変更しない。
- Stuck Recovery、MPC solver、trajectory/localizationには触れない。
- 直前の`20260801-overtake-completion-stability`差分を保持する。

## Definition of Done

- controllerはROS/model状態の収集とpolicy結果の適用を担当する。
- 抽出したpolicyはROS非依存の単体テストで現行式と境界条件を固定する。
- `multi_purpose_mpc_ros`のbuild/testが成功する。
- `git diff --check`が成功する。
- 動的性能の改善は次のステアリングへ分離する。
