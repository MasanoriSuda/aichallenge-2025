# 追い越し全区間mission path 要件

## 目的

追い越し開始時に成立すると判断した横経路を`ShiftOut -> Pass -> Return`まで一貫して
実行し、相手車両の横揺れやbehavior判定の一時変化で経路目標を動かさない。
成立しないmissionを`FollowPrepare`で保持し続けず、同じ相手を追い越し完了直後に
再捕捉しない。

## 対象

- `ShiftOut`、`Pass`、`Return`を合成した横経路profile。
- entry時の全区間static-wall、wall margin、横加速度preflight。
- 採用済みpass goalとsideの固定。
- `FollowPrepare`の時間・走行距離上限。
- rear-clear確定のlatchと、完了target IDの短時間再捕捉抑止。
- 上記policyのROS非依存単体テスト。

## 変更しない条件

- ROS topic/service/message、launch、評価JSONの契約を変更しない。
- 速度、加速度、壁余裕、gap幅など既存の攻撃度parameterを変更しない。
- Stuck Recovery、MPC solver、trajectory、localizationには触れない。
- 既存のstart-grid inter-vehicle corridorと停止車両bypassの所有権を維持する。
- 実車向けの挙動保証は行わず、現行simulation race設定として扱う。

## Definition of Done

- 初回entryは全区間profileがhorizon内で検査できた場合だけcommitする。
- commit後のpass goalはtarget lateral更新で移動しない。
- `FollowPrepare`は4秒または20 mでmissionを終了し、次周期から左右を再評価できる。
- rear-clear確定後は同じmissionを`Pass`へ戻さない。
- Return完了直後の同一target新規entryを短時間抑止する。
- `multi_purpose_mpc_ros`のbuild/testと`git diff --check`が成功する。

