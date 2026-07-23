# オーバーテイク根本原因修正 設計

## 基本方針

現行の横回避、壁制約、緊急停止は維持し、失敗ログで確認できた三つの境界だけを
小さく修正する。

## 1. Ready -> Start の V2X tracking 継続

AWSIM は Ready 中に物理発進し、V2X planning session も Ready から有効になる。
したがって同一セッションの Start は、stuck recovery の内部状態を初期化しても、
共有 `V2XGapPlanner` の車両履歴を消去してはならない。

V2X tracking を二層に分ける。

- Start / race-relaxed retry:
  - Recovery completeness epoch を進める。
  - 通常挙動用の車両位置・速度履歴は保持する。
- Spawned / Grounded / Finish:
  - completeness epoch と通常履歴を両方消去する。

これによりRecoveryはreset後の完全messageを必要としたまま、Start直後の速度推定は
Ready中の直前サンプルを利用できる。

## 2. ShiftOut / Pass の速度仲裁

通常 Follow の車間回復速度は変更しない。

- Idle / Follow:
  - 現行の距離回復 cap を維持する。
- ShiftOut:
  - generic moving-front clearance cap を適用しない。
  - adaptive closing speed、front risk、EmergencyBrake は維持する。
- Pass:
  - generic moving-front clearance cap を適用しない。
  - 既存の unlatched Pass closing cap、front risk、EmergencyBrake を維持する。
  - 同一targetとの物理的な横離隔をlatchした後は、前車由来のstage速度capも解除する。
    domain/global速度、加速度、front risk、EmergencyBrakeは引き続き維持する。
- start-grid breakout:
  - 既存の検証済み corridor 所有を維持する。

新規通常Overtakeを一律に実測速度優位で禁止すると、進入前はFollow側の速度capが
有効なため状態遷移がデッドロックし得る。このため通常の完遂距離判定は維持する。
一方、soft/hard curve entryが完遂距離不足を例外的に迂回する場合だけ、
現在の実測速度が前走車より`v2x_overtake_completion_min_relative_speed`以上速いことを
要求する。さらにtarget距離を
`v2x_overtake_guard_min_front_distance + v2x_overtake_line_shift_distance +
v2x_overtake_line_return_clear_distance`以内に限定し、遠方からpass lineへ出て
ヘアピン中に前車へ離される反復を防ぐ。この距離は現行設定で11 mである。

状態別の所有権と入口条件は `v2x_overtake_core` の純粋関数として切り出し、
単体テストする。

## 3. 入口と完遂距離の整合

- close-followはprepare距離を緩和できるが、通常の新規進入距離5 mを下回らない。
- 新規完遂距離は、nominal/reachable速度と実際のOvertake stage速度上限の小さい方で
  計算する。
- active hard-curve完遂距離も、同じstage速度上限で計算する。
- curve entry例外は完遂距離不足時のみ、実測速度差と近距離entry範囲を要求する。
- start-grid breakout、開始済みlineのcurve continuation、通常完遂距離が成立する入口は
  この例外条件の対象外とする。

実走中に一時的に前車leadが増えるだけでRecoveryへ落とす監視は、加速過渡と
ヘアピン減速を失敗と誤判定して再進入を繰り返したため採用しない。位置ジャンプ、
course progress discontinuity、solver recoveryなど既存の明示的fail-safeを維持する。

## 動的確認で残った課題

`output/20260724-021819/d1/autoware.log`では、横離隔latch後に
`cap_release=1`、`desired_v=11.11 m/s`となり、速度仲裁の解除自体は確認できた。
一方、固定したpass corridor目標が後続区間のwall/static-map制約で大きくclampされ、
実速度は約2 m/sまで低下した。target leadが投影範囲端まで増えて
`locked target course progress discontinuity`となり、`Pass -> Return`は未達である。

残作業は、固定corridor目標のtarget横離隔検証、物理横離隔を満たさない
`ShiftOut -> Pass`の禁止、同一targetに限定した短時間の分類境界holdである。
今回は原因範囲を越える横経路生成変更を追加せず、未完遂として記録する。

## 影響範囲

- `multi_purpose_mpc_ros` 内部状態と制御目標のみ
- ROS interface、launch、package依存、result schemaへの影響なし
- `aichallenge_system` 変更なし

## 安全性

- front risk Emergency は追い越し速度より優先する。
- Recovery は中央線へ戻る既存の速度制限付き経路を使う。
- V2X target が不正、非finite、位置ジャンプの場合は従来どおり fail-closed。
- シミュレータ確認前に実車設定へ反映しない。
