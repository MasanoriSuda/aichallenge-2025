# Tasklist

- [x] 最新ログからSafetyBrake移行時のbody-clear状態を確認する
- [x] body-clear deadline計算をROS非依存coreへ追加する
- [x] 候補選択をbody-clear成立時刻優先へ変更する
- [x] controllerへ観測target横速度とdeadline評価を接続する
- [x] 集約ログへdeadlineの選択・棄却情報を追加する
- [x] core単体テストを追加する
- [x] package test/buildを実施する

## 動的確認項目

- `body_clear_t`と`hard_t`を含むmission candidate selectedログ
- `body_deadline_rejected`を含むcandidate search rejectedログ
- Overtake -> SafetyBrake回数と、そのときの`body_clear`
- Idle -> ShiftOut -> Pass -> Return -> Idleの完遂率
- wall Recovery、接触、ラップタイム
