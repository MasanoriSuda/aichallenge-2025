# Tasklist

- [x] 最新runのPass中断から壁接触までを照合する
- [x] DynamicMissionWait保持判定とcommit snapshotを確認する
- [x] 保持要求へforward-completion latchを追加する
- [x] controllerからsnapshotを渡し、診断ログへ追加する
- [x] prefixなしcommit保持の回帰テストを追加する
- [x] `make autoware-build`を実行する
- [x] `colcon test --packages-select multi_purpose_mpc_ros`を実行する
- [x] 対象変更だけをコミットする

## 動的確認（ユーザー試走）

- [ ] `forward_latched=1`のPass-origin waitがreselect timeoutでIdleへ落ちない
- [ ] rear-clear後にReturn -> Idleへ完遂する
- [ ] SafetyBrakeと壁接触が減る
- [ ] hard fault時のRecovery保護が維持される
