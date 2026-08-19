# Tasklist

- [x] 最新ログでMission生成から実行開始までの時系列を照合する
- [x] async workerとmailboxの結果公開条件を特定する
- [x] 完了結果の単調公開条件を純粋関数化する
- [x] mailboxへ最新公開sequenceを追加する
- [x] context reset時に公開sequenceを初期化する
- [x] newer pending job存在時の単体試験を追加する
- [x] `make autoware-build`を実行する
- [x] `colcon test --packages-select multi_purpose_mpc_ros`を実行する
- [x] 変更対象だけをコミットする

## 動的確認（ユーザー試走）

- [ ] 停止車のcomplete Mission生成後にasync `adopted`が増える
- [ ] 生成済みMissionが`Idle -> ShiftOut`へ引き渡される
- [ ] 無理由のMission消失からSafetyBrake／Stuck Recoveryへ進まない
- [ ] context/target/phase不一致の古い結果を採用しない
