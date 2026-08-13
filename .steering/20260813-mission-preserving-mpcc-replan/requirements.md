# Requirements

## 目的

最新走行 `output/20260813-222025` で残った追い越し中断を減らし、1位車および
Pro案の「現在のMissionを捨てず、実行可能な解を更新し続ける」方針へ寄せる。

## 対象事象

- `optimized horizon escaped target separation bounds` による即時Recovery
- `SafeSeparation aborted: local distance limit` 後の早期Mission破棄
- 速度を落とした成立性再検証で、相手の相対位置予測が元速度のまま固定される不整合

## 制約

- 実車体重複、壁接触、EmergencyBrakeなどのhard faultは従来どおりfail closedとする。
- ROS 2 topic、launch、評価インターフェースは変更しない。
- Recovery全般やReverseは今回の対象外とする。
- 新しい調整パラメータは増やさず、現行設定の意味を保つ。

## 完了条件

- 候補速度ごとに相手の時系列相対位置とtarget制約を再計算する。
- softなtarget境界不成立では、Freshな同側Missionまたはlast-feasible候補をRecoveryより先に試す。
- SafeSeparationのsoft abortでは、Freshな同側MissionをDynamicMissionWaitより先に試す。
- hard faultが従来どおりRecoveryへ入ることを単体テストで維持する。
- 対象packageのビルドとテストが成功する。
