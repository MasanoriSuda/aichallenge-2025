# Requirements

## 背景

`output/20260815-084110/d1/autoware.log`では、MPCC-liteがPass継続候補を
hard-feasibleとして評価し続けている一方、Mission期限延長は
`deadline_extend=0.00/3.75`のままで、episode 5と7が
`same-target Mission total budget expired`によりRecoveryへ移行した。

現行実装の期限延長はDynamicMissionWaitから同側PassPlanを置換した瞬間に、
候補が完全なrear-clear予測を保持している場合だけ実行される。置換後に
MPCC-liteが更新する完遂予測はMission期限判定へ渡されない。

## 目的

- MPCC-liteが更新した同側Passの完遂予測をMission期限判定へ引き継ぐ。
- 完全rear-clear予測とreceding-prefixの完遂予測を区別する。
- hard faultや別側候補を理由にMission期限を延長しない。
- 延長は既存の累積上限3.75秒以内とし、無期限化しない。

## 制約

- ROS 2 topic、message、launch、評価インターフェースを変更しない。
- `aichallenge_system`を変更しない。
- Missionの基本上限15秒と延長上限の設定値を変更しない。
- ShiftOut、FollowPrepare、Return、Recoveryには新しい延長権限を与えない。
- ユーザー生成物`aichallenge/result-summary.json`を変更・コミットしない。

## Definition of Done

- freshかつhard-feasibleなMPCC-lite同側branchから型付き完遂予測を生成できる。
- active Pass、同じside、target連続、物理hard faultなしの場合だけ期限延長に使う。
- 完遂予測が必要残時間を上回る場合、累積上限内で期限が延長される。
- stale、別側、ShiftOut、wall/solver/emergency faultでは延長されない。
- 対象単体テストと`make autoware-build`が成功する。
