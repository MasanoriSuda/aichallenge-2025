# 壁接触予防・再合流安定化 Requirements

作成日: 2026-07-17
状態: Experiment Complete / Follow-up Required

## 目的

`output/20260717-225927`で発生した、前方危険車両の短時間消失による追突リスクと、
Stuck Recovery後のLowSpeedRejoin中の新規wall contactを、安全gateを維持したまま予防する。

## Baseline evidence

- D2はD3まで3.95 m、相対速度1.75 m/sでSafetyBrakeへ入った。
- 約0.4秒後にD3がfront/side分類から一時的に外れ、D2は2.16 m/sでCruiseへ戻った。
- D3再検出時にはfront有効距離が-0.66 mとなり、D2/D3がWP 183付近で停止した。
- D1はwall contactを44 cellsから0へ減らして累積2.015 m退避した。
- LowSpeedRejoin開始時はcurrent footprint clearだったが、前進rollout未確認のまま走行し、
  約1.8秒後に新規contactを作って`rejoin_unsafe` SafeStopとなった。

## 機能要件

### R-HAZARD-01: 前方危険target hold

- SafetyBrakeを発火させたtarget IDを短時間保持する。
- targetがfront/side分類から一時的に外れてもhold中はSafetyBrakeを解除しない。
- 新しい危険観測でhold期限を延長する。
- hold期限到達または明確なrear-clearで解除する。
- 非有限時刻・不正設定では安全側へ倒す。

### R-REJOIN-01: 前進経路preflight

- LowSpeedRejoinへ入る前と走行中に、normal MPC先頭操舵を使った前進swept-footprintを評価する。
- current footprint contact、out-of-map、unknown、前進rollout collisionでは駆動しない。
- current footprintはclearだが前進rolloutだけがblockedの場合、即SafeStopせず停止確認後に
  bounded recovery候補を再評価できる。
- current footprintに新規contactがある場合は従来どおりSafeStopする。

### R-REJOIN-02: headingを考慮した追加退避

- current footprint clear時のReverse Straight / Left / Right候補は、static safeな候補のうち
  終端heading errorを最小化する候補を優先する。
- contact中は従来どおりcontact reductionを優先する。
- actuation開始後は候補を変更しない。

### R-LOG-01

- V2X debugへhazard hold状態、target ID、残り時間を出す。
- Recovery logへrejoin forward preflight結果を出す。

## 非機能要件

- ROS topic/service/message、Domain、result JSONの契約を変更しない。
- 参加者実装内に閉じる。
- occupancy map、V2X completeness、gear、speed上限を緩和しない。
- 2025 AWSIM向け暫定値と明記し、2026公式値として扱わない。

## 受け入れ条件

1. hazard holdのarm/継続/期限/clear/不正入力をunit testする。
2. blocked rejoin pathがcurrent contactなしなら再評価へ戻り、contactありならSafeStopすることをtestする。
3. 対象testと`make autoware-build`が成功する。
4. dev3でD2が危険target消失直後にCruiseへ戻らない。
5. D1のLowSpeedRejoin中に新規contactを作らない。
6. 旧runの約62秒を超えて3台停止列が発生しない。

## 受け入れ結果

- 条件1〜4は達成した。
- 条件5は、D1がLowSpeedRejoinへ駆動する前に前進preflightがwall collisionを検出し、
  新規contactを作らず停止したため達成した。
- 条件6は未達。`output/20260717-232948`ではD2が旧runの停止時刻を超えて走行を継続したが、
  D2のStartから約79秒後に停止列へ到達し、最終的に3台停止した。
- 今回の未達原因はhazard targetの短時間消失ではなく、壁際で前進不能なD1、D1の後退経路を
  塞ぐD3、D3後方でSafetyBrakeするD2による閉塞である。
