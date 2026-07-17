# V2X低速回避・回復方向デッドロック修正 Requirements

作成日: 2026-07-17
状態: Experiment Complete / Acceptance Failed

## 目的

3台走行で停止車を回避する車両が`LowSpeedAvoidance`へ長時間残留し、先頭車の後退回復も
後方車で塞がれて全車停止する連鎖を、安全条件を弱めずに解消する。

## Baseline evidence

対象run: `output/20260717-220801`

- D3はWP 134でwall evidenceとOSQP連続失敗へ入り、WP 135で停止した。
- D2はD3を対象に`LowSpeedAvoidance`へ入り、約84.2秒間WP 135に停滞した。
- D3はReverse候補を選択したが後方車で塞がれ、`clearance_wait_timed_out`でSafeStopした。
- D1もWP 135でReverse候補を選択した後、後方corridor blockでSafeStopした。
- D2は周回後に停止D1を28.37 m前方で検出し、約2.54 m後方でSafetyBrake停止した。
  共通コース進捗による早期検出は動作している。

## 機能要件

### R-STALL-01: LowSpeedAvoidance停滞監視

- `LowSpeedAvoidance`中に設定速度未満が設定時間継続した場合、局所回避targetを解除する。
- 停滞解除後は、危険な前方車があればSafetyBrake、前方または横車両がいればFollow、
  関連車両がなければCruiseへ遷移する。
- 短いcooldown中は同じ低速回避へ即再進入しない。
- 時刻逆行、非有限時刻、観測gapでは停滞時間を継続しない。

### R-DIRECTION-01: 駆動前の回復方向再評価

- `SUSPECT_STUCK`、AWSIM回復待機、停止確認、clearance待機、recoverable SafeStopでは
  static候補だけをepisodeへ固定しない。
- gear要求またはForward maneuver開始に到達した時点で初めて候補を固定する。
- 候補固定後は途中で方向・操舵符号を変更しない。

### R-DIRECTION-02: 前進fallbackの評価範囲

- 現在footprintがmap上clearなら、Front / Noneを含めて安全なForward Straight / Left / Rightを
  後退block時の候補として評価できる。
- static swept footprintとfresh / completeなV2X forward corridorの両方がclearの場合だけ採用する。
- 現在map contactあり、unknown、out-of-map、V2X不完全時は従来どおりfail-closedとする。

### R-LOG-01: 実験診断

- V2X debugへLowSpeedAvoidance停滞時間、timeout、cooldown状態を出す。
- 回復候補ログは、実際に候補を固定した時点を示す。

## 非機能要件

- ROS topic/service/message、Domain構成、評価JSONを変更しない。
- `aichallenge_submit/`内の実装に閉じる。
- SafetyBrake、occupancy map、V2X corridor、Boost、gearの安全gateを緩和しない。
- `simulation_only=true`を維持する。
- D3のWP 134 wall侵入・OSQP失敗の予防制御は別課題とし、本実験では観測する。

## 受け入れ条件

1. 停滞watchdogの継続・reset・timeoutをunit testする。
2. 回復候補が駆動前に固定されず、駆動開始時に固定されることをunit testする。
3. 対象testと`make autoware-build`が成功する。
4. `make dev3`でLowSpeedAvoidanceの低速停滞が設定時間内に解除される。
5. `output/20260717-220801`と同じWP 134-136停止連鎖で、3台すべてが恒久停止しない。
6. 安全なstatic / V2X候補が存在しない車両は無理に駆動せずSafeStopを維持する。

## 実験判定

`output/20260717-225927`で実験した。条件1〜3と6は成立したが、条件4の対象となる
`LowSpeedAvoidance`停滞はこのrunでは発生せず、条件5は不成立だった。D1は壁接触から
2.015 m退避した後の再合流中に新規接触して`rejoin_unsafe`、D3は安全な前進fallbackが
成立しない姿勢でReverse corridorをD2に塞がれ`clearance_wait_timed_out`となり、D2は
両車を危険車両としてSafetyBrake停止した。詳細は`results.md`を参照する。
