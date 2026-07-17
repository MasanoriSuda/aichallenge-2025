# 通常MPC Failure操舵中立復帰 Design

作成日: 2026-07-18
状態: Experiment Complete（中立復帰を採用、姿勢復帰は別設計）

## 方針

`safe_failure_control()`が持つ減速fallbackは維持する。failure回数と強制中立条件から操舵モードを
決めるpure policyを`v2x_overtake_core`へ追加し、controller内の分岐を検証可能にする。

## 制御フロー

1. 現周期の連続failure数を`infeasibility_counter + 1`で求める。
2. OvertakeLine Recovery / solver re-entry gate中は即時neutralizeする。
3. 通常制御では`solver_failure_steering_hold_cycles`以下なら直前操舵を保持する。
4. 待機周期を超えたら、既存helperで操舵を0へrate limitする。
5. solver成功時は従来どおりfailure counterをresetする。

## 変更対象

- `config/config.yaml`: 通常failureの操舵hold周期。
- `v2x_overtake_core.hpp/.cpp`: neutralize開始判定のpure policy。
- `mpc_controller_cpp.cpp`: config parse、fallback分岐、diagnostic。
- `test_v2x_overtake_core.cpp`: policy境界とrate limit回帰。
- `docs/spec/mpc-integration.md`: 2025 AWSIM向け暫定仕様。

## 安全性と非対象

- 速度はfailure初周期から既存`a_min`で減速し、hold中も加速しない。
- solverが正常なWP222 wall事象は本変更では解決しない。
- 大きな横・方位誤差からの再発進gateや経路復帰は別設計とし、本変更に混ぜない。
- ROS / 評価インターフェースは変更しない。

## 判定

- Pass: unit境界どおりに動き、liveで連続failure時の操舵保持が解消し、既存追い越しRecoveryに回帰がない。
- Inconclusive: liveで対象failureが発火しない。unit/buildと通常走行回帰のみ採用判断に使う。
- Fail: 単発failureで不要な操舵変化、rate超過、または停止・接触の明確な悪化がある。

## 実験後の設計判断

操舵holdの長期継続は解消でき、D3の連続failure中も現在wall contactは0のまま停止できたため、
本変更は安全fallbackとして残す。ただし操舵を0へ戻すだけでは、すでに大きくずれた車体姿勢を
reference pathへ戻せない。次の修正ではsolver failure回数だけでなく`e_psi`、静的rollout、
footprint clearanceを使い、再配向可能なRecovery primitiveと再合流条件を別ステアリングで設計する。
`forward_duration_limit`だけを延長する変更は、壁方向への駆動時間も延ばすため採用しない。
