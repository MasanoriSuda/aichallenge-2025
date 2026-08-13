# Design

## 1. Target-bound Pass hold

予測された相手車両との離隔だけが不成立になった場合、直前のMPCC軌道を現在進捗へ
resampleする。resampleした軌道を相手制約なしで壁・横加速度・実車footprintに対して
再検証し、成立する場合だけ最大0.30秒かつ3.0 m保持する。

保持中はFSMをPassのまま維持し、Mission generationを無効化しない。したがって次周期も
通常のreceding-horizon optimizerが動き、新解が成立すればholdを解除できる。

速度は新たな加速を強制せず、hold開始時の実速度を下限とする。EmergencyBrakeなど既存の
上位hard guardは引き続きこの下限より優先する。

SafeSeparation中でも、失敗理由が短いhorizonや局所的な距離不足だけで、target継続・
実車分離・壁・Emergency・solverが正常なら、DynamicMissionWaitへ落とさずPassへ制御を
返す。次のreceding-horizon評価でholdまたは新しい候補へつなぐ。

## 2. Return preflight

PassからReturnへ遷移する直前に、現在の`e_y`から基準線までのReturn軌道を生成する。
既定距離から既存のPass extension上限まで1 m刻みで評価し、壁・横加速度を同時に満たす
最短距離をMissionへ保存する。

どの距離でも成立しない場合はReturnへ遷移せず、現在のPassを維持する。その後の通常の
壁hard guardが本当に不成立なら、従来どおりRecoveryが所有する。

## 3. 対象遷移

- rear-clear confirmed
- SafeSeparationからのspeed-preserving Return
- runtime wall warningからのReturn
- Pass horizon hard limitでrear-clear済みのReturn

FollowPrepareなど、すでにPass実行権を失った状態のReturnは今回の対象外とする。
