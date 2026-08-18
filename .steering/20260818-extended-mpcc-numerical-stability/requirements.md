# Requirements

## 背景

`20260818-181209` の試走では、拡張 velocity-progress MPCC が追い越し中に
`maximum iterations reached` を反復した。OSQP failure は 3.16 %、制御周期超過は
1.02 %、最大 solve は 29.6 ms であり、25 ms 制御周期を超えた。

## 目的

- 拡張 MPCC の progress 状態を局所座標化し、QP の数値スケールを改善する。
- 拡張 solve 失敗後の同一周期二重 solve 連鎖を抑える。
- 拡張 MPCC 単体の採用率・失敗率・計算時間を走行ログから確認できるようにする。

## 制約

- 既存の3状態MPCCをcycle-local fallbackとして維持する。
- hard speed cap、壁制約、物理再検証を緩和しない。
- ROS topic/service、提出インターフェースを変更しない。
- `aichallenge/result-summary.json` は試走生成物として変更・コミットしない。

## Definition of Done

- progress原点が大きくても拡張QP内部のthetaが0近傍になる。
- warm-startのthetaが新しい原点へ再基準化される。
- 拡張solve失敗後、設定時間は旧MPCCだけを解く。
- 拡張MPCC専用の1 Hz集約ログが出る。
- 対象packageのbuild/testが成功する。
