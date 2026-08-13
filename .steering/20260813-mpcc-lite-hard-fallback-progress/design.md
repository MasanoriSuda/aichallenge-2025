# Design

## 1. Receding-horizon failure class

failure を次の二種類に分ける。

- soft: optimizer の一時的な数値失敗。live bounds 内で再検証できる last-feasible、または検証済み baseline を利用できる。
- hard: wall bounds 不正、target separation と wall/trust bounds の矛盾、最適化結果の hard bounds 逸脱、物理再検証失敗。

hard failure では baseline を実行 horizon として残さず、既存の execution-infeasible 経路へ渡す。
これにより不可能な ShiftOut / Pass を MPC へ投入せず、既存の phase rebuild / Recovery が所有する。

## 2. Progress-aligned warm start

前回解と一緒に horizon の相対距離列、phase、phase 内進捗を保存する。
同一 Mission generation / side / phase のときだけ、今回までに進んだ距離を加えた位置で前回解を線形補間する。
前回 horizon の末尾を越えた点は今回の baseline を使う。

## 3. Shadow state scope

last-feasible resolution に target ID に加えて Mission generation、phase、side を保存する。
一つでも一致しない場合は lease を禁止する。Return の結果が Idle に漏れることを防ぐ。

current-side hold は active phase では frozen `mission_plan` を先に評価し、毎周期作り直した候補を hold として扱わない。

## 4. Timing

shadow の left assessment、right assessment、resolution、全体の wall time を既存の間引きログへ追加する。
制御周期悪化と両側 shadow 評価の相関を rosbag の command 間隔と照合できるようにする。

## 非対象

- shadow winner による side / speed / Mission の変更
- 全面 MPCC 化
- clearance parameter の攻撃化
