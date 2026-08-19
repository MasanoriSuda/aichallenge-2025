# Design

## 確認した欠陥

1. Follow中のdynamic escapeはOvertakeLine Mission開始前にも動くが、ログは
   `overtake_line_state_.episode_id`だけを出していた。このため大半が
   `episode=0`となり、planning failureとtracking failureを一意に結べない。
2. dynamic escapeのalternate候補評価はprimary側のtracking solver backoff時
   にだけ実行される。primaryのplanner/bridge失効では既知の反対側を評価せず、
   Follow authorityへ戻る。
3. active Mission失効時はsame-side/alternate cache、no-return、hard fault、
   debounceを用いているが、最終actionを含む正規化ログがなく、長い状態ログを
   手作業で照合しないと棄却点が分からない。

## 方針

### 1. 相関IDを分離

- `attempt_id`: Follow中のdynamic escape要求が開始するたびに採番する。
- `mission_episode_id`: OvertakeLine Missionの既存episode。Mission外なら0を許容する。

`V2XBehaviorOutput`から`MpcProblem`へattempt IDを渡し、planningとtrackingの
ログを同じIDで照合する。

### 2. candidate rejectを構造化

`GapPlannerOutput`へ次を追加する。

- `reject_gate`
- `reject_index`
- `reject_distance_m`
- `free_interval_count`

既存の人間向け`reject_reason`は維持し、Decision Traceに両方を出す。

### 3. primary失効時のalternate評価

候補評価を小さな局所関数へまとめる。primary sideが±1で確定しており、
primaryがbridge失効またはtracking backoffの場合だけ、反対側を1回評価する。
alternate採用条件は従来どおり以下をすべて要求する。

- planner active / feasible
- reachable bridge feasible
- 要求sideと解決sideが一致
- solver backoffなし

primary planner自体がsideを確定できない場合に左右を総当たりする変更は、同期
計算量を増やすため今回は行わない。

### 4. runtime failover trace

dynamic Mission waitごとに次をchange-awareで記録する。

- mission episode / generation / target / phase / trigger
- current・alternate候補のfeasible、mission有無、stable、fresh
- cross-side allowed / no-return / hard fault
- forward prefixの有無
- resolverのaction / reason

ログはcategorical change時と5秒heartbeatだけ出し、40 Hzで重複させない。

### 5. Mission失効後のurgent alternate admission

active Missionが既に失効し、通常のalternate Missionが完全生成・物理検証済み
にもかかわらずside debounceだけが未完了の場合、no-return前に限ってdebounceを
省略する。target continuity、position jump、予測sweep、実車体非重複、動的回廊、
cross-side回数の各gateは省略しない。

## 非対象

- MPCC重み、horizon、速度capの調整
- Recovery/Reverseの変更
- no-return後の反対側切替
- incomplete prefixを完全Missionとして採用する変更
