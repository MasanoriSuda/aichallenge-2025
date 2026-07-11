# V2X Overtake Stability Requirements

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Implemented / Runtime Verification Pending

## 目的

現行 C++ MPC の V2X 追い越しについて、次の2症状を同時に改善する。

1. 前方に通過可能な横方向 gap が見えていても `Follow` のまま追い越しを開始しない。
2. `Overtake` に入っても、横移動開始時または状態揺れにより速度を失い、追い越しを完了できない。

本作業では、追い越しの安全条件を単純に無効化しない。前方リスク、壁、曲率、MPC
feasibility を優先しつつ、候補判定、状態遷移、速度参照を分離して、追い越せる場面では
安定して追い越しを開始・継続・完了できるようにする。

Automotive AI Challenge 2026 の追い越し判定・安全余白・速度制限の公式合否条件は未確定である。
本書の数値は現行ローカル設定と 2026-07-11 のシミュレータログを基にした暫定値であり、
公式確定仕様として扱わない。

## 入力と関連ステアリング

- 実装本体: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- 現行設定: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`
- 最新ログ: `output/latest/d1/autoware.log`、`d2/autoware.log`、`d3/autoware.log`
- MPC 統合仕様: `docs/spec/mpc-integration.md`
- V2X gap planner: `../20260704-mpc-v2x-gap-planner/`
- V2X behavior FSM: `../20260705-mpc-v2x-behavior-fsm/`
- front risk arbitration: `../20260706-v2x-front-risk-arbitration/`
- overtake / recovery line: `../20260706-v2x-overtake-recovery-line/`

本作業は上記実装の置き換えではなく、runtime verification で判明した不具合と不足を修正する
後続ステアリングである。

## 調査結果

### Overtake に入らない主因

- `v2x_overtake_max_curvature=0.03 rad/m`、8 m先読み、カーブ3 m手前の開始禁止により、
  最新ログの周期サンプルでは約80〜87%が追い越し禁止状態だった。
- 曲率 zone が不許可の場合は gap planner を呼ばず、`plan_N=0` のまま `Follow` になる。
  D1 では `side_clear` が約2〜5 mでも `overtake start too close to curve` で拒否された。
- gap planner は後段の追い越し用0.8 m判定より先に、共通 `gap_min_width=1.8 m` で候補を除外する。
- `v2x_vehicle_vehicle_gap_enabled=false` と `v2x_multi_front_gap_enabled=false` により、
  2台の間および18 m以内に前方2台がいる場面は設定上追い越し対象外である。
- 最初に決めた pass side だけを探索し、反対側に連続 gap があっても再試行しない。

### Overtake 中に速度を失う主因

- D2 は実際に `Overtake` へ入ったが、`Idle -> ShiftOut` 直後に OSQP が失敗し、
  deceleration fallback が発動した。
- 同区間では V2X の `target_velocity_limit=inf` にもかかわらず、実速度が約2.76 m/sから
  約1.67 m/sへ低下した。前方車速度上限ではなく、MPC feasibility / 横目標側の問題である。
- OSQP の連続失敗ログは100周期ごとなので、1行のエラーでも複数周期 fallback していた可能性がある。
- `Overtake -> Cruise` の25 ms後に再び `Overtake` となった実例があるが、OvertakeLine は
  `Pass -> Return` に入った後、再取得しても `ShiftOut` / `Pass` へ復帰しない。
- gap 喪失で `Recovery` に入ると、`v2x_follow_speed_limit_enabled=false` でも
  `v2x_follow_velocity=3.0 m/s` が無条件上限として使われる。
- 追い越し専用の速度参照はなく、`v2x_overtake_front_velocity_limit_enabled=false` は
  前方車由来の上限を外すだけで、速度維持を指示しない。

### 車両別速度設定

- `domain_v_max` は D1/D2/D3 で10/20/40 km/hであり、追い越し中も同じ上限である。
- D2 の `domain_start_v_max=37 km/h` は通常上限20 km/hとの `min()` で適用されるため、
  現実装では速度を引き上げられない。
- start期間の計時はAWSIMの実際の`Start`ではなくMPC初期化時から始まる。最新D2ログでは
  MPC初期化からBoostが確認した`Start`まで約16秒あり、15秒のstart期間は走行開始前に期限切れとなる。
- trajectory CSV の `vx_mps/ax_mps2` は現行 runtime 速度参照へ使われず、毎周期の
  `effective_v_max` で全 waypoint の `v_ref` が上書きされる。

## 優先順位

制御判断は次の優先順位を守る。

1. odometry / solver fail-safe、`SafetyBrake`、非有限値拒否。
2. front risk emergency、車体・壁・path constraint、到達不能 gap。
3. 安全な Recovery / Return。
4. 追い越し継続と追い越し時の速度参照維持。
5. 通常 Cruise / Follow の速度最適化。

追い越し速度維持のために、1〜3の安全処理を無効化してはならない。

## 対象範囲

### 対象

- V2X front / side 判定の短時間欠落に対する hysteresis。
- 追い越し対象と pass side の短時間 lock。
- `Return` / `Recovery` 中に対象を再取得した場合の一貫した状態遷移。
- 左右両側の gap 候補評価と拒否理由の構造化。
- raw gap、wall margin適用後gap、guard required widthの単位・順序整理。
- curve entry禁止と追い越し継続禁止の分離。
- 追い越し中のソフトな速度参照と、Recovery専用速度上限。
- `domain_start_v_max` の意図どおりの適用。
- 追い越し開始時のMPC infeasible抑制と連続失敗診断。
- C++ unit test、設定回帰、dev2/dev3の実走証跡。
- package README と `docs/spec/mpc-integration.md` の更新。

### 対象外

- `/v2x/vehicle_positions`、`/control/command/control_cmd` など既存topic/message型の変更。
- `aichallenge_system/`、評価FSM、result JSON schemaの変更。
- 2026公式ルールとしての追い越し安全距離・速度の確定。
- Boostを追い越し戦略へ投入すること。現行のスタート時1回Boostは維持する。
- trajectory CSV の `vx_mps/ax_mps2` をruntime速度計画へ全面統合すること。
- 学習ベースplanner、全局race strategy、実車向け有効化。
- 安全確認なしに曲率ガード、車両膨張半径、壁marginを一括で無効化すること。

## 機能要求

### R-OBS: 計測と再現

- `R-OBS-01`: state、phase、対象vehicle ID、front/side判定、pass side、curve zone、
  左右gap、速度調停結果、solver連続失敗数を同一時系列で追跡できる。
- `R-OBS-02`: `reason` と `block` は少なくとも curve、gap width、prepare distance、gap time、
  lateral acceleration、multi-front policy、vehicle-vehicle policy、target lossを区別する。
- `R-OBS-03`: Overtake中の `desired_velocity`、最終上限、実速度、加速度指令をrosbagまたは
  周期debugで確認できる。
- `R-OBS-04`: solver失敗開始、継続周期数、復帰を明示し、100周期未満の連続失敗も判別できる。

### R-DET: 他車検出の安定化

- `R-DET-01`: 1周期のfront/side分類欠落だけで、追い越し対象をclear扱いにしない。
- `R-DET-02`: frontからside、sideからrearへの遷移は、同一対象の相対位置を使って連続的に扱う。
- `R-DET-03`: `Pass -> Return` は「検出されなかった」だけでなく、対象が自車後方へ抜け、
  所定clearanceを満たしたことを確認して決める。
- `R-DET-04`: V2X timeout、position jump、ID不整合時は対象lockを無期限保持せず、安全側へ遷移する。
- `R-DET-05`: 自車除外と他車ID一意性は現行の暫定V2X契約を前提とし、不明なIDを
  無条件に同一対象へ結合しない。

### R-GAP: Gap候補評価

- `R-GAP-01`: 左右両側を同じ膨張半径・wall margin・連続性条件で評価し、各側の結果を保持する。
- `R-GAP-02`: 追い越し開始前は第一候補が不成立なら反対側を評価する。`ShiftOut` 以降は
  安全上の中断を除きpass sideを頻繁に反転しない。
- `R-GAP-03`: raw free interval、wall margin適用後 interval、最終target clearanceを区別する。
- `R-GAP-04`: `gap_min_width` と `v2x_overtake_guard_min_gap_width` の適用順序を明文化し、
  後段設定が実質無効になる構成を避ける。
- `R-GAP-05`: vehicle-vehicle gapとmulti-front gapを不許可にする場合、単なる`no gap`ではなく
  policy拒否として記録する。
- `R-GAP-06`: covariance、prediction margin、車両半径を含む最終必要幅をログとテストで再現できる。
- `R-GAP-07`: 見た目の点在gapではなく、必要点数・距離にわたり連続するgapだけを実行候補にする。

### R-CURVE: 曲率zoneと実行許可

- `R-CURVE-01`: 「gap候補が存在するか」と「現在追い越しを実行してよいか」を別の判定として保持する。
- `R-CURVE-02`: curve zone中も候補gapと拒否理由は計算できるが、許可されない限りMPCへ反映しない。
- `R-CURVE-03`: 新規追い越しのentry条件と、既に横並びとなった追い越しのcontinue条件を分ける。
- `R-CURVE-04`: hard forbidden WP / hairpin、soft curvature lookahead、inner-curve passを区別する。
- `R-CURVE-05`: 曲率閾値を緩和する場合は、最初のヘアピン、壁clearance、最大操舵、横加速度を
  同時に検証し、単独のタイム改善だけで採用しない。

### R-FSM: 状態・フェーズ遷移

- `R-FSM-01`: `Overtake -> Cruise -> Overtake` の短時間揺れで `Pass` が不可逆に`Return`へ落ちない。
- `R-FSM-02`: `Return` / `Recovery` 中に同一対象を再取得した場合、再追い越し、Return継続、
  Recovery継続のどれを選ぶかを明示条件で決める。
- `R-FSM-03`: Return開始後の再追い越しには、対象、pass side、gap、curve、front riskを再確認する。
- `R-FSM-04`: target clear判定には時間または距離hysteresisを持たせる。
- `R-FSM-05`: EmergencyBrakeはphaseに関係なく優先し、解除後に古いtargetへ無条件復帰しない。
- `R-FSM-06`: phase遷移ごとに一意なreasonを出す。

### R-SPD: 追い越し速度

- `R-SPD-01`: Overtake用のdesired velocityを通常上限、進入時速度、前走車速度差から計算できる。
- `R-SPD-02`: Overtake desired velocityはMPCの参照値として扱い、危険時にも強制するhard lower boundにしない。
- `R-SPD-03`: global/domain/ref-velocity/curve/front-risk/SafetyBrakeの上限を超えない。
- `R-SPD-04`: gapが成立しfront riskが安全な直線では、Overtakeへ入ったこと自体を理由に
  参照速度をFollow速度へ下げない。
- `R-SPD-05`: Recovery速度は専用設定とし、`v2x_follow_velocity`を暗黙に流用しない。
- `R-SPD-06`: front risk、curve cap、MPC feasibilityにより減速した場合は、その制限元を記録する。
- `R-SPD-07`: Overtake終了後は通常速度参照へ連続的に戻し、1周期のstepを作らない。
- `R-SPD-08`: `a_max=1.0 m/s²`などの加速制約を守り、失速後の瞬間的な回復を要求しない。

### R-DOMAIN: 車両別速度

- `R-DOMAIN-01`: global hard maximum、通常domain maximum、start期間maximumを別値として保持する。
- `R-DOMAIN-02`: start期間maximumはglobal hard maximumを超えず、通常domain maximumより高い値を
  指定した場合は意図どおり一時的に引き上げられる。
- `R-DOMAIN-03`: start期間のepochはMPC初期化ではなく、公式`/awsim/state`の`Start`遷移または
  それと同じrace-session start判定を使う。Ready待機時間で期間を消費しない。
- `R-DOMAIN-04`: reset後の新しいrace sessionではepochを再初期化し、同一session中に再発火しない。
- `R-DOMAIN-05`: start期間終了後は通常domain maximumへ連続かつ決定的に戻る。
- `R-DOMAIN-06`: Overtake速度はdomain maximumを勝手に越えない。D1/D2の上限変更は明示設定で行う。

### R-MPC: Feasibilityとfail-safe

- `R-MPC-01`: `Idle/Follow -> ShiftOut` で横目標・制約を不連続に変更しない。
- `R-MPC-02`: MPCへ渡す前に、全horizonの`lb <= ub`、有限値、target範囲、必要横加速度を検査する。
- `R-MPC-03`: Overtake開始候補が事前検査に失敗した場合、OSQPへ不整合問題を渡さずFollowを維持する。
- `R-MPC-04`: OSQP失敗時の減速fallbackは維持する。
- `R-MPC-05`: 追い越し開始が原因の連続OSQP失敗を検出した場合は、phaseを安全にRecoveryへ倒し、
  同じ失敗targetを毎周期再投入しない。

### R-COMPAT: 互換性

- `R-COMPAT-01`: `control_method=mpc`、launch entry、既存topic/message型を維持する。
- `R-COMPAT-02`: V2X機能を無効にした場合は通常trajectory trackingを維持する。
- `R-COMPAT-03`: LowSpeedAvoidanceとgate2の停止車回避を通常Overtakeの変更から分離する。
- `R-COMPAT-04`: config key未指定でも安全な既定値で起動できる。
- `R-COMPAT-05`: D1/D2/D3で異なるtrajectoryと速度上限を使う現行機能を維持する。

## 非機能要求

- 判定coreをROS callbackとOSQP構築から分離し、synthetic入力でC++ unit testできること。
- 同じ入力、時刻、設定から同じstate、pass side、速度調停結果、reasonを返すこと。
- 制御周期内で左右候補を評価しても40 Hz deadlineへ有意な悪化を与えないこと。
- debug無効時に周期ログを増やさないこと。
- 実車向け既定値として有効化しないこと。
- `output/`、rosbag、result JSONを編集しないこと。

## 受け入れ条件

### 自動テスト

- straight上の前方1台・左gap、右gap、両側gap、no-gapを正しく分類する。
- 第一候補側が不成立で反対側が成立する場合、開始前だけ反対側を選べる。
- 1〜数周期のfront欠落では`Pass -> Return`へ遷移しない。
- 同一対象を再取得した場合、Return中の再追い越し規則が期待どおり動く。
- curve entry禁止とcontinue条件が別々に検証される。
- Recovery速度が専用設定を使い、Follow速度設定の変更に引きずられない。
- Overtake desired velocityがSafetyBrake、front risk、curve、domain hard capを上書きしない。
- D2のstart maximumが通常20 km/hより高く、global 40 km/h以下なら期間中に反映される。
- Readyで15秒以上待機しても、AWSIM `Start`後から所定期間だけD2 start maximumが有効になる。
- config key省略時と機能無効時の互換テストが通る。

### ビルド・静的検証

- `make autoware-build` が成功する。
- 対象packageの`colcon test`が成功する。
- `/v2x/vehicle_positions`と`/control/command/control_cmd`の契約差分がない。
- config/documentationの単位と既定値が一致する。

### シミュレータ検証

- dev2の安全な直線シナリオで、十分なgapがある場合にOvertakeへ入り、対象を抜いてReturnを完了する。
- 追い越し開始からReturn完了まで、理由不明のOSQP fallbackが発生しない。
- 安全な直線追い越しで、Overtake状態そのものによる3.0 m/s上限が掛からない。
- dev3で前方2台、左右並走、片側閉塞を再現し、policy拒否と実gap不足を区別できる。
- 最初のヘアピンでは、無理な内側追い越し、壁接触、SafetyBrake抑制が発生しない。
- gate2の停止車回避とgate3相当の車線維持を悪化させない。
- control command、odometry、V2X、state/phaseを記録し、速度低下の制限元を後から説明できる。

## 未確定事項

- 2026公式環境におけるV2X vehicle IDの一意性と自車ID取得方法。
- target vehicle lockをIDだけで行えるか、位置連続性fallbackが必要か。
- front/side/rear hysteresisの暫定時間・距離。
- curve thresholdをtrack共通値にするか、hard forbidden WPと併用するか。
- Overtake desired velocityを「進入時速度維持」「前車速度+advantage」「domain cap」のどれで決めるか。
- Recovery時の専用速度上限と解除条件。
- vehicle-vehicle gapとmulti-front gapを本番戦略で許可するか。
- runtime計測用debug topicを追加せず、ログ/rosbag既存topicだけで十分か。
