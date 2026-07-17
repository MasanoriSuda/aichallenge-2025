# V2X共通進捗・停止デッドロック正式修正 Requirements

作成日: 2026-07-17
状態: Implemented（dev3検証済み、深いwall接触のfail-closed制約を記録）

## 目的

速度差を付けた3台走行で、ヘアピン中の前走車検出が遅れて接触距離まで詰まる問題と、
接触後に先頭車がMPC fallbackへ固定されて後続車もSafetyBrake停止を続ける問題を解消する。

## Baseline evidence

対象run: `output/20260717-082215`

- P1/P2/P3は全車`env/final_ver3/traj_mincurv_org.csv`を使用した。
- P1は`1784244358.306660933`からOSQP連続失敗へ入り、停止後も回復しなかった。
- P2は終了時にP1を約1.92 m前方へ検出し、`SafetyBrake`で停止していた。
- 現行前方判定は自車直近waypointの接線座標だけを使うため、ヘアピンでコースに沿った
  前後関係を早期に判定できない。
- `stuck_recovery`は安全審査付き実装が存在するが、P1/P2/P3すべてで無効だった。

## 機能要件

### R-PROGRESS-01: 共通コース進捗

- 自車とV2X車両を同じreference pathへ射影し、経路に沿った前方距離を求める。
- circular pathの終端を跨ぐ前方車を検出できる。
- 射影は設定されたlook-behind/look-ahead範囲だけを探索し、遠いヘアピン枝を誤採用しない。
- V2X速度は採用した経路接線方向へ射影し、相対速度・停止距離判定へ渡す。

### R-PROGRESS-02: 早期検出

- 共通進捗による前方検出距離を設定可能にし、現行の停止距離・Follow距離より短くしない。
- 横方向が走行corridor外の車両は前方車にしない。
- 射影が無効な場合は、その車両を共通進捗の前方車として採用しない。
- side vehicle判定は実空間の近接判定を維持する。

### R-DEADLOCK-01: 安全条件付き解除

- SIMのP1/P2/P3で既存`stuck_recovery`を有効化する。
- Follow/SafetyBrakeなど前方車に対する意図的停止はrecovery対象外のまま維持する。
- 先頭車に前方車がなく、前進意図、停止継続、solver fallbackまたは接触証拠を満たした場合だけ
  recoveryへ進める。
- 後退・前進rolloutはoccupancy map、footprint、完全なV2X観測、Boost停止、gear確認を通過した
  場合だけ実行する。
- V2Xの受信freshnessとsource stampは異なるclock domain内で検証し、両clockを直接減算しない。
- 接触離脱は0.40 m単位で停止・再評価し、接触悪化または単step時間上限でも再評価へ戻す。
- 後退側に停止車がいる場合は、静的・V2X双方で安全なForward Straight / Left / Rightから、
  heading errorを減らす候補を選択できる。

### R-PREVENT-01: 追突予防

- 前方車の後ろにいる間はmoving-front速度上限と相対速度ベースSafetyBrakeを有効にする。
- 前方5.0 mへ入る前に横移動を完了できない追い越しを開始しない。
- 安全な横追い越しが成立した後は前方車速度への上限を解除できる。

### R-LOG-01: 診断

- V2X debugへ共通進捗採用有無、局所接線距離、経路横偏差を出す。
- recoveryの既存state/reason logを維持する。

## 非機能要件

- `/control/command/control_cmd`、`/v2x/vehicle_positions`、odometry、trajectoryの名前・型を変えない。
- `aichallenge_system/`、Domain構成、評価JSONを変更しない。
- 実車では`simulation_only=true`によりrecoveryをfail-closedとする。
- 既存のSafetyBrake、EmergencyBrake、wall/vehicle clearanceを弱めない。

## 受け入れ条件

1. 直線・ヘアピン・circular終端で共通進捗の前方距離をunit testできる。
2. 後方車や探索範囲外の別枝を前方車として採用しない。
3. P1/P2/P3でSIM限定stuck recoveryが起動設定になる。
4. 意図的なSafetyBrake車両はrecoveryしない既存優先順位を維持する。
5. `test_v2x_overtake_core`、`make autoware-build`が成功する。
6. `make dev3`で前走車への連続追突から3台が停止列になる現象が再発しないことをログで確認する。
   個別のwall接触で安全なrolloutがなくSafeStopした場合は、無理に解除せず残課題として記録する。
