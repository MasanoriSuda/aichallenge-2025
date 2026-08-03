# Design

## 1. Approach

全面GMPCC化を一度に行わず、現行MPCの入力をGMPCCに近づける。

```text
existing mission path (target_ey/target_epsi)
  + track/static-wall corridor
  + predicted target footprint
  + longitudinal progress/rear-clear objective
  + horizon speed reference
  -> one committed overtake horizon
  -> existing MPC
```

FSMはmissionの所有権、異常時の中止、Return完了に限定する。横軌道と縦速度を
別々のルールで毎周期上書きしない。

## 2. Stage 0: baseline freeze

先に`.steering/20260803-pass-horizon-safe-separation`の動的項目を確認する。
この結果をGMPCC参考変更のbaselineとし、未確認のSafeSeparation修正へ別の性能変更を
重ねない。

収集対象は次とする。

- Pass開始・Return完了・Recovery数
- SafeSeparation開始/終了/timeout
- rear-clear extension結果
- target別の追い越し所要時間
- Pass中の速度・加速度・front cap owner

## 3. Stage 1: horizon progress evaluation

既存の`OvertakeLineHorizonEvaluation`を拡張し、左右候補ごとに次を計算する。

- time-indexed `target_ey` / `target_epsi`
- time-indexed target footprintとの分離
- predicted rear-clear time/distance
- horizon終端での相対前後位置
- horizon内最低速度
- 必要closing speedと加速度
- wall clearance、横加速度、actual footprint余裕

候補は「横位置へ到達できる」だけでは採用しない。rear-clearへ進む候補を優先し、
終端でも相手後方に残る候補はFollow継続または延長計画とする。

初期scoreは設定可能な純粋関数とする。

```text
score = forward_progress
      + rear_clear_progress
      + retained_speed
      - lateral_motion
      - steering_change
      - wall_risk
      - predicted_overlap_risk
```

hard constraint違反はscore低下ではなく棄却する。

## 4. Stage 2: committed horizon speed ownership

採用時に横mission pathだけでなく、同じgenerationの速度ホライズンを保存する。

- ShiftOut: 保護車間を割らない範囲でclosingを作る
- Pass: rear-clearへ必要な進捗を維持する
- Return: 通常レーシングライン速度へ連続的に戻す

committed horizonがfreshかつ実行可能な間は、この速度計画が通常Follow capより優先する。
ただし次は常に上位とする。

- Emergency / actual body overlap
- locked target以外の新しい前方車
- target jump / observation timeout
- static wall physical infeasible
- solver failure

グローバル`a_max`は変更せず、既存上限内で不要な負加速度と速度capの重複を除く。

## 5. Stage 3: opponent-aware MPC corridor adapter

Stage 1/2で不十分な場合だけ、相手予測footprintをside-specific convex corridorへ変換し、
既存MPCの`lb/ub`へ時系列で渡す。左右に分割された非凸空間を一つのsolveへ直接入れず、
commit前に左右候補を評価し、採用側の凸corridorだけをsolveする。

これによりsolver全面置換を避けながら、GMPCCの「相手と壁を同じホライズンで扱う」性質を
近似する。

## 6. Stage 4: full GMPCC decision gate

次を満たさない場合に限り、別controllerとしてGMPCC prototypeを検討する。

- 低速NPCのPass完遂率が目標に届かない
- corridorは成立するが速度仲裁でrear-clearできない
- 複数車両でside-specific convex化が頻繁に不成立
- 現行solverへ追加した制約が計算予算を超える

prototypeは現行topic契約を維持し、`control_method`互換の実験モードとして分離する。

## 7. Separate follow-up: recovery

暫定1位のmulti-stroke K-turnは有望だが、本変更へ混ぜない。別steeringで次を扱う。

- 高加速指令なのに低速が3秒継続するpush-stuck検出
- heading errorに応じた短い前後切替
- 固定Reverse距離ではなく姿勢改善で早期終了
- 後方車両と壁footprintによる各strokeの実行guard

## 8. Kill switches and logging

各段階は独立した設定で無効化できるようにする。

- `v2x_overtake_horizon_progress_enabled`
- `v2x_overtake_horizon_speed_plan_enabled`
- `v2x_overtake_opponent_corridor_enabled`

ログはmission開始/更新/終了時のevent logを中心とし、毎周期出力しない。
候補score、rear-clear予測、速度owner、棄却理由を同一eventに含める。
