# Race V2X Follow And Overtake Requirements

## Background

Automotive AI Challenge 2026 のレース本番は複数台同時走行になる。現行実装では Gate1 の V2X 停止と Gate2 の V2X 追い越しを個別に通すためのロジックを `multi_purpose_mpc_ros` に追加している。

次の段階では、Gate2 専用挙動をそのまま本番レースへ広げるのではなく、2 台同時走行で以下を順番に確認する。

1. 先行車を検出して安全に追走できる。
2. 先行車が遅く、左右と壁の余裕があるときだけ追い越しへ入れる。
3. 追い越し後に通常ラインへ復帰できる。
4. 2 台が交互に追い越す状況でも、同時に同じ空間へ入り込まない。

2026-07-01 時点の運営回答により、障害物・他車両情報の正入力は `/v2x/vehicle_positions` のみを使用する。

## Goal

2 台同時走行で、V2X 上の先行車に対して「追走」「追い越し」「復帰」を安全に切り替える race behavior を作る。

最初の合格ライン:

- 自車前方の同一走行コリドー内にいる車両を primary target として選べる。
- 追い越し不可の間は停止ではなく追走速度 cap で距離を維持できる。
- 追い越し可能な直線・広い区間では Gate2 由来の lateral behavior を使って追い越せる。
- 自車が先行側になった場合は、後続車の接近と追い越し意思を見て速度を少し下げる yield behavior を持つ。
- 横並び、後方接近、壁距離不足、V2X stale では追い越しや復帰を許可しない。
- 追い越し判断の理由を log / rosbag から説明できる。

## Scope

主に変更してよい対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
- `aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/`
- `Makefile` または `aichallenge/run_autoware.bash` の race 検証用起動オプション
- `docs/spec/v2x-multivehicle.md`
- `docs/spec/mpc-integration.md`

変更しない対象:

- `aichallenge/workspace/src/aichallenge_system/`
- `/v2x/vehicle_positions` の topic 名、message 型、fanout 設計
- `/control/command/control_cmd`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- result JSON schema
- Gate1 / Gate2 の既存合格挙動

## Functional Requirements

### R1. V2X Input

- race behavior の他車両認識は `/v2x/vehicle_positions` のみを正入力にする。
- `vehicle_id`、`position`、`header.stamp` を使う。
- 相対速度は `V2XVehicleTracker` の有限差分推定を使う。
- 自車 echo、NaN / Inf、座標ジャンプ、stale sample は判断から除外する。
- V2X が一時欠損しても、直前の target lock と速度 cap を短時間維持して危険な急操作を避ける。

### R2. Target Model

- 同一走行コリドー内の最近前方車を `primary_front_target` とする。
- 左右の追い越し候補コリドー内にいる車両を `side_blocker` とする。
- 復帰先の後方または横並びにいる車両を `return_blocker` とする。
- 後方接近車両は、追い越し開始時よりも復帰許可判定で重く見る。
- target は `vehicle_id` で lock し、短時間で別 target へ頻繁に切り替えない。

### R3. Follow Behavior

- 追い越し不可の先行車には距離ベースで追走する。
- desired gap は `min_follow_gap_m` と `time_headway_sec * ego_v_mps` の大きい方にする。
- 目標速度は先行車速度、closing speed、gap error から決める。
- 危険距離では Gate1 の `V2XStopPlanner` による停止・減速へ倒す。
- 追走中は lateral offset を 0 に戻し、不要な横寄せをしない。

### R4. Overtake Permission

以下をすべて満たす場合だけ追い越しへ入る。

- 先行車が自車より十分遅い、または gap が縮み続けている。
- `gap_m` が race 用の start gap 範囲内にある。
- 追い越し側の V2X corridor に `side_blocker` がいない。
- 追い越し側の壁距離が `min_wall_clearance_m` 以上ある。
- 復帰予定ラインの前方と後方に十分な余裕がある。
- MPC の操舵角、操舵レート、速度 cap 内で横移動できる。
- cooldown 中ではない。

追い越し不可なら、停止ではなく follow を優先する。ただし危険距離では stop fallback を優先する。

### R5. Overtake Behavior

- 追い越し開始後は side を固定し、走行中に左右を切り替えない。
- side が unsafe になった場合は反対側へ切り替えず abort / follow / stop に倒す。
- lateral offset は Gate2 と同じ MPC path constraints bias を初期案として使う。
- Gate2 専用の強い補助操舵は race では既定無効または低めにし、必要時だけ検証する。
- 複数先行車が列になっている場合は、同じ side が safe なら列を抜け切ってから復帰する。

### R6. Return Behavior

- 追い越し target より `return_clearance_m` 以上前方へ出たら復帰候補にする。
- 復帰先の前方、横、後方に `return_blocker` がいない場合だけ復帰する。
- 復帰中は後続車が近い場合に急なライン変更をしない。
- 復帰完了判定は offset 指令値ではなく、実際の path lateral error `ey` を使う。

### R7. Yield Behavior

同じアルゴリズムの車両を同時走行させる場合、先行側が常に全開だと後続側が安全な追い越し機会を作れない。race behavior は、自車が先行側になった場合の協調的な yield を持つ。

- 後方車が同一走行コリドー内で `yield_detect_range_m` 以内にいる場合、後方接近 target として認識する。
- 後方車との gap が `yield_start_gap_m` 以下、または後方車の closing speed が `yield_min_closing_speed_mps` 以上の場合、先行側は `yield_speed_cap_kmph` まで速度 cap を下げる。
- 後方車が左右どちらかの追い越し corridor に入った場合、先行側はその side へ寄らず、中央または現在ラインを維持する。
- 抜かれ中は急加速しない。後方車が `yield_release_gap_m` 以上前方へ抜けるまで yield を維持する。
- yield は無制限に続けない。`yield_timeout_sec` を超えても後方車が追い越せない場合は通常 follow / clear へ戻す。
- yield 中でも前方危険距離では Gate1 stop fallback を優先する。

### R8. Alternating Overtake

2 台が交互に抜き合う試走では、以下を満たす。

- 追い越し中の車両をもう一方が同時に同じ side から抜きに行かない。
- 横並び状態では両車とも follow / hold side を優先し、急な復帰や切り込みをしない。
- 追い越された車両は、直後に同じ空間へ復帰する車両へ突っ込まない。
- 先行側は後続車の追い越し機会を作るため、必要な範囲で yield speed cap を使う。
- 追い越し完了後に cooldown を入れ、即座の再追い越し振動を避ける。
- 交互追い越しの成否は lap time より collision / wall contact / unstable steering の有無を優先して判定する。

### R9. Observability

最低限、以下を throttled log に出す。

- race behavior state
- selected target `vehicle_id`
- target role: front / side blocker / return blocker
- gap, signed gap, lateral offset
- target speed, relative speed
- selected side
- wall clearance
- follow speed cap
- yield speed cap
- overtake speed cap
- cooldown remaining
- permit / deny / abort / return-blocked reason

確認 topic:

- `/v2x/vehicle_positions`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- `/control/command/control_cmd`
- `/mpc/ref_path`
- `/mpc/prediction`

## Non-Goals

- 3〜4 台同時走行の最適戦略は初期対象外。まず 2 台で成立させる。
- ブロッキング、接触前提のライン取り、威嚇的な幅寄せは扱わない。
- 画像、LiDAR、CSV 障害物は race behavior の正入力にしない。
- 評価基盤や result JSON schema は変更しない。
- 実車向け高速度追い越しは、シミュレータ 2 台試走後に別途扱う。

## Definition Of Done

- `make autoware-build` が通る。
- race 用の pure Python 判断ロジックに単体テストがある。
- `make gate1` と `make gate2` が退行しない。
- `make dev2` または race 用 2 台起動で `/v2x/vehicle_positions` が両車から見える。
- 2 台同時走行で、片方が先行車を追走できる。
- 2 台同時走行で、安全な機会だけ追い越しへ入れる。
- 先行側が後方接近を検出し、必要な範囲で yield speed cap を適用できる。
- 追い越し後に通常ラインへ復帰できる。
- 交互追い越し試走で、同時横移動、接触、壁接触、制御スパイクがない。
- `output/latest/d<N>/autoware.log` と rosbag から判断根拠を追える。
