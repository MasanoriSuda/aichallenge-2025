# Design

## 現象

`output/20260721-094005/d2/autoware.log`では、停止前車が3.27 m、速度0.00 m/s、
左側clearance 5.09 mで検出されているにもかかわらず`low_speed=0`だった。
通常`Overtake`へ一度遷移した直後に`overtake hard curve blocked`で`Follow`へ戻り、
横移動が開始されなかった。

## 原因

- `v2x_low_speed_avoidance_min_prepare_distance=6.0 m`により近距離候補が除外される。
- `OvertakeLine`は、通常`Overtake`が選択済みでも停止前車を見ただけで
  「停止車両回避が所有する」と判断してlineを解除する。
- 停止車両候補の入口が通常の狭いfront-overlap判定に依存し、カーブや横ずれで
  早期候補から外れる場合がある。
- MPC初期化の`START!`直後からV2X判定が動き、AWSIMの実レース`Start`前に
  `LowSpeedAvoidance`のstall timeoutとcooldownを消費していた。

## 方針

1. 停止車両候補を共通コース進捗と走行回廊で別途収集する。
2. 候補距離の下限を3.0 mへ変更する。開始後は既存のclearance holdにより
   3.0 m未満でも状態を維持する。
3. `OvertakeLine`の所有権判定を純粋関数化する。
   `LowSpeedAvoidance`またはその候補が所有している場合はlineを解除するが、
   通常`Overtake`が選択済みなら停止前車だけを理由に解除しない。
4. generic front-risk/SafetyBrakeの正面重複判定は変更しない。
5. 広い回廊の候補だけが見えて通過gapが無い場合は、その候補を`Follow`の前車として
   採用し、距離・速度に応じた減速を行う。スタート猶予中は新規回避を開始しない。
6. AWSIM状態追跡中はレース`Start`までV2X挙動を休止し、`Start`で状態を初期化する。

## 影響範囲

- `mpc_controller_cpp.cpp`: 停止車両候補収集、状態入口、line所有権。
- `v2x_overtake_core.*`: 候補・所有権の純粋判定。
- `test_v2x_overtake_core.cpp`: 距離境界と所有権回帰テスト。
- `config.yaml`: シミュレーション向け近距離入口。
- `docs/spec/mpc-integration.md`: 2025 AWSIM向け暫定仕様。
