# Design

## 1. no-return 前の動的 side rescue

従来は次の条件を side replan の入口で要求していた。

```text
current body separated
AND prediction valid
AND current-side predicted sweep separated
```

これを次へ変更する。

```text
current body separated
AND prediction valid
```

現在側の predicted sweep が重複している場合は、現在 Mission を infeasible として扱う。反対側については既存の full Mission assessment を使い、成立した場合だけ debounce 後に frozen Mission を一括置換する。予測重複そのものを理由に反対側評価を止めない。

予測自体が無効、現在車体が重複、no-return 到達後の場合は従来どおり切り替えない。

## 2. side-by-side forward completion admission

forward completion の開始・継続条件へ以下を追加する。

- predicted body footprint sweep が非重複
- `target_s + rear_clear_distance` を埋めるための相対閉速度が正
- 推定される自車前進距離が safe-separation のローカル距離上限以下

概算は次式を用いる。

```text
forward_speed = min(v_max, max(ego_speed, target_speed + speed_delta))
closing_speed = forward_speed - target_speed
relative_distance = max(0, target_s + rear_clear_distance)
required_forward_distance = forward_speed * relative_distance / closing_speed
```

この判定は「抜き切れる見込みがない並走状態で前進固定し、相手と壁に挟まれる」事象を抑える。条件を失った場合は既存 SafeSeparation の同じ側での減速・再評価へ戻し、反対側へ瞬時に横断しない。

## 3. entry pre-arm の縦速度整合

横 Mission が選択済みでも実測相対速度が entry gate に届かない間はbase line上で加速する。この参照速度に常時最大closing speedを使わず、選択済みMissionの`closing_speed_mps`を使う。Missionがない、または値が不正な場合だけ従来の最大値へfallbackする。

## 4. ログとテスト

- forward completion の resolution に距離成立フラグと必要前進距離を保持する。
- debug log で必要距離を確認できるようにする。
- pure policy test で境界を固定し、controller の判断条件と乖離しないようにする。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure policy の入力・判定
- `mpc_controller_cpp.cpp`: 現在側予測重複を rescue assessment へ接続、走行状態を admission へ入力
- `test_v2x_overtake_core.cpp`: policy test
- `docs/spec/mpc-integration.md`: 挙動の永続仕様
