# Design

## 現状の問題

コントローラは `OvertakeMissionCandidate` とローカルな `CandidateMetadata` を同じ順序で別々の `vector` に追加し、選択後に同じ index で再結合している。さらに `SideAssessment` と `V2XBehaviorOutput` では shift distance、closing speed、deadline 判定を別フィールドへ分解して渡している。

この構造では候補軸を追加した際に、一方だけ追加・並べ替えされると、選択した候補と preflight 結果が食い違う可能性がある。

## 方針

### 1. 候補を自己完結させる

`OvertakeMissionCandidate` に以下を追加する。

- `pass_side_sign`: 候補を生成した側
- `current_position_clear`: 候補 corridor に現在位置が入っているか

preflight から選択後に必要な `goal_lateral_m`、`shift_distance_m`、`max_required_lateral_accel_mps2` はすでに候補に存在するため、別メタデータを持たない。

### 2. 選択結果を分解しない

`SideAssessment` と `V2XBehaviorOutput` は、選択済みの `OvertakeMissionCandidate` を `optional` で保持する。ログや side quality に必要な値は、この候補から参照する。

### 3. mission freeze を一か所へ集約する

Behavior が返した候補を `OvertakeLineState` の frozen mission へ反映する処理をメンバ関数へまとめる。fallback 値と clamp は現行と同一にする。

## 今回行わないこと

- 左右の全候補を一括ランキングする変更
- 現在速度・加減速遅れを含む時系列 rollout
- deadline slack による新しい順位付け
- Recovery 再取得時の mission 再構築ルール変更
- パラメータの攻撃化

これらは本リファクタ後の性能修正ステアリングで扱う。

