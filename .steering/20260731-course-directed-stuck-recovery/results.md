# Results

## 変更結果

- `abs(e_y) > rejoin.max_lateral_error_m` のとき、rollout終端が現在よりコース外へ離れる候補を棄却する。
- 接触削減量が同じ場合、guard有効中だけコース中心へ近づく候補を優先する。
- 一時的な障害物による Reverse-first だけは、安全確認済みの前進復帰候補を評価できる。
- ソルバ由来、協調停止、確定済みReverse intentの後退必須条件は維持した。
- Recovery状態ログへ `course_guard`、`course_rejected`、`course_improvement` を追加した。

## 検証

- Docker内ビルド成功。
- `multi_purpose_mpc_ros` の675テストが全件成功。
- 実ログ相当の `e_y=-3.63 m` から外へ離れるケース、中心へ戻るケース、中心付近の互換性、厳密なReverse-only条件を単体テスト化した。

## 残確認

`make dev2` で壁接触を再現し、以下を確認する。

- `course_guard=1` のとき、選択候補の `course_improvement` が原則0以上になる。
- コース外向きReverseの反復と `maneuver_direction_unknown` の反復が再発しない。
- 静的地図またはV2Xで安全な復帰候補がない場合は、従来どおり停止を維持する。
