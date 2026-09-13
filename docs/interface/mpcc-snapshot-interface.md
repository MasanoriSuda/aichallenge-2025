# MPCC監査snapshotの数値初期化policy

対象は内部の`mpcc-architecture-failure-snapshot/v3`に含む
`source.semantic_request.initial_tangent_policy`。ROS topic、提出tar、評価JSONの契約は変更しない。

| 値 | 数値初期化 | 対応 |
|---|---|---|
| 0 | CurrentSteering | 既存記録を維持 |
| 1 | ReferenceSteeringWithRestLaunch | 既存記録を維持 |
| 2 | SteeringBeforeDrive | Rejoin用数値初期化 |

policyは入力候補の数値的な意味であり、実行指令や認証ではない。各policyでも元のQP入力は
自由変数で、同じ状態・入力制約と物理証明を必要とする。source fingerprintにpolicyを含め、
異なるpolicy間でwarm startを流用しない。

読み取り側は未対応値を拒否する。値2の記録は対応したビルドで再生し、旧ビルドが値0へ
読み替えることは認めない。新しい読み取り側は既存の値0/1を引き続き受け入れる。
既存v3の構造を保つenum拡張であり、版名だけで新policyへの対応を推測せず、記録のcommit・
ソースhash・ビルドを固定する。R753で値1/2のround-trip・指紋改変・未知値拒否とwarm start分離を検証済み。

## 現在の壁区間の診断記録

2026-09-13: single-r30/1263とsingle-r31/2163の完全一致再生を終え、
一時的なruntime writerを削除した。以下のv1/v2は保存済み成果物の読取契約として残す。
既存v3、ROS、提出・評価の契約は変更しない。実境界1263は回帰テストで維持する。

`mpcc-current-wall-interval-observation/v1`は通常送信後の最初の壁区間拒否を保存する
独立した内部診断。`authority=false`で、既存v3入力・ROS・提出・評価契約は変更しない。
原地図のバイナリと指紋、時刻・判断ID、原姿勢・参照姿勢・投影と採用lag、元の横幅・
clearance・sampling anchor、連結した空き区間、選択結果とpreferred containmentを含む。
最大32区間を保存し、超過時は`complete=false`。不完全・未知schemaの記録から
完全再生や走行認証を主張しない。読み取り側はこのschemaを明示して既存v3と区別する。

`mpcc-current-wall-interval-observation/v2`は通常移動前も含む初回の実クエリ拒否へ
観測範囲を拡張する。v1の3つのflatな`prior_moving_*`フィールドに代えて、
`prior_moving`を未観測時はnull、観測済みなら`decision_id`・`pose_sec`・
`velocity_mps`のmappingとする。先行判断IDは正で現在ID未満、時刻と速度は有限。
v1をv2として読み替えず、旧記録は元の必須先行観測を持つv1として再生する。
地図・クエリ・区間の形式、32区間の上限、authority=falseは維持。
