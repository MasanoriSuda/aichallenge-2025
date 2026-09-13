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
