# E2E NPC teacher feasibility / student gate requirements

## Objective

LiDAR-only TinyLidarNet が他車両を動的障害物として扱えるかを、MPC 教師と同一の
AWSIM world で比較できる再現可能な gate を作る。runtime NPCはV2Xを発行しないため、
V2Xを持つ低速peerを教師用動的障害物とし、runtime NPCは学生Acceptanceで評価する。
教師が回避に成功した走行だけを学習候補とし、単車 baseline を退行させない。

## Scope

- 3 vehicle固定world、3 laps、domain 3 egoの教師 / 学生 pair
- 教師と学生で配置、peer controller、周回数、衝突設定を一致
- domain 1/2はMPC、domain 2は既存設定で低速、domain 3だけMPC/Tinyを切替
- 学生は LiDAR だけを model input とし、V2X / IMU topicを購読しない
- candidate checkpoint は production weight を上書きせず A/B 可能にする
- run-level provenance を維持した dataset 抽出

## Non-goals

- NPC データが成立する前の production weight 差し替え
- GNSS / IMU / V2X を student feature に追加
- 4 vehicle final gate
- 失敗した student command を教師 label として学習

## Definition of Done

1. `make e2e-peer-audit-mpc` と `make e2e-peer-audit-student` が同一world契約を持つ。
2. domain 3 teacher run が低速peerを含む 3 laps を完走し、bag にLiDAR/controlがある。
3. student run は `/scan` 以外の禁止入力を controller が購読しない。
4. NPC dataset は run identity、scenario、label source を追跡できる。
5. candidate は単車3 lapsを退行させず、peerおよびruntime NPC gateで改善する。

## Outcome

同一world契約とstudent acceptanceは成立したが、Definition of Done 2の教師admissionは
不成立だった。失敗したMPC runを教師として扱わないため、3〜5は次の障害物policy Sliceへ
持ち越す。本Sliceの成果は再現可能なnegative evidenceと監査gateであり、NPC教師dataset
ではない。
