# E2E Teacher Collection Design

## Controller Selection

`AIC_CONTROL_METHOD`をDocker環境へ渡し、`run_autoware.bash`で許可値を検証した後、
`aichallenge_system.launch.xml`の`control_method`引数へ変換する。system launchは同じ引数を
submit launchへ渡す。production entryのdefaultはTinyLidarNetから変更しない。

## Scenario Pair

| target | controller | LiDAR | GNSS | IMU | purpose |
|---|---|---:|---:|---:|---|
| `e2e-single` | TinyLidarNet | on | on | off | student closed-loop gate |
| `e2e-teacher` | MPC | on | on | on | teacher label collection |

IMU差はMPC localizationを成立させるためのinfrastructure差であり、student feature差ではない。
コース、start、車両数、NPC、lap数、collision設定は一致させる。

## Evidence

run logで`control_method: mpc`、rosbag metadataでLiDAR/control topicとmessage count、resultで
3周完了を確認する。抽出時は`--label-source mpc`を必須とする。
