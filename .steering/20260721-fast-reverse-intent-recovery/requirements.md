# Requirements

## Purpose

2025 AWSIM `make dev3` の停止デッドロックで、前方障害物に詰まった車両が
Reverse候補を保持できずForwardへ切り替わる問題と、停止後の復帰開始が遅い問題を修正する。

## Functional requirements

- SIMレースでは、自車速度が1 km/h相当（0.28 m/s）以下で前進要求があり、前方停止車、
  前方・側方wall/contactのいずれかが確認された場合、Reverseを最初の復帰方向とする。
- AWSIM補正後に選んだReverse方向は、停止確認とclearance確認をまたいで保持する。
  操舵primitiveは現在poseに対して再検証し、古いrolloutをそのまま使用しない。
- 協調停止episodeはaggressive SIM設定でもReverse-onlyとし、後方車両がいる場合は停止して
  clearanceを待つ。後方が塞がれたことを理由にForwardへ切り替えない。
- 非協調のReverse-first復帰が静的障害物や接触悪化で明示的に失敗した場合だけ、次の
  aggressive retryでForward候補を解禁する。
- 停止検出からReverse gear要求までの固定待機を短縮する。

## Constraints

- `/control/command/control_cmd`、gear topic、V2X message、Domain契約は変更しない。
- `simulation_only: true` と既存のstatic/V2X swept-rollout gateを維持する。
- 実車設定へは適用しない。値は2025 AWSIM dev3向け暫定値とする。

