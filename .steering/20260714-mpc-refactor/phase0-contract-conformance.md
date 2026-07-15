# Phase 0b Contract Conformance 証跡

- 実施日: 2026-07-15
- scope: Contract Conformance候補（E01〜E06）+ Phase 0 supporting baseline/tooling/documentation（E07〜E08と証跡）。Safety laneなし
- 状態: **未commitの候補実装。local static と sealed eval identity は RED=0。Phase 0 全体は未完了**
- production code anchor: `50e0de5dcd3861565c78c501a6f3db0ca8e9489d`
- 修正前HEAD: `caf3599d9bad42b3cc88d91cebd2fa1fcc44b0ee`
- current branch/HEAD: `feat/20260714-mpc-refactor` / `caf3599d9bad42b3cc88d91cebd2fa1fcc44b0ee` + dirty working tree
- oracle SHA-256: `bdd6a8affd40019332b311e85046014232b635b657afaa90540c4dff1b54d3dd`
- 方針: endpoint / type / Domain / result schema は変更せず、実装を既存契約へ適合させる

`50e0de5` から `caf3599` までの差分は `.steering/20260714-mpc-refactor/` の計画文書だけであり、production code は同一である。したがって、前者をコードの基準点、後者を修正直前の計画HEADとして区別する。

## 1. 実施順序と判定範囲

承認済み設計は Phase 0a black-box capture、observation seam、Phase 0b remediation の順を要求する。一方、今回の実行依頼を受け、静的に確定できた Contract Conformance RED の修正を先に実施した。このため、**変更前productionのlive evidenceは取得しておらず、現行挙動の凍結完了とは判定しない**。必要な比較は production code anchor の隔離環境から再取得する。

production behaviorの変更E01〜E06はContract Conformance候補である。E07〜E08、oracle、fixture、manifest、証跡文書はPhase 0を再現・検証するsupporting baseline/tooling/documentationであり、Contract laneそのものとは分けて扱う。`docs/interface/participant-interface.md` も実装経路とupstream identityの記録を更新しているため、working tree全体を「契約文書を変更しないContract laneのみ」とは扱わない。ただし、endpoint、type、Domain、schema、control methodの契約値は変更していない。

R-13を含むSafety laneは実装していない。Contract Conformance、supporting変更、Safety laneの独立commit/review、clean post-remediation anchor、live baseline取得は未完了である。評価基盤の `aichallenge_system/` には差分を入れていない。また、Phase 0aより先にremediationした順序差は記録済みだが、承認済み計画の例外としては未承認である。

## 2. 同一oracleによるbefore / after

現在のoracleを、修正前HEADの `git archive` 隔離展開と現在のworking treeに適用した。作業treeを巻き戻さず、両方を同じ判定実装で評価している。

tool identity、anchor、判定ID、集計値は `phase0-oracle-results.json` に機械可読形式でも保存した。

| 対象 | PASS | RED | NEEDS_RUNTIME |
|---|---:|---:|---:|
| 修正前HEAD | 7 | 7 | 5 |
| 修正後local、sealed manifest登録前 | 14 | 0 | 5 |
| 修正後local、sealed manifest照合後 | 15 | 0 | 4 |

実行例:

```bash
python3 .steering/20260714-mpc-refactor/tools/phase0_contract_oracle.py \
  --repo-root /tmp/mpc-phase0-before.waC5at
python3 .steering/20260714-mpc-refactor/tools/phase0_contract_oracle.py
```

最終 `PASS=15 / RED=0 / NEEDS_RUNTIME=4` の4件は次である。

- `R-DDS-GRAPH`: Domain 0..4のcanonical live graph、type、direction、QoS、owner
- `R-SOLE-PUBLISHER`: 選択したcontrol methodにおける `/control/command/control_cmd` の唯一publisher
- `R-TOPIC-RATE`: control / odometry / trajectoryの実測rate
- `R-ARTIFACTS`: 完走runのresult JSON、`output/latest/`、UID/GID

終了コード0はstatic REDとsealed identity REDが0であることを示すだけで、Phase 0やContract/Safety Floorの完了を意味しない。result schemaのnegative testは、正常fixtureだけPASSし、empty/duplicate vehicle、legacy field不一致、detailsのfile/vehicle不一致、penalty集計不一致の5破損fixtureがすべてREDになることを確認した。

## 3. Intentional delta

| ID | 修正前 | 修正後 | 契約への影響 |
|---|---|---|---|
| E01 | canonical `control_method` に非契約 `rl_train` が存在 | 契約済み5方式だけを公開 | 既存5値と既定 `mpc` を維持 |
| E02 | submit launchからDomain 1→0 reset bridgeへ到達可能 | submit wrapperを削除し、tool packageは非提出の開発用として隔離 | V2X以外のcanonical cross-domain経路を除去 |
| E03 | Joy-Conがlegacy statusを読み無条件にBoost pulseを送信 | `/awsim/status` のexact 7要素、finite、fresh、残数、active、confirmation、session境界をguardでfail-closed判定 | `/awsim/status`、`/awsim/cmd`、型、pulse形式を維持 |
| E04 | Joy-Con reset buttonがDomain 0 resetと固定initial poseを送信 | legacy reset/initialposeを非公式内部topicへ隔離しwarning-only | `/admin/awsim/reset` ownerと `/set_initial_pose` handshakeを維持 |
| E05 | dev/gate runnerにDomain 0 state managerの起動・監視がない | runnerがmanagerをDomain 0で監視し、evalはupstream evaluation launchをownerとして二重起動しない | `/admin/awsim/start` ownerを一つに維持 |
| E06 | dev/gate AWSIMのCWDがrun directoryではない | `LOG_DIR`を絶対化しAWSIMのCWDに設定 | result名/schemaを変えず所定run directoryへ出力 |
| E07 | eval buildがupstream branch先端へ依存 | upstream SHAを固定し `/aichallenge/.upstream-commit` を保存 | 外部endpoint変更なし。source identityの追跡性を追加 |
| E08 | submit tarにPython cacheが混入し得る | cache/build生成物を除外 | 最上位 `aichallenge_submit/` と提出契約を維持 |

E01〜E06はContract Conformance候補、E07〜E08はPhase 0 supporting baseline/tooling変更である。E03は新しい安全閾値を導入するSafety laneではなく、既存Boost契約を非同期status下でもfail-closedで守るContract enforcementとして分類した。外部endpoint、型、Domain、high→low pulseは変更していない。

`run_simulator.bash` はAWSIMとmanagerを別process groupで起動し、親が両PIDを監視する。外部INT/TERMは両groupへ転送し、先行終了・SIGKILL・TERM無応答のいずれでも残存groupをbounded TERM→KILLで停止してstatusを回収する。

## 4. Build / test / sealed verification

### Identity

| 項目 | 値 |
|---|---|
| dev image ID | `sha256:1c850655ff19086a783fe00fb8c77cc6eda5249575a7979119edfb69e32aadfc` |
| dev image created | `2026-07-10T07:43:25.517014439+09:00` |
| ROS / compiler | Humble / `g++ 11.4.0` |
| OSQP | `0.6.2` / `ros-humble-osqp-vendor 0.2.0-1jammy.20250718.232948` |
| `libosqp.so` SHA-256 | `bd0e9a39bb1a39e327d35579aea2d3cb7ab48fb2b5cbf6c736d3c1d880c02150` |
| built `mpc_controller_cpp` SHA-256 | `330f7956ee2933b4f41c343a66195ebf61f25f63f704159588be2adaa9979995` |
| eval image ID | `sha256:65189d6d538414a053355dfd83d78843bc216228521703713d589aad231076a5` |
| eval image created / size | `2026-07-15T10:04:46.028227255+09:00` / `19879003935` bytes |
| pinned upstream | `6124702b2f0eb364bf921b8fa827a092806ed1d1` |
| submit tar SHA-256 | `0d3b3399a93aed10363175577bdb23fcf4a44126fe8eae9e7bde20da972f6dbd` |
| eval build log / SHA-256 | `output/docker/20260715-093625-docker_build-332082.log` / `929789da3aa3f26d6a0ce505536cff8ca7391b6bc0f0cd2250531989ace911b3` |

sealed imageの完全なidentity、base image、AWSIM tree、host/image source hash、upstream source hashは `sealed-eval-manifest.json` に保存した。base tag、apt index、top-level pip、rosdep indexはmutableであるため、これは今回完成したimageの固定証跡であり、bit-reproducible rebuildの主張ではない。

MPC source/configはこのlaneで変更していない。

| 対象 | SHA-256 |
|---|---|
| `mpc_controller_cpp.cpp` | `f84b8659caad84d3e8f2ade3bed69ac1cde2e986f0f0fb43a20bab1a90228ff9` |
| `config/config.yaml` | `312af3289241cf28ae1d36df701dd10f1461a0d5c13d64ce8e38a9a5c30b8c61` |

### Verification result

| 検証 | 結果 |
|---|---|
| local `make autoware-build` | PASS、26 packages、4.32秒 |
| no-cache eval image build | PASS、26 packages、19分49秒 |
| Joycon guard unit test | host / dev / sealedで各7/7 PASS |
| supervisor isolation test | 8/8 PASS。sim/manager失敗、clean early exit、group cleanup、TERM無応答KILL、eval external owner、外部TERM、manager SIGKILLを確認 |
| result schema negative test | 6/6 PASS（正常1 + 破損5） |
| canonical launch `--show-args` | PASS。submit/reference/Joycon branchを解決し、control methodは5値 |
| sealed Joycon DDS proxy | `/awsim/cmd` はguardだけがpublish。raw/ignored endpointはguard内で終端し、`/admin/awsim/reset` はgraphに不在 |
| Boost正常status | exact 7要素で `[1.0]` → `[0.0]` を各1回publish |
| Boost不正status | 8要素をrejectし、公式commandは0件 |
| submit archive | 381 entries、最上位は `aichallenge_submit/` のみ、cacheと `aichallenge_system/` は0件、guard同梱 |
| Python compile / XML parse / `bash -n` / `git diff --check` | PASS |
| existing MPC package test | 15/16 target PASS、1 KNOWN_RED |

既存MPC testのREDは `PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory` である。fixtureが重複終端を期待する一方、現行 `traj_mincurv.csv` は459行で重複終端を持たない。MPC source/CSVは本laneで変更していないため今回の回帰ではないが、B-01は未完了のままとする。

sealed proxyはJoycon変更surfaceのDDS配線を検証したもので、実AWSIMを含むcanonical graphやartifact検証の代用にはしない。一時containerはすべて削除済みである。

## 5. 未完了 / blocker

### Live runtime

次は未取得である。

- Domain 0 / 1..4のcanonical endpoint、type、direction、QoS、owner
- 選択control methodでの `/control/command/control_cmd` sole publisher
- control / odometry / trajectoryの実測rate
- 完走runのresult JSON、`output/latest/`、UID/GID

B-08のsubmit tar作成とeval image buildまでは完了したが、Safety Floor未成立のため実AWSIMを起動する `make eval` は実施していない。B-02〜B-07、B-09も未実施である。

### R-13 Safety lane

R-13は未実装である。2026公式資料からcontrol commandの角度単位と加速度入力は確認できるが、final steering slewのauthoritative limit、fault/Recovery時のsafe-stop加速度、gear contextは確定できていない。現行のgain適用後steeringは理論上48 degまで到達し得るため、raw configから閾値を推測してPASSにはしない。

- Simulator specification: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/simulator.html>
- Interface specification: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html>
- SW class rules: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html>

H-01〜H-08のsignal、単位、authoritative criterion、観測方法を確定し、Safety laneを独立実装・検証・reviewするまでContract/Safety Floorは成立しない。さらにContract Conformanceとsupporting変更の独立commit/review、順序差の承認、clean anchor、残るlive evidenceが必要であり、**Phase 0完了およびPhase 1開始は保留**とする。
