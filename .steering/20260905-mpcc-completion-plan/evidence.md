# 現状確認の根拠

2026-09-05にsource、git履歴、保存ログ、YAML、過去のtest XMLを読み取り確認した。
今回solver replay、build、走行は再実行していない。
以下のsource行番号はHEAD `a8b968ac` 時点。`P`は
`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/` を表す。

## 完了済みと残作業

| 項目 | 確認結果 | 根拠 |
|---|---|---|
| 通常定式化の統一 | normal契約はseven-stateのみ許可 | `P/src/mpcc_execution_contract.cpp:342` |
| 状態・入力 | 状態はey、lag、heading、v、progress、command steering、response steering。入力はacceleration、steering rate、virtual progress speed | `P/include/multi_purpose_mpc_ros/mpcc_rate_resolved.hpp:15` |
| 旧normal authority削除 | 構造Slice 6は完了記録あり | `docs/spec/mpc-integration.md:2712`。six-state記述は当時の履歴、現行seven-stateは同文書2606行以降 |
| Returnのworker分離 | 8/31の個別受入れ済み。52.743 msで結果生成し、一連の追い越しを完遂 | [results](../20260831-return-worker-isolation/results.md)、run `20260831-151543/d1` |
| 低速Stopのstationary suffix | 修正・個別受入れ済み | commits `60061025`, `a06889ce`、[results](../20260831-stop-stationary-trajectory-contract/results.md) |
| 最終normal authority喪失のsnapshot | 観測実装・採取済み | `79ff9f88`、[results](../20260831-normal-authority-loss-snapshot/results.md) |
| FollowをStayBehindへ変える仮説 | 比較完了、単独の修正案として棄却 | HEAD `a8b968ac`、[results](../20260831-follow-dynamic-topology-audit/results.md) |
| horizon/solve cadence調整 | N=16、N=18、20 Hzの試験は棄却、採用変更なしで閉鎖 | [Slice 7](../20260828-slice7-closure/tasklist.md)、`docs/spec/mpcc-experiment-governance.md:59` |
| dev3と統合完了 | 旧統合tasklistに未完が残る。後続HEADの全intent・全車両受入れは未確認 | [tasklist](../20260827-slice6-integration-acceptance/tasklist.md) |

8/31の各局所修正で成功したrunには、別episodeのwall/corridor failureが残る。
個別修正の成功を、現HEADのレース全体合格に読み替えない。

## 凍結したFollow境界

- run: `output/20260831-192221`、Domain 1。
- runは `79ff9f88` の観測Sliceの動的証拠として記録されている。
  run直下にbuild commitのmanifestはなく、バイナリの厳密な同一性は再受入れ時に固定する。
- 最後の直前正常join: decision `1186`、wall-clockログ時刻 `1788171774.408509379`。
- 今回対象のFollow喪失: decision `1187`、`1788171774.433358975`。
- observation time: `13.734999692 s`、control origin: `13.864999692 s`。
- interaction fingerprint: `13608911548693048044`。
- target: `d2`、observation generation: `272`。
- log: `output/20260831-192221/d1/autoware.log:483` と484行。

decision 1186では提案Followが未認証でもCruiseのcurrent-world joinが成立していた。
1187でCruiseは`continuation-rejected`となり、Followも`intent-mismatch`で、
`no-current-world-authority`から外部Emergency Stopへ移る。
同runには先行するTrack decision 737の喪失もあるため、1187をrun全体の最初の異常とは呼ばない。

期待挙動は、同じworldで安全な通常軌道が存在する場面において、current-world証明を得た
Followまたは適合する通常軌道を継続できること。実際は線形化した動的障害物QPが棄却される。
過去監査ではA/B/C/DとStayBehindが失敗し、warm-start入力の非線形再積分は
wall/dynamic/terminal証明を通過した。これは今回の再実行結果ではない。

## 今回確認した時間軸の不整合

| 保存値・計算 | 値 |
|---|---:|
| semantic input全20 stageのdt | 0.25 s |
| publication interval | 0.025 s |
| target progressのstage差 | 約0.0973530 m |
| stage差 / semantic dt | 0.3894121 m/s |
| stage差 / publication interval | 3.8941214 m/s |
| ReplayWorld速度を近傍course headingへ投影 | 3.8813〜3.8941 m/s |

末段・初段間は19区間なので、速度の計算には19 × 0.25 sを用いた。
約10倍の差は、publisher周期で生成したtubeをsemantic stageへ対応付けた仮説と整合する。

対応するsource境界:

1. `P/src/mpc_controller_cpp.cpp:21575`: progress実行activationと通常intentのmetadata要求を分離。
2. 同`:21712`: semantic向け`progress_stage_dt_sec`はprogress linearizationから取得。
3. 同`:21812`: `dynamics_stage_dt_sec`は`model->Ts`で初期化し、
   `progress_contouring_active`時だけprogress dtへ更新。
4. 同`:22305`: target tubeのarrival timeは後者を累積して構築。
5. 同`:23178`: semantic requestには前者のdtを格納。
6. 同`:24564`: `build_current_overtake_target_tube()`は渡されたarrival timeから予測する。

**Confirmed:** 保存されたQP用tubeとsemantic時間の数値不整合、および別々の時間列を作るsource構造。
**Hypothesis:** この時間列分岐がdecision 1187の認証可能な通常軌道を失わせた最初のproducer欠陥。
時間軸を揃えた比較で認証が回復するまでは、原因修正の成立を断定しない。

`P/src/mpcc_stateless_maneuver.cpp:100`のtarget horizon検証はidentity、generation、
サイズ、finite値を検査するが、stage時刻の同一性を検査していない。これが検出漏れの候補。
`docs/spec/mpc-integration.md:3298`は、以前の有限course window再投影がtargetを
窓端へ固定した欠陥と、その重複projector削除を記録している。
世界座標からの再構築を試す場合も、この棄却済みproducerを復活させない。

## 因果関係の分類

| 分類 | 現時点の判定 |
|---|---|
| Root cause候補 | target tubeとsemantic stage時間を別ownerが作る不整合。比較による確定待ち |
| Contributing cause | 通常intentでも旧組立経路の時間列とcanonical metadataが併存。非同期遅延は別途計測が必要 |
| Mask / 後段処理 | 1186の前intent Cruise継続は表出を遅らせるが、認証済み継続自体は必要。自動削除対象ではない |
| Detection gap候補 | tubeのidentity・個数検査だけでは時間列の意味の違いを検出できない |
| Recovery behavior | 1187の外部Emergency Stopは未認証指令を拒否する結果。Recovery一般の閾値変更は根本修正にならない |

## 証拠ファイルの保全

リポジトリ相対パス:

- S1: `output/20260831-192221/d1/mpcc_architecture_snapshots/000000001187-bcdc90f34f8f6aec-follow-side-neutral-physical-proof-normal-authority-unavailable/snapshot.yaml`
- S2: `aichallenge/mpcc_architecture_snapshots/000000001187-bcdc90f34f8f6aec-follow-side-neutral-dynamic-obstacle-refinement-solve-rejected/snapshot.yaml`

| ファイル | SHA-256 |
|---|---|
| S1 | `909f23b096e9fba696c0ce935d341bb069d452ff86a50ccd803fe4bc6b8679c4` |
| S2 | `e5c1b0646eb603e28a94a8d856816fbdb5f16605403631af85e33f2bee9de2da` |
| 両ディレクトリのwall-grid.bin | `f32d3bb9f6ac8aebc03689aa7d70e6779d13de8a4000251f2f5c34106576de98` |

S1とS2の`source`全体、interaction fingerprint、wall binaryは一致を確認した。
S1は`exact_qp_available=false`。S2にはexact QPとwarm-start primal/dualがある。
非線形warm-start oracleをS1だけで実行する計画にはしない。
S2は未追跡であり、後続作業の前に両payloadを参照する保全manifestを作る。

## 現作業を誤認しやすい点

- `git status --short`の既存差分は結果JSON 2件と、crash、snapshot、symlink manifest群。
  制御source/configに未コミット差分はなかった。
- `output/latest/d1`のlog/MCAPは`e2e-preliminary-video-seed2037`、resultは
  `20260902-e2e-submission-freeze-single`。D2〜D4も異なるrunのリンクが混在。
  最新リンクだけでMPCC HEADの完走・安全ゲート合否を判断できない。
- `.steering/20260901-e2e-authority-timeseries-audit/`は`__pycache__`のみで、
  現時点に計画・報告sourceがない。MPCCの正式な再開地点としない。
- 中央registryはsnapshot 12件、experiment 48件。末尾は8/31 ShiftOut/Pass handoffで、
  decision 1187の最新監査は未登録。実験から参照される未登録snapshot IDも7件ある。
- `/aichallenge/workspace/build`と`/aichallenge/build`には別時点の成果物がある。
  正規buildスクリプトは`/aichallenge/workspace`へ移動してbuildする。
- 保存XMLではworkspace側architecture comparisonが34/34成功、旧build側は31/31。
  過去報告の2272 testsと、今回見たpackage XMLの集計範囲は異なる。
  新しいHEADのtest合格として再利用せず、後続でbuild/install/test対象を統一する。
- 読み取り専用の`docker ps`は成功し、稼働中コンテナは0件だった。
  通常sandboxからsocketには接続できず、読み取り権限の昇格で確認した。

## 今回の検証結果

- `git diff --check`: 成功。既存差分は保持し、新規変更は本計画の4文書のみ。
- Pythonによる相対リンク13件、根拠source/testパス、末尾空白、code fence確認: 成功。
- YAML比較によるS1/S2のsourceとfingerprint一致、S2のwarm-start availability確認: 成功。
- 記載したsnapshot/wallのSHA-256再照合: 成功。
- build、package tests、solver replay、AWSIM走行、提出評価: 今回は計画作成のため未実施。
