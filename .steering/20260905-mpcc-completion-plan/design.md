# 完遂までの順序と判断基準

最新方針: [2026-09-09の更新計画](../20260909-mpcc-actuation-application/completion-plan.md)
を優先する。指令加速度と車体加速度の意味、および実適用時刻の不整合が判明したため、
当初の「構造移行のやり直しは不要」という判断を再検討する。以降のP0〜P2は初期計画の
記録であり、完了済み監査を最初から繰り返さない。通常走行・追い越し・停止復帰の
統合受入れと提出物評価というP3〜P4の完了条件は維持する。
現在のsourceと根拠は [evidence.md](evidence.md)、全体の完了条件は [requirements.md](requirements.md)。

## 実施順序

| 段階 | 作業 | 次へ進む条件 |
|---|---|---|
| P0 証拠・実行基準の固定 | 最新監査、snapshot、build/install、実験台帳を揃える | 対象バイナリと固定入力を再現できる |
| P1 時間軸の比較監査 | decision 1187で元tube／semantic時間へ整合したtubeを比較 | producer欠陥と修正前に失敗するtestが確定 |
| P2 原因箇所の修正 | 確定した時間ownerを統一し重複生成経路を削除 | 同一snapshotの認証成功、回帰test/build合格 |
| P3 統合品質の受入れ | 全intent、遷移、非同期経路、多車両、長時間走行 | 安全・identity・時間制約と網羅条件を充足 |
| P4 提出と文書の閉鎖 | 固定tar.gzからeval、仕様・台帳・tasklist更新 | 提出物由来の合格証拠と最終報告が揃う |

P1が反証された場合、P2の時間軸修正を自動実施せず、後述の比較分岐で
原因を分類し直す。この計画は実装開始や制御変更の実施記録ではない。

## P0: 証拠と実行基準を固定

1. `a8b968ac`、local/cloud config、reference/map、Compose設定、image ID、
   起動コマンド、seed/初期配置をrun manifestへ記録する。
2. S1（実行権限喪失）とS2（exact QP/warm start）を一組として参照するmanifestを作る。
   SHA-256とsource一致を保存し、生成物はコミットに混ぜない。
3. 最新Follow監査のaccepted/rejected/inconclusiveと再検討条件を中央registryへ登録する。
   参照される未登録snapshot IDを補完し、後期8/31の局所修正も台帳と照合する。
   観測結果と仮説の採否を区別し、根拠不足はinconclusiveにする。
4. `make autoware-build`後、`/aichallenge/workspace/install`の実体を確認する。
   `/aichallenge/build`やE2E残存installを誤って評価しない。既存成果物は無断削除しない。
5. standalone比較の基準結果とwarm-start oracleを再実行する。
   出力は新しい監査runディレクトリへ保存し、元snapshotを上書きしない。

比較コマンド（正規build済みコンテナ内、`<...>`はevidence.mdの実在ファイルへ置換）:

```bash
ros2 run multi_purpose_mpc_ros mpcc_architecture_compare <S1のsnapshot.yaml>
ros2 run multi_purpose_mpc_ros mpcc_architecture_compare <S2のsnapshot.yaml> --warm-start-primal-physical-nonlinear-oracle
```

## P1: target tubeとsemantic時間の比較監査

修復対象の不変条件:

> QPの障害物stage kと、そのstageの物理証明は、同じ観測epoch、control origin、
> cumulative semantic time、同じtarget運動仮定を参照する。

最初に以下を一表へ出す。publisher dtやstage indexだけを時刻の代用にしない。

- observation time、control-origin delay、stage dt、累積予測時刻
- target ID/generation、元の位置・速度・加速度と予測horizon/打切り条件
- 元tubeのprogress/lateral、semantic時間で生成したprogress/lateral
- 使用course frame、循環経路のprogress lift、finite window境界
- 該当constraint row、残差、exact timed proofとの最初の相違

比較は同じS1/S2 world、車体、wall、solver設定、hard constraints、terminal意味を固定し、
候補で変えたtubeの内容に新しいcandidate fingerprintを付ける。
world/sourceの対応は保持するが、問題内容を変えて元problem fingerprintを使い回さない。

主比較は、既存のcanonical target producerが使う運動仮定を維持して
sampling時刻だけをsemantic時間へ合わせる。加速度モデルや座標投影も異なる場合は
別armに分ける。ReplayWorldからの直接再構築は独立の照合用とし、有限course window端へ
targetを固定した過去のprojectorをproductionへ復活させない。

| 比較結果 | 判断・次の作業 |
|---|---|
| 時間だけの整合でQPからterminalまで認証成功 | 時間owner欠陥を確定しP2へ |
| 時間整合後もQP失敗、exact nonlinear oracle成功 | dynamics線形化・障害物row生成のmodel/certificate境界を監査 |
| 新しい入力のQPは成功、exact proofは失敗 | 時刻origin、加速度、projection、coverageの差を特定。productionへ昇格しない |
| 元の失敗を再現できない | build/input/provenanceを再確認しP0へ |
| A/B/C/Dが全失敗 | 有界な不可能性証明がなければUnknown。局所optimizer失敗を物理的不可能と扱わない |

成果物はcomparison表、修正前に失敗するdeterministic test/replay、producerのfile:line、
置換して削除する経路、合否・rollback条件。これを揃えて次の実装単位を具体化する。

## P2: 比較で確定したproducerを修正

時間owner欠陥が確定した場合の候補範囲:

| 対象 | 方針 |
|---|---|
| `src/mpc_controller_cpp.cpp` | semantic時間を確定してからtarget tubeを生成。normalのtubeにpublisher dtが残る経路を削除 |
| 時間契約の小さなhelper/型 | originとstage timeの単一生成・照合を可能にする。追加する場合も通常authorityを増やさない |
| `src/v2x_overtake_core.cpp` | arrival time受渡しの意味が不整合な場合に限定して修正 |
| `src/mpcc_stateless_maneuver.cpp`とsnapshot契約 | consumerがsampling identityを検査。必要なprovenanceのみ追加し旧snapshot読取を検証 |
| 対応testと`test_single_authority_source_contract.py` | 時間契約とproducer配線、normal authority不変を回帰確認 |

上記はpackage内の相対パス。実際の差分はP1で確定した最小範囲に絞る。
新normal authority=0、runtime切替flag=0、fallback/lease/retry追加=0を目標にする。
消すのは誤った時間生成の重複経路であり、Emergency、正当なretained再検証や物理guardではない。

先に固定する回帰ケース:

- dt=0.25 sとpublisher=0.025 s、一定速度targetでの距離差。
- 可変stage dt、control-origin delay=0.13 s、同一observationの扱い。
- Follow/Cruise/ReturnとShiftOut/Passの両target-tube入力経路。
- 停止target、加減速target、予測範囲外、循環seam、course window終端。
- wrong-generation、horizon/time列不一致、旧snapshot schemaの扱い。

修正前test失敗→producer修正→不要な経路削除→focused tests→package tests→build→
固定replay→限定dev2の順に進める。比較moduleと単体helperだけの合格で閉じず、
本番のproblem組立がその時間列を使うことまで確認する。

rollback基準は新しいidentity違反、未認証publish、壁/車両接触、停止証明の回帰、
継続callback overrun。戻し先は各実装単位の直前の受入れ済みcommitを記録する。
最初の比較前HEADは`a8b968ac`だが、これをレース品質合格commitとは呼ばない。

## P3: 統合受入れを一つの基準で閉じる

最初に限定dev2で1187系の再発と修正効果を検証し、その後に以下を実施する。
正式な受入れcampaignは同じcommit/config。途中で修正した場合、影響する列を再実行する。

| シナリオ | 主な観測 | 完了条件 |
|---|---|---|
| 単車Track/Cruise・六周 | 直線、ヘアピン、循環seam、速度、wall、callback | 全周完了、安全・identity違反0 |
| dev2 Follow | 移動/低速/停止先行車、Cruise→Follow→Cruise | 不要なEmergencyを発生させず認証済み指令で追従・離脱 |
| dev2追い越し・六周 | 左右、ShiftOut→Pass→Return→Idle、進入拒否 | 実行可能な左右各episodeの完遂と、安全な拒否の両方を確認 |
| Hold/Stopと再発進 | 低速stationary suffix、terminal stop、障害解消 | 無認証再加速・停止証明喪失0 |
| async/provenance | 左右完了順逆転、target更新、intent変更、non-normal割込み | stale/wrong-generation採用0、指令と証明のidentity一致 |
| Recovery/Rejoin注入 | 接触/stuck/gearの原因、復帰後normal authority | upstream decisionを保持し、通常制御へ安全に復帰 |
| dev3・六周 | 非target車の割込み、複数遭遇、V2X generation | 全車の結果・証明・時間制約が合格 |
| dev4 | 4 Domain起動、V2X、負荷、各車指令 | 4台対応範囲の走行を通し、起動確認だけで完了扱いにしない |
| `make gate1/2/3` | 停止、NPC追い越し、車線維持のローカル判定 | 対応するローカル結果がすべて合格 |

通常走行campaignは同じ初期条件で最低3回を提案する。seed指定が使える場合は
固定seed再現と別seedを分ける。回数とシナリオは開始前に固定し、最良runだけを選ばない。
六周は既存validation-plan由来の開発基準で、公式提出のrequired lapsを変更する意味ではない。
gate番号と公式2026課題の完全対応はTBDのまま維持する。

計測は問題組立、worker待ち/実行、solver、物理証明、callback、publish間隔の
mean/p95/p99/maxと連続超過数を取得する。40 Hzの25 msはcallback周期であり、
background worker全件へ25 ms完了を要求するものではない。worker結果が実際の
published predecessorへ間に合い、正しくjoinできたかも記録する。

全通常publishについてsolution/certificate/fingerprint、操舵・加減速の同一起源を検査する。
既存ログに全周期の情報がなければ、低負荷な観測を別の実装単位として追加する。
throttleされた警告が0件であることだけから、不正publishが0件とは断定しない。

残るShiftOut wall/corridor、terminal viability、長い計算遅延はそれぞれ最初の失敗を
別snapshotに固定する。同一failure familyへの第3パッチ前、またはsolver/proof不一致等の
package AGENTS triggerでA/B/C/D比較へ戻る。方式比較で採用・棄却を決め、
resume/timeout/retryの追加を積み重ねない。

## P4: 提出物と文書を閉じる

```bash
make autoware-build
# コンテナ内の /aichallenge/workspace で実行
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
# 以下はリポジトリrootで実行
./create_submit_file.bash
./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz
make eval
```

tar.gzのSHA-256、最上位`aichallenge_submit/`、entry launch、eval image ID、
local/cloud設定差を固定する。evalがそのimage/提出物を使用したことを確認し、
完走・penalty・safety結果を同じrunのJSON/log/MCAPで照合する。
`output/latest`の混在リンクをこの照合に使用しない。

最後に`docs/spec/mpc-integration.md`、実験registry、元の統合tasklistへ
現行構造・全intent網羅・全run結果・残る暫定仕様を反映する。
旧six-state/5x3等の歴史記述やconfigコメントは、現行説明と履歴を明確に分ける。
重複資料を増やして完了状態が分散しないよう、この計画は統合完遂の索引として閉じる。
