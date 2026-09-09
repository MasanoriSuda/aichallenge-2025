# 現在のMPCCから提出評価までの完遂計画

2026-09-09、調査基準`ae2efe6a`、制御基準`1c4f377e`。
今回の計画具体化・比較観測は完了。**MPCC全体の実装・統合受入れは未完。**
自律実装・検証・必要分のlocal commitの既存承認を引き継ぐ。pushはしない。
旧計画の固定bias拡張案は[今回の比較](results.md)で棄却した。

## 完了の定義と現在地

単車だけの完走や局所test合格で終了しない。最終的に、同じcommit/configと
提出tarからの実行について、全intent・多車両・ローカルgate・評価JSONを確認する。
normalの解・軌道・proof・最終指令は一つの不変identityを保つ。

| 状態 | 根拠・扱い |
|---|---|
| Astra向けハーネス整理 | ローカルcommit済み。AGENTS/Skillsとreasoning方針の変更済み。品質/コストの比較実験は未実施 |
| 時刻・座標・proof provenanceの局所修正 | 保存済み。26package build/2381記録は1c4f377eの証拠 |
| 単車 | 通常baseline6周254.074秒/penalty0。追加観測6周253.054秒は診断用 |
| dev2 | baseline930、直接入力観測933のmoving Emergencyを保存。現在も不合格 |
| 縦モデル | wire=netの不一致を再現。全horizonのflat補正は停止端で棄却。採用モデル未確定 |
| 多車両・全動作・提出 | 以下の依存順で実行。過去runを新モデルの合格に数えない |

## M1: 採用できる物理入力・時刻モデルを確定

最初の不変条件は、同じwire入力と観測初期状態がprefix、QP、非線形proof、
Stop、retained、publishで同じ物理応答を意味すること。
現行`src/mpcc_rate_resolved.cpp:189`付近の速度積分はwireを直接微分に使い、
同`:309`付近は速度のJacobianを強制的にaffineへ戻す。片側だけ直さない。
以下のpackage相対パスはすべて`multi_purpose_mpc_ros`内。

**固定済み入力**: native rest r2、観測単車の学習15〜90秒/保留90〜250秒、
別dev2の最初のEmergency前、元371の932Accepted→933rejected Request。
未来実測やEmergency後の入力変化を過去decisionへ混ぜない。

| 比較案 | 現時点の判断 | 次の扱い |
|---|---|---|
| H0 wire=net | 不一致を再現 | 比較baselineとして固定 |
| H1 observer残差を全horizonで保持 | 無入力静止と制動→加速で破綻 | 棄却。条件を変えない再試行はしない |
| H2 rolling/dragを明示 | 移動区間の一部は改善、接地・横力を未表現 | 単独採用しない。componentとして検証 |
| H3 wheel forceと横運動を明示 | 実測中点を使うfitは改善 | 未来実測なしの予測比較が必要。係数の本番転記は禁止 |

次の有界作業は、入力選択→force→body速度の診断oracleを作り、H2/H3の
因果的な予測を同じ現在状態・公開済み指令で比較すること。計測が必要なら、
元binaryを保持した短時間診断でforce使用epoch、body frameのv/ω、接地を
同じFixedUpdateに固定する。今回の2点probeやEuler角速度を厳密な同時刻状態と扱わない。
残るモデル誤差と観測不能な接地・横状態を明示し、平均誤差の改善だけで採用しない。

productionから利用できるのは現行ROS入力であり、私有Unity状態・真の適用traceを
制御へ流用しない。七状態で表現できる縮約モデルと、横速度/実yaw等を明示する
状態拡張を同じworld・入力境界で比較する。状態拡張が必要なら、根拠と入出力契約を
先に記録し、旧七状態を第二normal authorityとして残さない。

入力時刻も別に閉じる。最新値選択では全packetの順次適用は保証されない。
公開、受信、選択、適用、次のphysics stepを区別し、0.13秒を縦入力の確定遅延と
しない。実適用traceは比較用oracleに限り、本番の予測は因果的な公開履歴と
観測から組み立てる。不確かな区間を一つのphaseへ決め打ちしてproofしない。
実測最大183.331msは保証上限ではない。適用/モデル誤差を有界にできない場合は
未確定として残し、証明基準を緩めてモデル採用しない。

**M1完了条件**: 停止・加減速・旋回・可変dt・接地変化・指令上書きを含む
固定入力で、予測誤差と適用不確かさの扱いを説明できる一案を選択する。
事前に固定した保留評価を満たす根拠、失敗native/replay、変更producer、削除経路、
rollback commitを一組にする。候補全失敗ならUnknownと記録し、物理的不可能と断定しない。
モデル誤差の扱い・取得方法は比較前に定義し、速度だけでなく位置/yawと停止端の
proofまで同じ誤差範囲を使う。平均誤差だけを合格基準にしない。既に閲覧した単車保留窓は
方式比較に使い、最終採用の確認は候補を固定した後の独立runで行う。

## M2: 共通モデル・不変artifactへ一つの実装単位で移行

M1を満たした案だけ実装する。gain、delay、margin、solver設定を同時に調整しない。

| 対象 | 必要な変更と消す不整合 |
|---|---|
| `mpc_longitudinal_prediction.*` / `mpc_state_prediction.*` | 公開履歴と実応答の意味を分離し、prefix限定の別モデルを共通モデルへ置換 |
| `mpcc_rate_resolved.*` / `mpcc_rate_resolved_problem.*` | 同じ非線形transitionと接線を使う。速度の旧affine上書きを採用モデルに従って置換 |
| `mpcc_rate_resolved_physical_adapter.*` / `mpcc_rate_resolved_physical_wall.*` | 同じモデル値・入力履歴・初期状態でphysical rolloutと壁/peer sweepを証明 |
| `mpcc_rate_resolved_execution_artifact.*` / `mpcc_rate_resolved_execution_source.*` | model/schema/係数/推定epoch/不確かさの意味をsourceへ封印。可変global参照を除去 |
| `mpcc_rate_resolved_retained_revalidation.*` / Stop生成経路 | complete Stop、source-free successor、rest/再発進を同じ意味に統一。旧wire=netの別積分を削除 |
| `mpcc_execution_contract.*` / `mpcc_architecture_snapshot.*` | context fingerprint、snapshot読取、async/warm-start互換判定を更新。旧schemaは診断のみ |
| `mpc_controller_cpp.cpp` | ROS観測→不変入力の組立とworker clone、最終serializeの一回変換を接続 |

wire加速度の上下限とnet body加速度の上下限を混同しない。停止時に補償だけで
再加速しない。入力/速度制約はQPとproofの同じ境界で扱い、publish後clampで隠さない。
Emergency/Recoveryの正当な監督権限は保持し、normal fallbackは追加しない。

**M2完了条件**: 旧不変条件違反のtestが修正で通り、同じartifactの全consumerに
モデル差がない。新旧schema/モデル値/epochの混在、async逆順、停止/再発進時の
入力変換を回帰確認。外部ROS/評価契約を変える必要があれば先に`docs/interface/`へ記載。

## M3〜M6: ローカル合格から提出物の合格へ進める

| 順序 | 実行・保存する証拠 | 合格条件 |
|---|---|---|
| M3a native/replay | 対象回帰、全consumer、保存932/933等。変更した問題に新identityを付けA/B/C/Dを同一worldで比較 | 正しいphysical proofと最終指令の一致。全方式失敗はUnknownとして保存 |
| M3b build/package | `make autoware-build`、Compose内`colcon test --packages-select multi_purpose_mpc_ros`、`colcon test-result --verbose` | 変更packageの有効なtest/build合格。既存の26/2381を新しいHEADの証拠にしない |
| M3c 短い統合確認 | 通常DLLの単車6周→限定dev2、run/Domain/source/receipt/decisionを固定 | penalty・説明不能なmoving override・未認証publishなし。最初の失敗で保存して原因調査へ戻る |
| M4 動作・遷移 | Follow、左右ShiftOut→Pass→Return、Hold/Stop→再発進、Recovery/Rejoin、Boost、async完了順逆転/target変更 | 移動中の認証済みStopを複数tick実適用し、停止・再発進まで観測。各動作の未観測を残さない |
| M5 固定campaign | 単車/dev2各3回、dev3六周、dev4六周、`make gate1/2/3`各1回 | 同じHEAD/configの全runを報告。通常走行は全車完走、gateは各ローカル判定合格。安全/identity違反0、callback/publishの時間条件合格 |
| M6 提出・閉鎖 | `./create_submit_file.bash`→`./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz`→`make eval` | 同じtarのSHA/imageと評価runを対応付け、archive最上位・ROS handshake・評価JSON・所有者/最新リンク契約を合格 |

M4の不足scenarioは、実行前にローカルscenario/初期配置と完了条件を記録して補う。
Recovery試験の意図的な注入と通常campaignの予期せぬRecoveryを区別する。
M5はseed指定の可否も記録し、途中で失敗を除外して3成功に数え直さない。
callbackの40Hz/25msとbackground workerの所要時間を分け、source/receiptの
欠測・逆転・重複、topic hz、mean/p95/p99/max、連続超過を同runで確認する。
既存の時間受入れ条件は、callbackの連続deadline超過とそれに伴うstale実行0件。
単発超過は計測・帰属し、throttleされた警告0件だけで全publishの正当性を断定しない。
dev3/dev4はDomain/V2Xと**全車**の結果を確認し、起動のみの成功で終了しない。
六周は開発受入れ条件。公式提出の必要周回やgate番号の公式対応は既存の暫定/TBDを維持する。

修正後は影響する検証をそのHEADへ揃える。ビルド中・シミュレーション中は編集せず、
simulationとheavy replay/buildを同時に動かさない。保護JSON、元DLL、既存rosbagを保持する。

## 中断・閉鎖の扱い

各段階の失敗時は、最初のdecisionと実際のsource/artifact/commandを一組で保存し、
accepted/rejected/inconclusiveと再試行条件をregistryへ登録する。原因を確定してから
次の修正へ進み、残作業を同じtasklistへ引き継ぐ。今のrollback基準は`ae2efe6a`。
ユーザー変更をresetせず、将来の修正はslice単位のrevertで扱う。

M1〜M6を証拠付きで閉じた時点で、正本仕様・registry・全体tasklist・必要分の
local commitを揃えて全体完遂とする。現時点ではモデル採用以降を未実施として明示する。
