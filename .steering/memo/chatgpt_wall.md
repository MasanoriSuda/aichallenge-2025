まず最新ルールと公開リファレンス、2024版との差分を確認し、読む資料・改修箇所・検証方法を実装向けチェックリストに整理します。ルールPDFやGitHubがあれば、差分表まで具体化できます。

大きな差分として、2026は単独タイムアタックではなく3〜4台同時走行、V2X、追い越し、完走順やペナルティを含むレース最適化になっています。そのため速度チューニング中心ではなく、他車対応・安全ゲート・ペナルティ回避を設計に入れる必要があります。

思考時間: 7m 35s

参加されるのは、公式ドキュメント上の SW部門、TIER IV記事でいう Sim to Real SW部門 に相当するものだと思います。ここは「Autoware/ROS 2ベース＋ルールベース制御＋V2X」を使って走行性能を上げる部門です。
ただし公式ルールページ自体がまだ WIP で、一部未定・今後更新可能と明記されています。なので、改変方針は「現行ルール固定」ではなく、ルール差分に追従できる構造にしておくのがかなり大事です。

結論：2024優勝者ベースをそのまま速くするより、まず“競技構造”を作り替えるべきです

2024は 単独走行のタイムアタック が中心で、コース上を同時に他車が走ることはなく、仮想障害物・ピットストップ・コンディション管理がありました。
一方、2026のSW部門は 3〜4台同時走行のレース形式 で、6周の完走順が基本になり、他車を追い抜く技術が求められます。

なので必要な改変は、単なるパラメータ調整ではなく、だいたい次の5層です。

層	2024ベースからの変更方針
インターフェース層	2024用トピック、ピットストップ、仮想障害物処理を外し、2026のGNSS/IMU/車両状態/AWSIM状態/V2X入力へ合わせる
走行戦略層	“最速ラップ”から“複数台レースで勝つ”へ変更
経路計画層	単一レーシングラインだけでなく、追い越しライン・防御ライン・復帰ラインを持つ
速度計画層	加速度上限、ペナルティ、他車接近、ブースト、ハンディキャップを考慮する
評価・安全層	dev3/dev4、安全ゲート、提出環境、実車決勝を想定して壊れにくくする
まず捨てる／隔離するべき2024要素

2024ソフトから流用するなら、最初に「使う部分」と「捨てる部分」を分けてください。

流用しやすいもの は、操舵制御、MPC/Pure Pursuit系のチューニング、カーブごとの速度プロファイル、コースに対するレーシングライン生成、ログ解析ツールです。2026でもAutoware-Micro的な構成を使い、Planning/Controlを自作ノードに差し替えられる思想は続いています。

そのまま流用しにくいもの は、2024のピットストップ、仮想障害物、単独タイムアタック前提の速度最適化、固定スタート位置前提の初期化、他車がいない前提のライン取りです。2024には /aichallenge/objects や /aichallenge/pitstop/* 系のインターフェースがありましたが、2026のインターフェースでは構成が大きく変わっています。

改変で最優先に作るべきもの
1. 2026用の「入力アダプタ」

2024コードが自己位置を /sensing/gnss/pose_with_covariance 前提で見ているなら、まずここを直す必要があります。2026の仕様では、GNSSは /sensing/gnss/nav_sat_fix の sensor_msgs/msg/NavSatFix として扱われ、racing_kart_gnss_poser がNavSatFixを車両座標系の姿勢に変換する説明になっています。

最低限ほしい内部状態はこれです。

EgoState:
  x, y, yaw
  speed
  steering_angle
  lap_count
  section
  race_state
  boost_remaining
  is_boosting
  nearest_opponents[]

2026はROS 2のドメインIDで1〜4台の車両を分離する構成なので、複数台走行テストでは ROS_DOMAIN_ID 周りで事故りやすいです。通常の単独開発では意識しなくてもよい一方、AWSIMは各車両ドメインに接続してセンサと制御をやり取りします。

2. 他車をコース座標で扱う「レース判断層」

2026の本質はここです。世界座標 x, y のまま他車を見るより、コース中心線またはレーシングラインに射影して、

s: コース進行方向の距離
d: 中心線からの横偏差
Δs: 前後距離
Δv: 相対速度

で扱う方がかなり楽です。

実装する判断はこの4つで十分スタートできます。

if 前方車両が近い:
    速度を落とす or 追い越しラインへ
elif 横に車両がいる:
    横方向の急なライン変更を禁止
elif 後方から接近されている:
    無理な防御より安定走行を優先
else:
    通常レーシングライン

2026ルールではV2X情報、つまり他車両の位置情報が使用可能センサーに含まれています。
ただし、公式のインターフェースページ上ではV2Xの具体的なトピック名がまだ見えにくいので、実装では リファレンスソフト内の実トピック名・メッセージ型を必ず確認 してください。ここ、罠ポイントです。

3. 追い越し用の「複数ライン」

2024優勝者コードが単一の最速ラインを持っているなら、2026では最低3本に増やすのがおすすめです。

center_line       : 安全な基本線
racing_line      : 単独走行で速い線
overtake_line_L/R: 追い越し・回避用
recovery_line    : 壁接触後や姿勢崩れ後に戻す線

2026は衝突や不正加速度に対して一定時間の速度制限ペナルティがあり、壁衝突時の姿勢補正はあるものの、スタックを完全に防ぐものではないとされています。
つまり、「最短で突っ込む」より「接触しない・止まらない」ほうがレート戦では強い可能性が高いです。攻めすぎてペナルティを食らう車、サーキットあるあるの“速いけど勝てないやつ”です。

4. 加速度上限・ブースト・ペナルティ対応の速度計画

2026では加速度上限が約 1.0 m/s²、実車では速度上限30 km/h、レース内順位による加速度・速度ハンディキャップ、衝突等によるペナルティ、一時的に加速度を上げるブーストアイテムが記載されています。

そのため、速度計画はこういう形にするのがよいです。

target_speed = min(
  curvature_limit_speed,
  opponent_limit_speed,
  penalty_limit_speed,
  rule_limit_speed,
  stability_limit_speed
)

target_accel = clamp(
  speed_controller(target_speed, current_speed),
  min_decel,
  +1.0
)

さらに、2026の /awsim/status には boostRemaining と isBoosting があり、/awsim/cmd の boostCommand でブーストを発動する仕様が書かれています。
なのでブーストは、直線・姿勢安定・前方クリア・壁から離れている・追い越し中、の条件で使うのが安全です。

悪い例：

残っているから即ブースト

良い例：

if straight_section and yaw_error small and no_car_close_ahead and speed_stable:
    use_boost()
5. 安全ゲート対応

2026の決勝以降は、障害物停止、NPC追い越し、車線維持の安全ゲートをすべてクリアする必要があります。
開発環境にも make gate1, make gate2, make gate3 が用意されています。

ここは後回しにしない方がいいです。理由は、速い走行ロジックを作った後で安全ゲートに合わせると、全体がぐちゃっとします。最初から、

通常走行モード
追従モード
追い越しモード
停止モード
復帰モード

のように状態機械として切っておくのが堅いです。

具体的な作業順
Phase 1：2026リファレンスを無改造で通す

まず2024コードは触らず、2026公式リポジトリを素のまま動かします。開発サイクルは、aichallenge/workspace/src/aichallenge_submit/ を編集し、make autoware-build、make dev、make eval、提出ファイル作成という流れです。

./setup.bash bootstrap
./docker_build.sh dev
make autoware-build
make dev
make eval

評価結果は output/latest/d<domain_id>/ に、ログ、rosbag、詳細JSON、ラップタイムサマリー、motion analytics HTMLとして出るので、ここを基準値にします。

Phase 2：2024コードを“丸ごと移植”せず、差分で読む

2024優勝者コードから、まず以下を抽出します。

- 制御器：MPC / Pure Pursuit / 独自制御
- 経路：trajectory, lanelet2 map, racing line
- 速度計画：カーブごとの目標速度、減速開始点
- ログ解析：ラップタイム、横偏差、操舵、加速度

逆に、以下は一旦コメントアウトまたは削除候補です。

- /aichallenge/objects 依存
- /aichallenge/pitstop/* 依存
- 単独タイムアタック専用の最短ライン固定
- 固定スタート位置前提
- 他車が絶対にいない前提の速度上限
Phase 3：まず単独6周で安定化

ローカルの make eval は単独走行のタイムアタックですが、本番は複数台レースなので完全一致ではありません。それでも、提出環境に近い環境で依存関係やビルド失敗を検出する目的があると説明されています。

ここで見るべき指標は、タイムより先にこれです。

- 6周完走率
- 壁接触ゼロ
- 最大横偏差
- 最大yaw error
- target_accelが上限を破っていないか
- control_cmd publish周期
- 起動からReady/Startまでの状態遷移
Phase 4：dev3/dev4でレース化

2026環境では、make dev2, make dev3, make dev4 により1〜4台の同時走行を試せます。SIM予選は3台、SIM決勝は4台が想定されています。

ここから必要になるのは、単独最速化ではなく、

- 前走車に追突しない
- 横並びで蛇行しない
- 無理な追い越しをしない
- 抜ける場所だけで抜く
- 接触後に復帰できる

です。2026では蛇行走行や他車を邪魔する走行、危険走行、シミュレーション環境のハックは禁止で、決勝以降はコードチェックがあるとされています。

Phase 5：提出物として壊れない形にする

提出用ファイルは ./create_submit_file.bash で submit/aichallenge_submit.tar.gz に作られます。
外部依存、ローカル絶対パス、未コミットファイル、ビルド済み生成物への依存は本番で落ちがちなので、提出前に毎回これを確認してください。

./create_submit_file.bash
./docker_build.sh eval
make eval
make gate1
make gate2
make gate3
AIや他人に改変を依頼するなら、渡すべき情報

改変をかなり具体的に進めるには、次の情報が必要です。

1. 使用している2026公式リポジトリのcommit hash / branch
2. 2024優勝者ベースから変更済みの aichallenge_submit/ 一式
3. reference.launch.xml
4. package.xml / CMakeLists.txt
5. 自作ノードのソース
6. make autoware-build のログ
7. make dev または make eval の output/latest/
8. 2024コード内で購読・publishしているROSトピック一覧
9. どこまで目指すか：予選通過、SIM決勝、実車決勝、優勝狙い
10. 現状の問題：ビルド不可、走らない、遅い、壁に当たる、他車に追突する、など

特に必要なのは、これです。

tree -L 4 aichallenge/workspace/src/aichallenge_submit

grep -R "/aichallenge/objects\|pitstop\|pose_with_covariance" -n \
  aichallenge/workspace/src/aichallenge_submit

grep -R "control_cmd\|actuation_cmd\|awsim\|boost\|v2x\|vehicle/status" -n \
  aichallenge/workspace/src/aichallenge_submit

これで「2024依存がどこに残っているか」「2026の入出力に合わせるべき箇所」が見えます。

最初に入れるべき最小改変セット

勝ちに行く前の、まず2026仕様に乗せる最小セットはこれです。

- 2024のpitstop/virtual object処理を無効化
- GNSS入力を2026仕様に合わせる
- /awsim/status を購読して lap, section, boostRemaining を内部状態化
- /awsim/cmd でboostを安全条件付きで使えるようにする
- 単一レーシングラインを center / racing / overtake の複数ラインへ分離
- 前方車両とのΔsを見て追従・追い越し・減速を切り替える
- target_accelを +1.0 m/s²以下に制限
- make dev3 / dev4 / gate1〜3 をCI的に回す
実装方針のおすすめ

一番きれいなのは、2024コードを直接いじり倒すのではなく、2026側にこういう薄い構成を作ることです。

race_state_estimator
  └─ 自車状態、ラップ、セクション、ブースト、他車位置を統合

track_projector
  └─ x/y/yaw を s/d 座標へ変換

race_strategy_planner
  └─ 通常走行、追従、追い越し、防御、停止、復帰を決める

trajectory_selector
  └─ center/racing/overtake/recovery line を選ぶ

velocity_planner
  └─ カーブ、他車、ペナルティ、ブースト、加速度上限を考慮

controller
  └─ 既存MPCまたはPure Pursuitを流用・調整

safety_supervisor
  └─ 逸脱、急操舵、急加速、接触リスクを最終チェック

この構造なら、2026ルールが更新されても race_strategy_planner と velocity_planner を差し替えやすいです。

かなり重要な注意点

2026のルールはまだ未定箇所があり、タイムアップ、スタート位置、ブースト、スタック復帰などに「未定」表記が残っています。
なので、マジで避けたいのは「今の記述にだけ過適合したソフト」です。

おすすめは、パラメータを全部YAML化しておくことです。

race:
  max_accel: 1.0
  safe_follow_distance: 3.0
  overtake_min_gap: 5.0
  lateral_margin: 0.8
  boost_min_straight_length: 20.0
  boost_min_speed: 5.0
  collision_risk_ttc: 1.5

ルールが更新されたらコードを書き換えるのではなく、まずパラメータで追従できる状態にします。

まとめ

2024優勝者ソフトを2026向けにするために必要なのは、速い制御の移植 より先に、複数台レース・V2X・追い越し・ブースト・安全ゲート・新インターフェース への再設計です。
2024の資産は「制御器」「経路」「速度チューニング」「ログ解析」として使い、2024特有の「ピットストップ」「仮想障害物」「単独タイムアタック前提」は捨てるか隔離するのがよいです。

次に進めるなら、まず aichallenge_submit/ のディレクトリ構成、reference.launch.xml、2024依存トピックのgrep結果、make eval のログをそろえるのが最短です。