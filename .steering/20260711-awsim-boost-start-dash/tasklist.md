# AWSIM 2026 Start Dash Boost Tasklist

作成日: 2026-07-11
更新日: 2026-07-11
状態: Implementation and Runtime Verification Complete / A-B Pending

## Definition of Done

- 2026公式 `/awsim/status`、`/awsim/state`、`/awsim/cmd` 契約が文書とコードで一致する。
- スタートダッシュ時にBoost pulseを1回だけ送る。
- Boost中、使用済み、残数なし、status異常・stale、fail-safe中は送らない。
- 未確認timeoutでも再送しない。
- `/awsim/status` で残数減少またはBoost中を確認できる。
- A/B走行ログでBoost効果を確認できる。
- legacy `boost_commander` を公式Boost経路として使用しない。
- build、unit test、runtime checkが成功する。
- ROS 2、Domain、launch、提出物、result JSON、`output/latest/` 契約を壊さない。

## Phase 0: 仕様・Baseline

- [x] 2026公式インターフェースで `/awsim/cmd` と `/awsim/status` の型・配列indexを確認する。
- [x] 2026公式シミュレーター仕様でBoost効果と `--boosts` 可変設定を確認する。
- [x] 現行公式teleopの `[1.0]` → `[0.0]` 実装を確認する。
- [x] ローカル `dev` / `eval` / `parallel` が `--boosts 2` であることを確認する。
- [x] 現行MPCは `/awsim/status` のlap/timeだけを読み、公式Boostを送信していないことを確認する。
- [x] 現行production launchでlegacy `use_boost_acceleration=false` であることを確認する。
- [x] requirements / design / tasklistを作成する。
- [ ] Boost未使用時の開始後15秒の `/awsim/status`、velocity、control command baselineを保存する。

## Phase 1: 契約文書の移行

- [x] `docs/interface/participant-interface.md` に2026公式AWSIM車両面topicを追加する。
- [x] Domain NへのAWSIM直接接続とDomain 0管理面の境界を明記する。
- [x] `docs/spec/open-questions.md` のBoost topic/typeを解決済みにする。
- [x] オンライン予選のBoost回数はTBDとして残し、コードへ固定しない方針を書く。
- [x] `docs/spec/mpc-integration.md` にstart-once/no-retry/legacy境界を追加する。

## Phase 2: Pure guard

- [x] `AwsimBoostConfig` と `AwsimBoostStartDashGuard` を追加する。
- [x] state文字列を正規化し、Start / Finish / Spawned epochを管理する。
- [x] status size、finite、remaining、isBoosting、steady ageを検証する。
- [x] control enabled / fail-safe条件を入力できるようにする。
- [x] 正常条件で `PublishPulse` を1回だけ返すlatchを実装する。
- [x] confirmation pending / confirmed / unconfirmed spentを実装する。
- [x] confirmation timeoutで再送しないことを実装する。
- [x] Finish後の新しいSpawnedだけ次セッションへrearmする。

## Phase 3: MPC node integration

- [x] `config.yaml` に `awsim_boost` sectionを追加する。
- [x] config modeとtimeoutのvalidationを追加する。
- [x] 既存 `/awsim/status` callbackからguardへ全statusを渡す。
- [x] `/awsim/state` subscriptionをシミュレーション時だけ追加する。
- [x] `/awsim/cmd` Reliable publisherをシミュレーション時だけ追加する。
- [x] 正常control cycleの安全判定後にguardを評価する。
- [x] `PublishPulse`時に `[1.0]`、`[0.0]` を各1回publishする。
- [x] trigger、confirmation、skip、timeoutの診断ログを追加する。
- [x] 実車、feature disabled、未知modeではpublishしない。
- [x] legacy `use_boost_acceleration` と公式Boostを接続しない。
- [x] `domain_enabled`で`ROS_DOMAIN_ID`ごとにBoostの有効・無効を上書きできるようにする。

## Phase 4: Automated tests

- [x] Start前はpulseなし。
- [x] status未受信、size不足、NaN / Inf、staleではpulseなし。
- [x] remaining 0とisBoosting trueではpulseなし。
- [x] control disabledとfail-safeではpulseなし。
- [x] 正常条件でPulse actionが1回だけ返る。
- [x] 100 control tick / status更新でも追加Pulseがない。
- [x] duplicate Start / Readyでは再armしない。
- [x] confirmation timeout後も追加Pulseがない。
- [x] remaining減少またはisBoostingでConfirmedになる。
- [x] Finish後の新しいSpawnedで次セッションに1回だけrearmする。
- [x] Domain別設定のtrue/falseと未登録Domainのglobal fallbackを単体テストする。
- [ ] ROS publish seamでhigh/lowがこの順序で各1回だけ出る。

## Phase 5: Build・Runtime verification

- [x] `make autoware-build` を実行する。
- [x] `colcon test --packages-select multi_purpose_mpc_ros` を実行する。
- [x] `colcon test-result --verbose` でfailure 0を確認する。
- [x] `make dev` で `/awsim/cmd` の型とpublish数を確認する。
- [x] `/awsim/status[5]` の1減少とindex 6のBoost中を確認する。
- [x] 同一レース中に2回目が出ないことを確認する。
- [ ] Boost無効化時にcommandが出ないことを確認する。
- [ ] Boost無効/有効の開始後10秒をA/B比較する。
- [x] `/control/command/control_cmd` の型とrateが維持されることを確認する。
- [ ] 必要に応じて `make dev2` 以上で各Domainが自車のBoostだけを消費することを確認する。

## Phase 6: 提出前確認

- [ ] `./create_submit_file.bash` で提出物を作成する。
- [ ] tar.gz最上位が `aichallenge_submit/` であることを確認する。
- [ ] `./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz` を実行する。
- [ ] `make eval` で1回だけ発動し、結果JSONと`output/latest/`契約が維持されることを確認する。
- [x] 実行コマンド、ログ時刻、結果をtasklistへ追記する。

## Follow-up: 戦略化

- [ ] 効果確認後、スタート以外の候補条件を別ステアリングで定義する。
- [ ] 直線、低速コーナー立ち上がり、追い越し時の比較指標を決める。
- [ ] V2X、順位、壁距離、yaw error、残り周回を使う安全guardを検討する。
- [ ] 2回目以降を使用する場合も `isBoosting` とconfirmationを必須にする。

## Verification Record

### Domain別有効化（2026-07-11）

- `awsim_boost.domain_enabled`を追加し、該当`ROS_DOMAIN_ID`のboolを`enabled`より優先する。
- 未登録Domainまたは`ROS_DOMAIN_ID`未設定時はglobal `enabled`へfallbackする。
- 既定のDomain 1〜3は従来動作を維持するためすべて`true`とした。
- `make autoware-build`: 25 packages成功。
- `multi_purpose_mpc_ros`: 13 test targets、242 tests、failure 0。Boost guardはDomain解決を含む15 tests成功。

2026-07-11 16:11 JSTにDomain 1の`make dev`で確認。走行ログは`output/20260711-161103/d1/autoware.log`。

- `make autoware-build`: 25 packages成功。
- `colcon test --packages-select multi_purpose_mpc_ros` + `colcon test-result --verbose`: 13 test targets、241 tests、failure 0。新規Boost guardは14 tests成功。
- `/awsim/cmd`: `std_msgs/msg/Float32MultiArray`、publisher 1 / subscriber 1。
- Boost発動ログ: 1回。確認ログ: 1回。`remaining_before=2`から`remaining=1, is_boosting=1`へ遷移。
- 発動後の`/awsim/status`: index 5=`1.0`、index 6=`0.0`（10秒効果終了後）。
- `/control/command/control_cmd`: 型は`AckermannControlCommand`を維持し、実測約99.7〜100.0 Hz。
- `git diff --check`と`config.yaml`のYAML parseは成功。`pre-commit`はhostにcommandがなく未実行。
- 起動したAWSIM / Autoware containerは確認後に`make down`で停止。
- Boost無効/有効のA/B効果定量比較、複数Domain、提出/evalは後続検証とする。
