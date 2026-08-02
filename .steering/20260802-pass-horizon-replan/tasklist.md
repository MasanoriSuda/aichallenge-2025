# Tasklist

## 設計

- [x] 最新runのPass遷移と失敗理由を整理する
- [x] 既存progress watchdogとの差を確認する
- [x] rear-clearまでのend-to-end mission案を定義する
- [x] Proレビューを実施する
- [x] 静的・動的validation horizonを分離する
- [x] horizon actionとextension commitを分離する
- [x] generationとabsolute Pass原点の契約を定義する
- [x] Hold中の固定OvertakeLine出力を定義する
- [x] AbortToRecoveryとAbortToSafeSeparationを分離する
- [x] wall contactとmargin-only violationの優先順位を分離する
- [x] planner生成時刻とprediction epochを分離する
- [x] atomic updateへPass保持距離とReturn pathを追加する
- [x] fresh horizonなしでPassへ遷移しない契約を定義する
- [x] 静的geometry validityを距離基準へ限定する
- [x] AbortToSafeSeparationの上限を定義する
- [x] Phase 1～3の段階実装を定義する

## Phase 1: Mission range

- [ ] rear-clear時刻・距離・ego speedをkinematic rolloutへ追加する
- [ ] rear-clear確認時間と制御遅延からconfirmation reserveを計算する
- [ ] dynamic Pass距離のsoft/hard上限を純粋関数化する
- [ ] Return区間をPass距離と分けて静的preflightへ追加する
- [ ] 静的validation距離をcandidateへ追加する
- [ ] 動的validation距離・時間をcandidateへ追加する
- [ ] planner生成時刻・prediction epoch・source age・絶対失効時刻をcandidateへ追加する
- [ ] extension planner result max ageを設定化する
- [ ] `OvertakeMissionPath` にPass保持距離・Return開始位置・Return距離を追加する
- [ ] `PassMissionValidation` を定義する
- [ ] Pass開始点基準のabsolute distance/timeを保存する
- [ ] runtime slackを保存せず毎周期算出する
- [ ] actual `ShiftOut -> Pass` 時刻を唯一のPass時間原点として保存する

## Phase 2: One-shot replan

- [ ] `PassHorizonAction` の純粋判断関数を追加する
- [ ] `SameSideExtensionCandidate` を定義する
- [ ] generation付きatomic updateを実装する
- [ ] replacement mission pathを他のmission状態とatomic更新する
- [ ] pending中の重複extension要求を抑止する
- [ ] same-side extensionを1 missionにつき最大1回接続する
- [ ] opposite-side candidateをPass中に生成しない
- [ ] extension後もabsolute Pass距離・時間を再装填しない
- [ ] rear-clear、Return、Recovery、target変更時にpending candidateを無効化する
- [ ] `AbortToRecovery` と `AbortToSafeSeparation` を接続する
- [ ] wall contactとmargin-only violationの処理順を分離する
- [ ] ShiftOut中のdynamic TTLを監視する
- [ ] fresh horizonなしでPassへ遷移しない境界処理を接続する
- [ ] fresh horizon待機を1回、1.0秒／3.0 mに制限する
- [ ] SafeSeparationを1.0秒／3.0 mに制限し、Holdから上限を再装填しない
- [ ] 状態変化ログを追加する

## Phase 3: Bounded fallback

- [ ] `PassHorizonMode::Holding` を追加する
- [ ] Hold中も固定OvertakeLineをpublishする
- [ ] Hold中のclosing speedを概ね0へ制限する
- [ ] Hold上限1.0秒／3.0 mを適用する
- [ ] Hold中のrear-clear成立で即Returnを評価する
- [ ] Phase 1～2の結果を確認後、複数回extensionの要否を判断する
- [ ] 動的予測不確実性をmission admissionへ反映する

## Unit tests

- [ ] 低速対象をrear-clearできるcandidateを確認する
- [ ] predicted Pass time budget内にrear-clear不能なcandidateを棄却する
- [ ] V2X予測1秒・mission 20 mを動的に完全検証済み扱いしない
- [ ] V2X source ageを差し引いたprediction epochからdynamic expiryを計算する
- [ ] planner生成が新しくてもsource predictionが失効済みならcandidateを棄却する
- [ ] planner result ageが設定上限を超えたcandidateを棄却する
- [ ] 動的予測終端より前に再計画slackがないcandidateを棄却する
- [ ] validation lead到達前はKeepになる
- [ ] validation lead到達後はRequestSameSideExtensionになる
- [ ] rear-clear済みかつReturn corridor成立でReturnになる
- [ ] opposite-sideだけ成立してもmid-Pass横断しない
- [ ] stale generationのextension結果を棄却する
- [ ] target IDまたはside不一致のextension結果を棄却する
- [ ] extensionのローカル距離・時間をPass原点へ正しく変換する
- [ ] extension後もabsolute limitを再装填しない
- [ ] extension後のvalid-untilが前進しないcandidateを棄却する
- [ ] extension commitでPass保持距離・Return開始位置・Return距離も一括更新する
- [ ] lap seam付近でもcircular path進捗を誤らない
- [ ] Return corridor blocker時にbase trajectoryへ戻らない
- [ ] margin-only violationかつrear-clear済みならReturnを優先する
- [ ] physical wall contactはReturnよりhard abortを優先する
- [ ] side-by-side AbortはSafeSeparationを選ぶ
- [ ] repeated Refreshで40 Hz全探索を繰り返さない
- [ ] extension goal adjustmentで同側goal jumpを許さない
- [ ] Return、Recovery、target変更でpending candidateを無効化する
- [ ] ShiftOut中に入口の動的予測が失効した場合、Pass進入時に再評価する
- [ ] dynamic TTLがbody-clear予定時刻未満ならstale予測のままShiftOutを継続しない
- [ ] fresh Pass horizon未取得時にPassへ遷移しない
- [ ] fresh horizon待機の再評価回数と時間・距離上限を超えない
- [ ] commit待ち中に失効したextension candidateを棄却する
- [ ] actual Pass startがpredicted Pass startからずれてもabsolute timer原点を誤らない
- [ ] Hold条件不成立時はHoldへ入らない
- [ ] Hold中のrear-clear成立でReturnになる
- [ ] Hold上限超過で無期限保持しない
- [ ] AbortToSafeSeparationが1.0秒／3.0 mを超えない
- [ ] HoldからSafeSeparationへ移ってもfallback上限を再装填しない

## Build verification

- [ ] `git diff --check`
- [ ] `make autoware-build`
- [ ] 追い越しコアテスト

## Dynamic verification

- [ ] `make dev2` で6周以上実施する
- [ ] front target捕捉数を記録する
- [ ] candidate admission数を記録する
- [ ] ShiftOut開始数とPass到達数を記録する
- [ ] rear-clear数とReturn完了数を記録する
- [ ] mission abort数と理由を記録する
- [ ] same-side extension数を記録する
- [ ] Hold回数・最大時間・最大距離を記録する
- [ ] validated horizon超過最大値を記録する
- [ ] SafetyBrake、wall、solver failure、crash penaltyを現行runと比較する
- [ ] クリアラップ45～46秒台を維持する

## Acceptance criteria

- [ ] admitted ShiftOut -> Pass: 100%
- [ ] Pass -> Return -> Idle: 90%以上
- [ ] validated horizon超過: 0 m
- [ ] stale dynamic horizonでのPass遷移: 0回
- [ ] mission pathとmission generationの不一致: 0回
- [ ] Hold最大: 1.0秒 / 3.0 m未満
- [ ] 同一mission extension: 初期版は最大1回
- [ ] wall / solver Recovery: 現行runより減少
- [ ] crash / wall penalty: 増加なし
- [ ] candidate admissionを不当に減らして見かけ上の成功率を上げていない
