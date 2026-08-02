# Requirements

## 目的

ShiftOut後に固定した横目標を、入口または実行中に検証した距離・時間を超えてPassのまま保持し続ける事象を解消する。

追い越しmissionは、対象車のrear-clear成立までに必要なPass距離と、その後のReturnを入口で予測する。静的なコース成立性と短時間だけ有効な動的車両予測を分離して管理し、実行中に予測が外れた場合は、検証済み区間を使い切る前に同じ側の継続経路を再評価する。

## 最新走行の根拠

対象runは `output/20260802-175108`。

- ShiftOutは9回中9回Passへ到達した。
- 正常に `Pass -> Return -> Idle` まで完遂したのは2回だった。
- Pass中断はSafetyBrake 4回、壁余裕違反2回、corridor消失1回、target消失1回、solver Recovery 1回だった。
- 最後の失敗ではPass開始後に約34.5 m走行したが、設定上のPass距離は8 mだった。
- 現行の進捗watchdogは32 mごとに0.5 m以上対象へ接近すると再装填されるため、ゆっくり接近し続ける長時間Passを止めない。
- P1は6周したが、crash 5回、wall 5回、MPC failure 104ログ、後半ラップ92～121秒となった。
- 現在のV2X動的予測時間は1.0秒であり、24～32 m先までを動的に検証済みとは扱えない。

前回追加したkinematic rolloutとglobal candidate選択は、ShiftOut成立率を改善しているため維持する。

## 必須変更

1. candidate rolloutでbody-clearだけでなくrear-clear予測時刻・距離も求める。
2. 予測rear-clear位置までのPass保持と、その後のReturnを一つのmissionとして扱う。
3. 静的壁・曲率・横加速度・Return成立性はmission全体を事前検証する。
4. 対象車・第三車両の動的corridorは信頼できる予測範囲まで検証し、静的検証期限とは別に保存する。
5. 実効検証期限は静的期限と動的期限の早い方とし、その終端より手前で同じ側の継続経路を再評価する。
6. 動的validationはplanner生成時刻と予測基準時刻を分離し、V2X source ageを含む真の失効時刻を保持する。
7. horizon判断は `Keep / Return / RequestSameSideExtension / EnterHold / Abort` に分類する。
8. 再計画要求と、生成された延長candidateのmissionへの採用を別処理にする。
9. 延長candidateはgeneration、target ID、side、phase、有効期限の前進を確認し、Pass保持距離とReturn経路を含むmission path全体を一括適用する。
10. Pass距離・時間は最初のPass開始点を絶対原点とし、extensionやHoldで再装填しない。
11. Pass中に反対側candidateを生成せず、対象の横揺れで左右を直接横断しない。
12. side-by-side状態で通常/base trajectoryへ無条件に戻さない。
13. Hold中も固定OvertakeLineをpublishし、無経路状態を作らない。
14. side-by-sideでの中止は、通常Recoveryではなく同じ側を維持して前後分離する経路を優先する。
15. 物理的な壁接触と、追加wall margin不足を別条件として扱う。

## Validation契約

### 静的validation

次はrear-clear後のReturn終了まで成立必須とする。

- 壁footprint sweep
- map内判定
- 曲率速度cap
- 横加速度
- Return corridor

静的mission全体が成立しないcandidateは採用しない。

静的geometryの有効範囲は距離で管理する。candidate rollout上の到達予測時間は性能・absolute budget評価には使うが、「静的mapの時間的な失効期限」としては扱わない。

### 動的validation

対象車・第三車両とのcorridorはV2X予測の信頼可能範囲だけを検証する。

- 遠未来が未観測であることだけではcandidateを棄却しない。
- 動的検証期限を短く設定し、期限前に再計画できるslackを必須とする。
- 動的検証期限を超えた区間を「検証済み」と表示・利用しない。
- 入口予測がPass進入時点で失効済み、またはlead time未満なら、Pass用動的horizonを即時再評価する。
- 動的失効時刻は `prediction epoch + prediction horizon` とし、plannerを実行した時刻からhorizonを再加算しない。
- V2X source timestampとcontroller時刻のclock domainを正規化し、`prediction_epoch_monotonic = planner_now_monotonic - source_age` として比較する。
- ShiftOut中もremaining dynamic TTLを監視し、body-clear予定時刻より前に期限切れとなる予測で横移動を継続しない。

## Mission lifetime契約

- mission作成時のgenerationは1とする。
- extension candidateは作成元generationを保持する。
- extension candidateはplanner生成時刻、予測基準時刻、予測horizonを別々に保持し、commit処理待ちの間に失効した結果を適用しない。
- 採用時にgeneration、target ID、side、phaseを再確認する。
- extension採用時はgenerationを1増やす。
- rear-clear、Return、Recovery、target変更時はpending candidateを無効化する。
- extension距離・時間は現在位置基準からPass開始点基準へ変換して保存する。
- extensionまたはHoldによってPass開始距離、Pass開始時刻、絶対上限をリセットしない。
- 初期実装では1 missionにつきextensionは最大1回とする。
- atomic commitはfixed goal、closing speed、Pass保持距離、Return開始位置、Return距離、rear-clear予測、静的horizon、動的horizon、generationを一括更新する。

## ShiftOut / Pass境界契約

ShiftOut中からremaining dynamic TTLを監視する。Pass遷移時の動作を次へ固定する。

- freshな動的horizonが成立：Passへ移行する。
- fresh horizon未取得だが、現在footprint非重複かつ固定lineの短区間が安全：ShiftOut完了位置と固定OvertakeLineを維持し、closing speedを0として1回だけ再評価する。
- 短区間も不成立：横並びなら `AbortToSafeSeparation`、十分に前後分離していればRecoveryへ移行する。

古いhorizonでPassへ遷移してはならない。bounded再評価は1.0秒または3.0 mの早い方を上限とし、上限を再装填しない。

## Hold / Abort契約

`Hold` は上位BehaviorではなくPass内部の `PassHorizonMode::Holding` とする。

Holdへ入れるのは次をすべて満たす場合に限る。

- 現在footprintが対象と非重複
- 短区間の静的壁clear
- target continuity正常
- 対象が選択側へ侵入していない
- Emergencyではない
- solver正常

Hold中は固定OvertakeLine、target ID、side、fixed lateral goal、lateral ownershipを維持し、closing speedを概ね0へ落とす。上限は1.0秒または3.0 mの早い方とする。

Abortは次へ分ける。

- `AbortToRecovery`: 対象が十分前または後ろで、通常経路へ戻っても車体重複しない。
- `AbortToSafeSeparation`: side-by-sideのため、現在sideを短時間維持し、前後分離後にReturnまたはRecoveryへ移れる。

`AbortToSafeSeparation` も1.0秒または3.0 mの早い方を上限とする。Hold終了後にSafeSeparationへ移っても同じ上限を最初から再装填せず、同一fallback episodeの開始点から計測する。

## Guard優先順位

1. 物理接触、map外、現在footprint検証不能
2. confirmed current overlap、Emergency
3. target continuity異常
4. solver failure
5. rear-clear済みなら `ReturnBeforeWallMarginRecovery`
6. margin-only violation
7. Pass horizon判断

物理接触はhard abortとする。追加margin不足だけなら、rear-clear済みのReturnをRecoveryより優先できる。

## 初期パラメータ案

- revalidation lead distance: 3.0 m
- revalidation lead time: 0.75 s
- extension planner result max age: 0.10 s
- predicted Pass time budget: 8.0 s
- absolute Pass time limit: 10.0 s
- validated Pass soft distance: 24.0 m
- absolute Pass distance limit: 32.0 m
- same-side extension max distance: 8.0～12.0 m
- Hold上限: 1.0 s / 3.0 m
- ShiftOut fresh-horizon待機上限: 1.0 s / 3.0 m、再評価1回
- SafeSeparation上限: 1.0 s / 3.0 m（Holdと同一episodeなら共有）
- 初期版extension count: 最大1回

値は設定化し、ハードコードしない。24 mはsoft limitとして扱い、成功Pass距離の分布を確認してからhard limitの妥当性を再評価する。

## 対象外

- `a_max: 1.0 m/s^2` の変更
- ROS 2 topic、message、launch、評価インターフェースの変更
- Stuck Recovery全体の再設計
- 反対側へ移るmid-Pass weaving
- start-grid breakoutの経路選択変更
- Phase 1からの複数回extension

## Definition of Done

- 入口でpredicted rear-clearとReturnを含む静的candidateを評価できる。
- 静的・動的validation期限を混同しない。
- 動的expiryからV2X source ageを差し引く。
- ShiftOut中に失効した動的validationをPass開始時に使い回さない。
- freshなPass horizonがない状態でPassへ遷移しない。
- Passが実効検証期限を無通知で超えない。
- rear-clear前の再計画は同じ側だけを対象にする。
- stale generationの延長candidateを適用しない。
- extension後も絶対Pass距離・時間上限を再装填しない。
- extension採用時にPass保持距離とReturn経路もatomic更新する。
- Return corridorが塞がっているとき、通常trajectoryへ戻らない。
- Hold中も固定OvertakeLineを出力する。
- side-by-side Abortで対象側へ横断しない。
- AbortToSafeSeparationを別名の無期限Holdにしない。
- 旧32 m watchdogを残しても、新しいabsolute horizonが先に長時間Passを検出する。
- 純粋関数テスト、`make autoware-build`、追い越しコアテストが成功する。
- `make dev2` でentry数を隠さず、Pass完遂率とペナルティが改善する。
