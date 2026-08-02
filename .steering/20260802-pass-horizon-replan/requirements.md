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
6. 動的validationは予測生成時刻と失効時刻を保持し、ShiftOut中に古くなった入口予測をPass開始時に新規予測として扱わない。
7. horizon判断は `Keep / Return / RequestSameSideExtension / EnterHold / Abort` に分類する。
8. 再計画要求と、生成された延長candidateのmissionへの採用を別処理にする。
9. 延長candidateはgeneration、target ID、side、phase、有効期限の前進を確認して一括適用する。
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

### 動的validation

対象車・第三車両とのcorridorはV2X予測の信頼可能範囲だけを検証する。

- 遠未来が未観測であることだけではcandidateを棄却しない。
- 動的検証期限を短く設定し、期限前に再計画できるslackを必須とする。
- 動的検証期限を超えた区間を「検証済み」と表示・利用しない。
- 入口予測がPass進入時点で失効済み、またはlead time未満なら、Pass用動的horizonを即時再評価する。

## Mission lifetime契約

- mission作成時のgenerationは1とする。
- extension candidateは作成元generationを保持する。
- extension candidateは予測生成時刻を保持し、commit処理待ちの間に失効した結果を適用しない。
- 採用時にgeneration、target ID、side、phaseを再確認する。
- extension採用時はgenerationを1増やす。
- rear-clear、Return、Recovery、target変更時はpending candidateを無効化する。
- extension距離・時間は現在位置基準からPass開始点基準へ変換して保存する。
- extensionまたはHoldによってPass開始距離、Pass開始時刻、絶対上限をリセットしない。
- 初期実装では1 missionにつきextensionは最大1回とする。

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
- predicted Pass time budget: 8.0 s
- absolute Pass time limit: 10.0 s
- validated Pass soft distance: 24.0 m
- absolute Pass distance limit: 32.0 m
- same-side extension max distance: 8.0～12.0 m
- Hold上限: 1.0 s / 3.0 m
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
- ShiftOut中に失効した動的validationをPass開始時に使い回さない。
- Passが実効検証期限を無通知で超えない。
- rear-clear前の再計画は同じ側だけを対象にする。
- stale generationの延長candidateを適用しない。
- extension後も絶対Pass距離・時間上限を再装填しない。
- Return corridorが塞がっているとき、通常trajectoryへ戻らない。
- Hold中も固定OvertakeLineを出力する。
- side-by-side Abortで対象側へ横断しない。
- 旧32 m watchdogを残しても、新しいabsolute horizonが先に長時間Passを検出する。
- 純粋関数テスト、`make autoware-build`、追い越しコアテストが成功する。
- `make dev2` でentry数を隠さず、Pass完遂率とペナルティが改善する。
