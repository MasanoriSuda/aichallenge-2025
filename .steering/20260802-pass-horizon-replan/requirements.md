# Requirements

## 目的

ShiftOut後に固定した横目標を、入口で検証した距離を超えてPassのまま保持し続ける事象を解消する。

追い越しmissionは、対象車のrear-clear成立までに必要なPass距離と、その後のReturnを入口で予測する。実行中に予測が外れた場合は、検証済み区間を使い切る前に同じ側の延長経路を再評価する。

## 最新走行の根拠

対象runは `output/20260802-175108`。

- ShiftOutは9回中9回Passへ到達した。
- 正常に `Pass -> Return -> Idle` まで完遂したのは2回だった。
- Pass中断はSafetyBrake 4回、壁余裕違反2回、corridor消失1回、target消失1回、solver Recovery 1回だった。
- 最後の失敗ではPass開始後に約34.5 m走行したが、設定上のPass距離は8 mだった。
- 現行の進捗watchdogは32 mごとに0.5 m以上対象へ接近すると再装填されるため、ゆっくり接近し続ける長時間Passを止めない。
- P1は6周したが、crash 5回、wall 5回、MPC failure 104ログ、後半ラップ92～121秒となった。

前回追加したkinematic rolloutとglobal candidate選択は、ShiftOut成立率を改善しているため維持する。

## 必須変更

1. candidate rolloutでbody-clearだけでなくrear-clear予測時刻・距離も求める。
2. 予測rear-clear位置までのPass保持と、その後のReturnを一つのmissionとして静的壁・動的corridor検証する。
3. missionに「Passをどこまで検証済みか」を保存する。
4. 検証済み区間の終端より手前で、現在状態から同じ側の継続経路を再評価する。
5. 再評価結果を `Return / ExtendSameSide / HoldFrozenLine / Abort` に明示的に分類する。
6. 延長採用時はgoal、closing speed、rear-clear予測、検証済み距離、deadlineを一括更新する。
7. Pass中に反対側へ直接横断しない。
8. side-by-side状態で通常trajectoryへ無条件に戻さない。

## Hard guard

以下の優先順位は変更しない。

1. 実車体の壁接触・壁余裕違反
2. 現在車体の重複とEmergency判定
3. target continuity異常
4. solver failure
5. Pass horizon再計画

Pass horizon機能は、既存hard guardを無効化する機能ではない。

## 対象外

- `a_max: 1.0 m/s^2` の変更
- ROS 2 topic、message、launch、評価インターフェースの変更
- Stuck Recovery全体の再設計
- 反対側へ移るmid-Pass weaving
- start-grid breakoutの経路選択変更

## Definition of Done

- 入口でpredicted rear-clearとReturnを含むcandidateを評価できる。
- Passが検証済み距離を無通知で超えない。
- rear-clear前の再計画は同じ側だけを対象にする。
- Return corridorが塞がっているとき、通常trajectoryへ戻らない。
- 旧32 m watchdogを残しても、新しいabsolute horizonが先に長時間Passを検出する。
- 純粋関数テスト、`make autoware-build`、追い越しコアテストが成功する。
- `make dev2` でShiftOut成立率9/9相当を維持しつつ、Pass完遂率とペナルティが改善する。
