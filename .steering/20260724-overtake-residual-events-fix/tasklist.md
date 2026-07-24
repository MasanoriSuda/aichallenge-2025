# 追い越し残事象修正 タスクリスト

## 調査

- [x] `070818`のcorridor hold中にno-gap制限が重複する経路を特定する
- [x] `073134`で2.0 m/sへの崩落が消えたことを比較する
- [x] common-course投影失敗とRecovery経路を確認する
- [x] `083221`のV2X観測、behavior、横計画、操舵、壁接触を時系列化する
- [x] 制動設定を超える急失速が物理的な壁接触によることを確認する
- [x] `085142`で既知の誤起動連鎖が再発しないことを確認する
- [x] Pass移行後のRecovery理由と残る減速を確認する

## 実装

- [x] bounded corridor hold中のno-gap制限を抑止する
- [x] 投影失敗を制約なし診断投影で分類する
- [x] front cap解除に現在の物理的横離隔を要求する
- [x] committed Passの距離進捗watchdogを追加する
- [x] OvertakeLineのraw footprint/static clamp guardを追加する
- [x] V2X速度観測の有効性とdistinct 3 sample確認を追加する
- [x] LowSpeed direct controlに横計画の単独所有権を持たせる
- [x] direct targetの作動中上書きを禁止する
- [x] measured speedによるdirect操舵capを追加する
- [x] dedicated confirmation gap、behavior速度cap、最終publish cap、
      direct raw footprint wall stopを最終差分へ反映する
- [x] `docs/spec/mpc-integration.md`へ内部動作を反映する

## 静的検証

- [x] final safety追加前の対象package testを実行する（23/23 test target成功）
- [x] final safety追加前のDocker buildを実行する（25 package成功）
- [x] final safety追加後に対象package testとDocker buildを再実行する
- [x] `git diff --check`とinterface差分を確認する

## 動的検証

- [x] corridor hold修正を同一dev2で比較する
- [x] hold区間の連続最大制動と2.0 m/sへの収束が消えたことを確認する
- [x] 停止車確認が最大1/3で誤起動しないことを確認する
- [x] 既知箇所でLowSpeed direct、wall、contact、stuckが発生しないことを確認する
- [x] lap 1完走を確認する
- [x] Pass移行と安全なRecovery理由を確認する
- [ ] 真の停止車に対するdirect開始、操舵/速度cap、wall stopを動的確認する
- [ ] `Pass -> Return -> Idle`の正常完遂を確認する

## 完了条件

- [x] 既知のno-gap急制動と壁衝突急失速の原因を分離して説明できる
- [x] `085142`で`083221`の誤起動・壁接触連鎖が再発しない
- [x] final safety差分をbuild/testで確認する
- [x] topic/service/message、Domain、launch、評価schemaに差分がないことを確認する
- [x] 動的未検証と未完遂事象を`results.md`へ明記する
