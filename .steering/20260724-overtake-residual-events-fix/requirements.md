# 追い越し残事象修正 要件

## 目的

dev2の追い越しシナリオで確認された急失速と壁接触について、制御指令、
V2X追跡、操舵経路、静的壁判定を時系列で突き合わせ、原因ごとに最小限の
安全化を行う。追い越し完遂率の調整とは分離し、再現性のある事実だけを
効果として扱う。

## 対象事象

1. `output/20260724-070818/d1`
   - ShiftOut中のlive execution corridor一時欠落を既存のbounded holdで
     継続許可している一方、`no_gap_target_velocity: 2.0 m/s`が重複適用され、
     最大減速度指令が連続する。
2. corridor欠落後のtarget loss
   - 24 mのbounded common-course探索外へ出た通常lossまで
     `locked target course progress discontinuity`と誤分類する。
3. `output/20260724-083221/d1`
   - 初回または同時刻のV2X観測で速度を計算できない車両を停止車と誤判定し、
     LowSpeed direct controlを開始する。
   - direct controlの右操舵とOvertakeLineの左操舵を同時に生成し、古いdirect
     controlが最終出力を所有する。
   - 実速度約7.19 m/sでraw steering約-0.445 rad、最終出力約-0.667 radとなり、
     壁接触後に約-6.17 m/s²の急失速へ至る。
4. Pass継続中の残事象
   - lateral-clearの履歴だけで前方保護を解除すること、進捗のないPass継続、
     実車体と静的壁の不整合、静的壁clamp後の過大横加速度をfail closedにする。

## 変更範囲

- `multi_purpose_mpc_ros`内部のV2X追跡・停止車確認
- gap plannerのno-gap速度制限適用条件
- common-course投影失敗の診断分類
- OvertakeLineとLowSpeed direct controlの横計画所有権
- Pass中の前方保護、進捗watchdog、静的壁guard
- direct controlの速度・操舵・静的壁guard
- 上記判定のpure C++ helperと単体テスト
- ステアリング文書と`docs/spec/mpc-integration.md`の内部動作説明
- 同一dev2シナリオによる効果確認

## 機能要件

### corridor hold

- bounded live corridor hold中は、同じ一時欠落を根拠に2.0 m/sのno-gap制限を
  重ねない。
- hold期限切れまたはhard failure時は、従来どおりcorridor blockとno-gap制限を
  戻す。

### target loss診断

- 進捗連続性制約付き投影だけが失敗した場合と、同じbounded探索条件の
  制約なし診断投影も失敗した通常lossを区別する。
- 制約なし結果は診断専用とし、追跡対象やfront判定には採用しない。
- どちらのlossも既存のtarget hold後にRecoveryへ移行し、追い越し完了扱いしない。

### 停止車direct control

- 停止車候補は速度計算が有効なV2X観測だけを対象とする。
- 同一IDの異なる受信時刻による連続3観測でのみ開始し、同一観測を読む複数の
  制御周期は加算しない。候補消失、ID変更、無効値、許容時間超過で確認をリセットする。
- 確認観測間隔はstall watchdogとは別の専用設定とする。
- direct control作動中はOvertakeLineを同時実行せず、選択済みpass targetを
  反対側の候補で上書きしない。
- 実速度に基づく横加速度上限を操舵target、direct内部出力、最終publish後の
  すべてに適用する。
- direct速度はphase設定値に加え、behaviorの`desired_velocity`と
  `target_velocity_limit`を上限とする。
- 実車体footprintを静的壁gridで検証できない、map外、接触、または必要clearanceを
  満たさない場合は、direct controlをfail closedで停止・中立操舵する。

### Pass安全化

- 過去に横離隔が成立したというlatchだけで前方速度保護を解除せず、
  現在の物理的横離隔を要求する。
- committed Passで32 m走行しても対象前方距離が0.5 m以上改善しない場合は
  Recoveryへ移行する。
- ShiftOut/Pass中の実車体footprintが静的壁に対して無効、map外または接触なら
  Recoveryへ移行する。
- 静的壁clamp後に横加速度上限を再超過するtargetは採用せず、Recoveryへ移行する。

## 制約

- SafetyBrake、front risk、solver fail-safe、wall/static footprint、
  rear-clear確認を緩和しない。
- common-courseの24 m探索窓を拡大しない。
- 追従距離、EmergencyBrake、評価基盤は今回の原因対策のために変更しない。
- ROS 2 topic/service/message、Domain、launch、評価結果schemaを変更しない。
- `output/`、rosbagなどの生成物を編集・コミットしない。

## Definition of Done

- 対象helperの境界条件を単体テストし、対象packageのtestとDocker buildが成功する。
- 同一dev2走行でcorridor hold中の2.0 m/sへの崩落が消える。
- `083221`で発生した単一観測によるLowSpeed direct誤起動、左右横計画の競合、
  壁接触、約-6.17 m/s²の急失速が同じ走行箇所で再発しない。
- 正常な停止車に対するdirect経路を未走行の場合は、静的検証済みと動的未検証を
  区別して記録する。
- Pass完遂、Recovery中の減速など未解消事象を、観測値と次の確認条件付きで記録する。
