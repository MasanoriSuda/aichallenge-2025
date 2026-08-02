# 設計

## 方針

前段のmission ownership resolverを利用し、Behaviorの最終状態決定直前に committed Pass owner を判定する。

owner成立時は以下を行う。

- Behavior stateを`Overtake`に維持
- missionで固定済みのpass sideを出力
- candidate gap/curve/completion判定は診断値として残す
- lateral pathは作り直さず、OvertakeLineの固定goalを継続
- Pass speed policyを継続

## 安全責務

Behavior ownerはEntry条件だけを無視する。現在車体が重複している場合やtarget continuityが壊れた場合は成立しない。

Behavior評価後に実行するgap plannerはlive corridorを再確認し、`overtake_execution_corridor_blocked`を生成する。OvertakeLineは従来どおり壁、実行horizon、横加速度、solver failureを監視し、必要ならRecoveryへ遷移する。

予測footprint重複だけではownerを解除しない。現行attack modeと同様、現在車体が分離し、実行経路が成立している間は前方へ抜き切る。予測が実重複またはlive corridor不成立へ進展した時点で既存Hard abortが働く。

## ログ

既存の状態遷移ログと周期debugログへ`pass_owner`を1項目追加する。ログメッセージ数は増やさない。
