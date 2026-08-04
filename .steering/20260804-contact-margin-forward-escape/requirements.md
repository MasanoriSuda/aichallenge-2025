# 車体間余裕と横並びPass完遂

## 目的

低速車との追い越しで、車体幅合計1.45 mに対して約5 cmしか余裕を持たない
minimum-motion goalを改善し、横並びまで到達したcommitted Passが将来予測の
一時的不成立で低速車より減速する事象を解消する。

## 要求

- 車体寸法`2.0 m x 1.45 m`の物理境界は変更せず、追い越し目標の中心間横距離を
  1.55 mへ引き上げる。
- minimum-motionは車両側のcorridor境界そのものを狙わず、空き幅がある場合は
  最大0.20 mを車体間余裕へ追加する。壁側の成立範囲は越えない。
- Pass front-capの再適用は中心間1.50 mで開始し、実車体重複の確認時間を
  0.05秒へ短縮する。
- 予測footprint重複は既存の確認時間を満たすまでactive Pass continuationの
  center separationを復活させない。
- 現在車体が非重複でfront cap解除済み、かつ相手がほぼ横並びの場合は、
  SafeSeparation中も現在速度以上を維持してrear-clearを目指す。
- actual footprint wall contact、現在車体重複の確認成立、EmergencyBrake、
  target discontinuity、solver recovery、絶対Pass時間・距離上限は緩和しない。
- ROS 2 topic/service/message契約は変更しない。

## 完了条件

- 余裕付きminimum-motion goalを純粋関数として単体試験できる。
- 横並びforward escapeの速度参照と適用条件を純粋関数で検証できる。
- transient predicted overlapがPass continuationを即時失敗させない。
- パッケージの単体テストとビルドが成功する。

