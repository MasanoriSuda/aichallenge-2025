# Requirements

## 目的

`20260820-145301` で観測した壁接触について、DynamicEscape の経路自体、
solver fail-operational handoff、通常 racing-line 追従誤差のどこで壁余裕を
失ったかを次回試走ログだけで区別できるようにする。

## 変更範囲

- `multi_purpose_mpc_ros` 内の最終制御決定ログ
- DynamicEscape から別の制御源へ切り替わった直後の限定監視
- 最終採用済み予測軌道に対する車体 footprint と静的壁の距離評価
- 既存の衝突 condition ログの識別情報
- 上記ロジックの単体テスト

## 制約

- 制御、経路選択、速度・操舵パラメータは変更しない。
- 候補軌道ではなく、公開指令に対応する最終採用軌道を記録する。
- 40 Hz で全ホライズンを常時計算しない。
- 通常走行ログを増やさず、handoff、壁接近、接触のイベント時だけ出す。
- ROS 2 topic/service/message 契約を変更しない。
- `aichallenge_system` と生成済み `output/` は変更しない。

## Definition of Done

- DynamicEscape の開始・終了・solver fallback 切替時に1行の構造化ログが出る。
- 切替後2秒以内の壁接近・接触が即時ログになる。
- ログから現在車体と採用予測軌道の最小壁距離、位置、壁方向を確認できる。
- 操舵指令の切替差分と車両状態を同じ行で確認できる。
- 関連単体テストとパッケージビルドが通る。
