# Requirements

## 目的

`20260820-160913` で再現した、DynamicEscape の solver bounded continuation
終了直後に壁接触予測を持つ racing-line 解を採用し、約0.6〜0.9秒後に実車が
壁へ接触する欠陥を防止する。

## 確認済み事実

- DynamicEscape から racing-line へ戻る瞬間の現在 footprint は壁から離れている。
- 最終採用された `current_prediction` は切替時点ですでに壁へ接触している。
- 操舵差分は約0.03 radであり、急な操舵切替より採用経路自体が主因である。
- 同じ因果列が一回の試走で2回再現している。
- AWSIM collision condition は発火しておらず、occupancy footprint 証拠が必要である。

## 変更範囲

- solver bounded continuation から通常MPCへ戻る最終ハンドオフ契約
- 最終採用予測に対する壁footprint admission
- 危険なハンドオフ中の短時間の横指令保持と減速
- ハンドオフ拒否・再認定・解除を追跡する構造化ログ
- 純粋な状態遷移ロジックの単体テスト

## 制約

- 通常の追い越し候補選択、速度、壁余裕パラメータは変更しない。
- solver の数値的成功だけではハンドオフを許可しない。
- 壁接触、out-of-map、無効・欠損予測、必要壁余裕未満を許可しない。
- 危険な解をタイムアウトだけで採用しない。
- ROS 2 topic/service/message 契約を変更しない。
- `aichallenge_system`、`output/`、ユーザーの既存変更は変更しない。

## Definition of Done

- continuation終了後の最終予測が壁契約を満たさない場合、通常MPC指令を公開しない。
- 物理的に有効な予測が連続2周期成立してから通常MPCへ戻る。
- 保留中は直前の横指令を維持し、加速せず制御された減速を行う。
- ログからentry/block/requalifying/release、拒否理由、壁距離、保持周期を確認できる。
- 単体テストとパッケージビルドが通る。
