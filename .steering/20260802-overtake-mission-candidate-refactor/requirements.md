# Requirements

## 目的

追い越し候補の生成、選択、Behavior 出力、OvertakeLine mission 確定の間で同じ値を別構造へコピーしている箇所を局所的に整理する。

次の性能修正（左右をまたぐ全候補比較、時系列 rollout、deadline slack 評価）を追加する前に、候補と選択結果の対応を一意にし、添字ずれや部分更新を起こしにくくする。

## 対象

- `OvertakeMissionCandidate` に選択後も必要な補助情報を保持する。
- 候補本体と補助メタデータの並行 `vector` を廃止する。
- Behavior から OvertakeLine へ渡す選択結果を一つの候補オブジェクトにまとめる。
- mission freeze 時の状態反映を一つの関数にまとめる。
- 候補選択で補助情報が失われない回帰テストを追加する。

## 制約

- 候補生成数、候補順、比較優先順位を変えない。
- gap、壁余裕、横加速度、closing speed、deadline の閾値を変えない。
- ShiftOut / Pass / Return / Recovery の遷移条件を変えない。
- ROS 2 topic、message、launch、parameter の契約を変えない。
- `aichallenge_submit` 外を変更しない（ステアリング文書を除く）。

## 完了条件

- 候補と補助情報が単一オブジェクトで選択・伝搬される。
- mission の freeze 項目が一か所で同時に設定される。
- `multi_purpose_mpc_ros` がビルドできる。
- `test_v2x_overtake_core` が成功する。

