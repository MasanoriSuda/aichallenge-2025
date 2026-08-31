# E2E Dataset Contract Requirements

## Objective

TinyLidarNetの壁スタックを再学習で改善できるよう、bagから生成する教師データを
run単位で再現・監査できる形へする。モデル構造と走行パラメータは変更しない。

## Root Causes Addressed

- 各runのbag directory名が同じ`rosbag2_autoware`で、並列抽出先が衝突する。
- scanに最も近いcommandを同期誤差の上限なしで教師として採用する。
- 同一runのsampleがtrain/validationへ混在し得る。
- 同期誤差、topic数、reject数、元bagを成果物から追跡できない。
- 失敗したstudent自身のcommandとMPC/人間教師を区別できない。
- deserialize/type/shape異常を黙って捨て、dataset品質が分からない。

## Constraints

- 推論入力契約は750点LiDARのまま維持する。
- MPC/MPCC commandは教師labelとしてだけ使用し、E2E runtime入力へ入れない。
- output/やrosbag自体を編集・コミットしない。
- sample単位splitは禁止し、bag/runを分割単位とする。

## Acceptance

- 同名bagを複数指定しても抽出先が一意で、上書きしない。
- 同期誤差が設定上限を超えるsampleをrejectする。
- `delta_times.npy`と`metadata.json`を常に保存する。
- label sourceを明示し、student commandを教師として暗黙採用しない。
- train/validationのsequence IDが重複しない。
- type、scan shape、empty topic、全sample rejectを明示的に失敗分類する。
- unit testsで同期境界、split安定性、ID衝突を固定する。
