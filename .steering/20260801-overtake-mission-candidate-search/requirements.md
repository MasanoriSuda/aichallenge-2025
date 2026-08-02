# Requirements

## 目的

追い越し開始時に単一の横目標・単一のShiftOut距離・固定時点のReturnまでを
一括判定する現行方式を改め、コース形状と動的空き領域に合う実行候補を選べるようにする。

## 要件

- 横目標とShiftOut距離の複数候補を評価する。
- 動的車両corridorはShiftOutとPass保持区間を開始時に確認する。
- Returnはlocked targetのrear-clear成立後に開始し、既存のlive wall/corridor判定を使う。
- 採用後は横目標とShiftOut距離をmissionへ固定する。
- 候補がなければFollowを継続し、開始判定だけを理由にRecoveryへ入れない。
- topic、service、message、提出物のインターフェースは変更しない。
- 既存のstart-grid breakoutとRecovery処理は変更対象外とする。

## 完了条件

- 複数候補から成立する最短ShiftOutが決定される。
- 固定4 mでは不成立でも、別のShiftOut距離で成立するケースを単体テストで確認する。
- Return区間の動的閉塞が追い越し開始を不必要に棄却しない。
- `multi_purpose_mpc_ros`のbuild/testが成功する。
