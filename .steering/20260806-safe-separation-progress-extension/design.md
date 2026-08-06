# Design

## 方針

SafeSeparation の局所上限を単純に拡大せず、前方完遂の進捗を監視して一度だけ局所枠を再設定する。

進捗は SafeSeparation の現在区間で観測した対象の最小前方距離で評価する。対象前方距離が設定値以上縮み、最後の改善からの経過時間が freshness 上限以内なら、前方完遂が進行中とみなす。

## 状態

- 区間開始時の対象前方距離
- 区間内で観測した最小対象前方距離
- 最後に対象前方距離が改善した時刻
- 進捗延長回数

局所枠を再設定した場合、上記の距離基準も現在値へ更新する。これにより、複数回を許可する設定でも各区間で新しい進捗が必要になる。現行設定は最大 1 回とする。

## 延長条件

次をすべて満たす場合のみ局所枠を再設定する。

- forward escape が許可され、対象が設定された前方 window 内
- 現在車体が非重複
- committed minimum-motion corridor と front-cap release が有効
- target ID、進行、位置観測が連続
- 短期安全条件が成立
- 区間内の前方距離改善量が設定値以上
- 最後の距離改善が freshness 上限以内
- 延長回数上限未満
- Pass 全体の絶対時間・距離上限未満

## 終了理由

pure core の resolution に reason を追加し、controller の phase transition log へそのまま反映する。局所距離上限と局所時間上限が同時に成立した場合は、実走で支配的な距離上限を優先表示する。

## 設定初期値

- progress extension: enabled
- minimum progress: 0.75 m
- progress freshness: 0.75 s
- maximum extensions: 1

Pass 全体の既存絶対上限 10 s / 32 m は変更しない。

