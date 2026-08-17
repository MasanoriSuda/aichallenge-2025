# Requirements

## 目的

追い越し中の横位置、速度、コース進捗を同じQPで最適化し、固定距離Missionの
再構築だけに依存しないcontouring-progress MPCCの基礎を導入する。

## 必須要件

- 既存のFrenet DP stage corridorをhard lateral boundとして維持する。
- Persistent OSQPとprimal/dual shift warm-startを維持する。
- 追い越し実行中は第3状態を経過時間`t`ではなく実コース進捗`s`として扱う。
- contour errorは既存のstage-wise `e_y`目標、heading alignmentは`e_psi`目標を使う。
- lag errorは線形化進捗と最適化進捗の差としてQP costへ入れる。
- progress rewardとprogress trust regionをQPへ入れる。
- 従来の空間領域MPCへ設定一つで戻せる。
- topic、service、launch、提出物のインターフェース契約を変更しない。
- ユーザーが変更した`steering_tire_angle_gain_var: 1.6`を変更しない。

## 今回の対象外

- 複数回RTI-SQP
- terminal velocity cost
- dynamic bicycle / tire model
- HPIPMへのsolver変更
- tactical workerとRecoveryの別プロセス化

## Definition of Done

- 進捗線形化、trust region、cost生成をpure C++で単体テストできる。
- progress MPCC有効時もQPの状態・入力次元と固定疎構造を維持する。
- mode切替時に異なる意味のwarm-startを再利用しない。
- 対象packageのbuild/testが成功する。
