# Requirements

## 目的

20260818-192235 の試走で確認した Extended MPCC の壁際破綻を抑える。
追従性能の回復は維持し、全区間の追従重みを再び弱めない。

## 変更範囲

- Extended MPCC の横参照と横追従重み
- 設定、起動ログ、単体テスト

## 制約

- 壁・車両の既存hard constraintを緩和しない。
- 狭いcorridorを新しい余裕でhard infeasibleにしない。
- 既存のlast physically validated trajectory保持を再利用する。
- ユーザー変更中の操舵・平滑化設定と結果JSONには触れない。

## 完了条件

- 壁境界上のsoft referenceが内側へ補正される。
- 指定余裕を確保できない区間では横追従重みだけが下がる。
- 既存テストとビルドが成功する。
