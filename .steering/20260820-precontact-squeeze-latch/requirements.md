# Requirements

## 目的

pre-contact squeeze responseがfront capを再適用した次周期に、
`front-cap-not-released`を理由として自己終了する不具合を解消する。

## 変更範囲

- responseの取得条件と継続条件の分離
- front cap再適用後の継続latch理由の可視化
- core回帰テスト

## 制約

- 初回取得には従来どおfront cap release済みを必須とする
- predicted sweepがclearに戻った場合は終了する
- actual contact後はContactContinuationへ引き渡す
- target不正、Pass終了、機能無効時はラッチしない
- ROS 2・評価インターフェースは変更しない
- `aichallenge/result-summary.json`のユーザ変更は対象外

## Definition of Done

- response開始後にfront capが再適用されてもactiveを保持する
- 保持中は`active-held-after-front-cap-reapply`と記録される
- 未取得状態でfront capがreleaseされていない場合は取得しない
- package build/testが成功する
