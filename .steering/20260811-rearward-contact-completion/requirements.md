# Requirements

## 目的

20260811-165236 の走行で、追い越し対象が自車後方へ移り始めたにもかかわらず、
side contact の固定 0.8 秒上限が先に切れ、`short horizon unsafe` から Recovery へ
落ちた追い越しを完遂させる。

## 対象事象

- `ContactContinuation` は target が `s=1.24 m` から `s=0.24 m` まで改善中でも
  0.83 秒で終了した。
- その約 0.65 秒後には target が `s=-0.66 m` まで後方へ移ったが、車体はまだ
  rear-clearしておらず、予測不成立が `Pass -> Recovery` を選んだ。
- 既存の rear-clear完了処理は車体非重複後には有効だが、接触／近接状態から
  非重複へ抜ける最後の短い前進区間を保持できていない。

## 制約

- `target_s < 0` だけで追い越し完了またはReturnにしない。
- 延長はSideBySide committed、forward-completion latch済み、同じ側の接触形状、
  freshな前進進捗がある場合だけ許可する。
- 壁接触、target continuity喪失、正面衝突形状、過大な相対横速度、停止、
  Pass絶対時間／距離上限は従来どおりhard guardとする。
- ROS 2 topic/service、launch、評価インターフェースは変更しない。
- ユーザーの `aichallenge/result-summary.json` 変更は編集しない。

## 完了条件

- 通常のcontact continuation上限0.8秒は維持する。
- targetが後方へ移り、完遂条件を満たす場合だけ2.5秒まで延長できる。
- rear-clear成立後は既存のPass完了／Returnへ接続する。
- 条件不成立または延長上限到達時は従来どおりfail-closedになる。
- core単体テストと対象packageのビルドが成功する。
