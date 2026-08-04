# Pass継続とReturn preflightの分離

## 目的

既にPassへ入り同じ側を安全に走行している車両が、将来のReturn形状だけを理由に
SafeSeparationへ移行し、相手より減速して追い越しを中断する事象を解消する。

## 要求

- 初回Entryでは従来どおりShiftOut/Pass/Return全体を成立確認する。
- committed Passの延長・再検証では、現在位置から同じ側を維持してrear-clearへ
  到達するまでのShiftOut/Pass部分だけを静的preflightする。
- rear-clear後のReturnは、その時点の現在位置から既存のReturn実行horizonとして
  再生成・検証する。
- 現在の壁接触、実行horizonの壁不成立、車体重複、緊急制動、solver異常、
  絶対時間・距離上限は緩和しない。
- 設定値およびROS 2インターフェースは変更しない。

## 完了条件

- Pass continuation policyがReturnを検証範囲外とする契約を明示する。
- committed Pass preflightがShiftOut/Pass範囲だけを評価する。
- 初回mission admissionのfull-path preflightが維持される。
- 単体テスト、パッケージビルド、既存テストが成功する。
