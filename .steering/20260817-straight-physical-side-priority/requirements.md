# Requirements

## 背景

`output/20260817-221508` では、2 周目以降のスタート直線で右側が見た目上開いているにもかかわらず、18 m 先のカーブ内外判定を含む左側候補だけが採用された。右側はスコア負けではなく、初期 ShiftOut preflight で全候補が棄却されていたが、棄却理由が集計ログから欠落していた。

## 目的

- 直線入口では、将来カーブの内外ラベルより現在の物理的な開き幅を優先する。
- ロバスト離隔だけが成立しない場合、短い Progressive Entry に限り実寸離隔で再評価し、走行中の receding-horizon 再最適化へ引き継ぐ。
- 壁および車体の実寸境界、完全 Mission の成立条件は緩和しない。
- 片側候補が消えた理由を次回ログだけで判定できるようにする。

## 制約

- ROS 2 topic/service、提出物、評価インターフェースは変更しない。
- ユーザー変更中の `config/config.yaml` と `aichallenge/result-summary.json` は変更しない。
- 既存の locked-side/no-return 規則は維持する。
