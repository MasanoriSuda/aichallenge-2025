# Design

## 方針

progress MPCCの準備処理を、QP本体を直接書き換える逐次処理から、成功時だけ適用するtransactional preparationへ分離する。

## Stage距離正規化

- 有限かつ正の距離はそのまま使う。
- 有限かつ0以上で、`minimum_reference_speed_mps * minimum_stage_dt_sec`未満の距離はその値へ持ち上げる。
- 負値、NaN、Infはrejectする。
- 正規化はprogress MPCC用コピーだけに適用し、legacy空間MPCのstage geometryは変更しない。

この最小距離は現設定で0.005 mであり、循環境界の重複点を約1 mの通常segmentへ誤拡張しない。

## Transactional preparation

`ProgressContouringMpcPreparation`へ次を事前構築する。

- 正規化済みstage距離
- unwrapped progress reference
- 全stageのtrust region
- 全stageの時間領域Frenet線形化
- stage/terminal progress cost

すべて成立した場合だけstate[2]、dynamics、costをprogress形式へ切り替える。一つでも不成立なら、その周期は既存legacy MPCを構築する。例外をcontrol fallbackへ送らない。

## 周回wrap

`MpcProblem`へ現在のprogress originを持たせる。前周期からtrust region幅を超える不連続が出た場合は、QP構造が同じでもOSQP primal/dual warm-startをresetする。

## ログ

- 0 m stageの正規化数を周期制限付きで記録する。
- progress preparation rejectとlegacy縮退理由を周期制限付きで記録する。
- 周回wrapによるwarm-start resetをmode切替と区別して記録する。
