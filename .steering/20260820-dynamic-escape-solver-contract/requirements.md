# Requirements

## 背景

`output/20260820-140020` と `output/20260820-140412` では、static wall
preflight を通過した dynamic obstacle escape candidate が、約 10 ms 後の追従
QPで `maximum iterations reached` になり、side quarantine と追従への戻りを
繰り返している。

ログ上は次の二種類が混在している。

- `preflight_mode=margin-escape` だが、車体中心が通常境界内なので追従契約が
  `already-inside` となり、解の実寸footprint検証が作動しない。
- `preflight_mode=clear` でも、直前周期のwarm startを使った最初のsolveが最大反復で
  失敗し、cold startを試さずcandidateを隔離する。

## 要求

- margin-escape candidateは、中心境界の緩和が不要な場合も実寸footprintの解検証を
  必ず維持する。
- 壁境界と車両境界は緩和しない。既存の中心境界外だけを復元する契約は維持する。
- dynamic escapeのwarm-start solveが最大反復で失敗した場合だけ、一度cold solveを
  試し、単発の数値失敗でcandidateを隔離しない。
- cold retryも失敗した場合は従来どおりfallback/backoffへ移る。
- tracking traceだけでpreflight、境界契約、corridor、target adjustment、cold retryの
  採否と結果を特定できる。
- 通常Cruise、通常Follow、明確なinfeasible、実寸壁接触の扱いは変えない。

## 制約

- ROS 2 topic、message、launch、評価インターフェースを変更しない。
- clearance値や車両境界を緩めない。
- `output/` とユーザー所有の生成物を編集・コミットしない。
