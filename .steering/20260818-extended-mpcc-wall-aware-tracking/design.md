# Design

## 背景

直前runでは Extended MPCC の成功率と追い越し完遂が改善した一方、失敗理由が壁関係へ集中した。stage corridorは既に車体footprintとhard wall clearanceを含むが、soft tracking referenceがその境界上に置かれると、強い横追従重みが実行誤差の余裕を残さない。

## 方針

各Extended MPCC stateの横参照を、既存hard bound内の0.15 m内側へ投影する。これはsoft objectiveだけの補正であり、hard bound自体は縮めない。

幅が0.30 m未満で指定余裕を両側に確保できない場合は、参照を中央へ置き、確保できた余裕に比例して横追従重みを1.0から0.25まで下げる。したがって狭いが物理的に有効な経路を新たに棄却しない。

既存の物理再検証、last-feasible trajectory保持、実壁接触guardは変更しない。新しいfallback FSMを追加せず、参照生成の原因側を修正する。

## 影響範囲

- Extended 5x3 MPCCのみ
- legacy 3x2 MPCC、V2X契約、評価インターフェースは不変
- hard wall/target bounds、EmergencyBrakeは不変
