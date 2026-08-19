# Design

## 1. Target/壁競合の動的repair budget

現行はShiftOut/Passごとに0.35/0.75/1.50秒、2/4/8 mの固定holdを使う。
最新ログでは競合点が約9〜20 m先でも固定budgetを消費し、現在車体と短期予測がclearなまま
Missionを破棄していた。

純粋関数`resolve_target_bound_execution_repair_budget()`を追加し、次でbudgetを解決する。

- 基本値は既存設定値。
- 将来競合距離が有限なら、競合点の手前に既存safe-prefix reserveを残した距離まで拡張。
- 時間budgetも、解決した距離を現在速度で走る時間まで拡張。
- 無効入力では保守的に既存budgetを維持する。

拡張後も、既存の現在車体、予測sweep、実壁、front emergency、Mission絶対budgetを通す。
実行経路そのものは毎周期の物理再検証済みprefixを使う。

runtime wall preplan warningは0.20 mのhard reserveに追加した0.10 mの予告帯である。
物理hold pathが成立し、現在のhard wall guardもclearなら、警告は即終了理由ではなく
再計画要求として扱う。

## 2. SafeSeparation入力欠落

`InvalidInput`は数値破損だけでなく、一周期のtarget observation欠落でも発生し得る。
これをhard faultからsoft Mission failureへ分類し、既存のsame-side replan、
last-feasible、dynamic Mission waitの順で処理する。

またrear-clearが確認済みでReturn corridorがある場合は、target速度等の不要な入力検証より
先にReturnを選ぶ。rear-clear後に観測が消える正常事象をInvalidInputへ落とさない。

## 3. rear-clear済みReturn fallback

Returnでは追い越し対象が後方へ抜け、V2X target continuityが消えることが正常である。
それにもかかわらずlast-feasible leaseがShiftOut/Pass専用だったため、Returnの物理再検証失敗が
Recoveryへ直結していた。

- rear-clear latch済みReturnは、target continuityなしでもlast-feasible Returnを再検証可能にする。
- それでも実行horizonを生成できず、実壁接触・壁サンプル欠落・solver recoveryがない場合は、
  完了targetをblockして通常レーシングラインへhandoffする。
- 実壁接触または制御solver recovery時は従来どおりRecoveryを維持する。

## 非対象

- callback overrun削減
- horizon/model高度化
- 新しい左右戦術
- Recovery/Reverse性能調整

これらは本変更の動的効果確認後に別ステアリングで扱う。
