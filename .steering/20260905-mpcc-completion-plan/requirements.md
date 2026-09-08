# MPCC現行作業の完遂計画

作成日: 2026-09-05。今回の作業範囲は現状確認と計画作成。
計画の入口は [design.md](design.md)、根拠は [evidence.md](evidence.md)、進捗は [tasklist.md](tasklist.md)。

## 目的

現行の単一seven-state MPCCを、個別修正の受入れから、再現可能な統合受入れと
提出物からのローカル評価まで完遂する。今回、制御コード・設定・評価基盤は変更しない。

「完遂」は次の三段階を区別する。

| 段階 | 現状 | 完了の意味 |
|---|---|---|
| 制御構造の統合 | 実装済み | 通常指令の定式化をseven-stateへ限定し、旧normal authorityを削除 |
| 統合品質の受入れ | 未完 | Follow、追い越し、停止・復帰、多車両で証明と実行が連続する |
| 提出物の再現性 | 現HEADの証拠は未確認 | 固定tar.gzからeval imageを作り、同一成果物でローカル評価合格 |

## 基準

- branch: `develop_july`
- HEAD: `a8b968ac2e4a86a28432a52d132774459d355cb5`
- 最新の完了作業: [Follow topology監査](../20260831-follow-dynamic-topology-audit/results.md)。
- 再開地点: decision 1187のtarget tubeとsemantic stage時刻の整合比較。
- 現行値: local/cloudとも `mpc.N=20`、`control_rate=40 Hz`。
- [旧統合tasklist](../20260827-slice6-integration-acceptance/tasklist.md)のdev3受入れと
  動的統合完了は未チェック。後続の局所成功だけでこれを完了扱いにしない。

## 制約

- normal commandは同じsolutionから操舵・加減速を取得し、problem、trajectory、certificate、
  最終publishのidentityを一致させる。EmergencyとRecoveryは外部supervisorのまま維持。
- horizon、重み、clearance、solver tolerance、timeout、lease、grace、fallbackを
  原因修正の代用として変更しない。Slice 7の棄却済み調整を再開しない。
- 対象は原則 `aichallenge_submit/multi_purpose_mpc_ros` と関連ドキュメント。
  Domain、topic/service、launch入口、result schema、提出tar.gz最上位、`output/latest`契約は維持。
- ユーザーの結果JSON、未追跡snapshot、manifest、crash artifactを保持する。
- ログ・snapshot・build成果物の生成時期をHEADの証拠と混同しない。
- ローカルAWSIM検証として扱い、実車検証や2026公式安全ゲート適合を意味させない。

## 最終受入れ条件

1. decision 1187の修正前失敗と修正後成功が同じworld入力・物理条件で再現する。
   時間軸修正だけで成功しなければ、別原因の検証を完了してから次の実装単位を決める。
2. 全通常intentと遷移に動的証拠があり、未観測intentを合格に含めない。
3. 未認証・stale・wrong-generation・異なるsolution由来の通常指令は0件。
4. 受入れ用の通常走行では壁接触・他車衝突・説明不能なauthority消失が0件。
   停止・Recovery注入試験は別集計し、原因identityと安全復帰を確認する。
5. 40 Hz callbackの連続deadline超過と、それに伴う古い指令の実行が0件。
   単発の超過は計測・帰属し、隠れた品質問題を残さない。
6. 同一commit/configで単車、dev2、dev3、dev4およびローカルgateを検証し、
   再試行を含む全結果を保存する。単車・dev2・dev3は六周検証を含める。
7. build、対象package tests、提出tar.gz、eval image、`make eval`を通し、
   正本仕様・台帳・統合tasklistに結果と残存制約を反映する。

性能最適化や実車対応はこの完遂の後続課題。原因不明の異常を性能課題へ分類して閉じない。
