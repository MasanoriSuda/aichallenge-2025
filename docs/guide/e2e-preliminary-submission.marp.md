---
marp: true
theme: gaia
paginate: true
size: "16:9"
title: Automotive AI Challenge 2026 E2E AI 予選提出
description: LiDAR直接操舵と結果証明付き改善サイクル
style: |
  :root {
    --head: #0b3d5c; --accent: #e14534; --text: #17242e;
    --muted: #5b6b7a; --pale: #eef3f6;
  }
  section { color: var(--text); font-size: 23px; padding: 42px 54px; line-height: 1.42; background: #fff; }
  h1, h2 { color: var(--head); }
  h2 { border-bottom: 3px solid var(--accent); padding-bottom: .16em; }
  strong { color: var(--head); }
  table { font-size: .78em; }
  th { color: #fff; background: var(--head); }
  code { background: var(--pale); }
  .columns { display: grid; grid-template-columns: 1fr 1fr; gap: 1em; }
  .small { font-size: .76em; }
  .muted { color: var(--muted); }
---

<!-- _class: lead -->

# Automotive AI Challenge 2026
## End to End AI 部門 予選提出

**2D LiDARから横方向制御までをMLで直結**<br>
結果証明付きデータとclosed-loop Gateで改善

- チーム: **[チーム名を記入]**
- 動画: **[公開URLを記入]**

---

## 背景と戦略

<div class="columns">

<div>

### 目標

- 地図・自己位置・V2XをE2E推論へ入れない
- 2D LiDARから直接steeringを出す
- NPC環境で停止・横回避をclosed-loop実証する
- 学習に使わない開始seedでも再現する

</div>

<div>

### 開発方針

1. 単車で車線維持を成立
2. NPC環境で失敗を収集
3. base modelを凍結して回避補正を学習
4. run単位でtrain/validationを分離
5. 実行結果でartifactを昇格・棄却

</div>
</div>

---

## モデルアーキテクチャ

```text
750点 2D LiDAR ──> TinyLidarNet ──────────────┐
       │              base steering           │
       └─> frozen Conv特徴 + 車輪速度 + base舵 ├─> spatial ML adapter
                                                │    └─> steering command
LiDAR freshness / 前方距離 ─────────────────────┴─> longitudinal safety
```

- 横方向authorityはML出力のみ
- runtime許可入力: LiDAR、車輪速度
- spatial adapterはfreshなadmitted scanで補正を推論し、normal anchorで通常時を0付近へ抑える
- 最終操舵: `clip(base steering + spatial correction, -0.64, +0.64 rad)`
- watchdogはstale LiDAR時に前進指令を保持しない
- production: `0.8 m/s²`、速度上限`4.6 m/s`

<p class="small muted">GNSS、IMU、V2X、地図trajectory、MPC出力はE2E推論入力に未使用。</p>

---

## 開発前後の比較

| | 初期TinyLidarNet | 現在のfrozen base + spatial ML |
|---|---|---|
| 通常走行 | 単一scanから直接操舵 | baseを凍結し通常走行を維持 |
| 障害物対応 | 前方距離による縦停止が中心 | LiDAR空間特徴と車輪速度で横補正 |
| 観測結果 | 約150秒後に前方約1.5 mで長時間停止 | NPC 3 seedを各3周、1位・penalty 0 |
| 品質管理 | sample単位評価 | run分離、closed-loop、artifact SHA、実行coverage |

改善はモデルを全面再学習するのではなく、失敗をrun outcomeまで遡り、凍結baseへ必要な
横補正だけを追加することで得た。

<p class="small muted">初期runと最終runは同一seedの厳密A/Bではないため、観測された開発段階比較として記載。</p>

---

## 学習データと品質契約

- MPC単車走行と成功normal runから車線維持を学習
- NPC／peer失敗の接触前区間を解析し、LiDAR回避教師との差分を収集
- frozen baseに対するsteering residualとして回避を学習
- sequence/run単位でtrainとvalidationを分離
- checkpoint SHA、元bag、topic型、同期差、run outcomeを記録

### 採用を止めるGate

- Finish未達、penalty、stallのある教師runはhard labelへ採用しない
- 同じ状態で矛盾するnormal／teacher labelを監査
- offline改善だけではproductionへ昇格しない
- 単車 → NPC未見seed → 混走の順にclosed-loop確認

---

## 他車両への回避と停止

<div class="columns">

<div>

### 横回避

- LiDARの空間特徴から回避方向を学習
- base操舵と速度を条件にfull-steering補正
- 近接時にもMLが横方向commandを所有

### 縦安全

- 前方`3.0 m`以下: 加速抑止
- 前方`1.5 m`以下: `-1.0 m/s²`制動
- stale scan: 前進をfail-closed

</div>

<div>

### 動画に表示するもの

**[seed 2037 NPC走行の静止画を配置]**

- egoがNPCへ接近
- ML steeringで空間を選択
- 縦安全の作動状況を併記し、作動した場合だけLiDAR制動と表記
- 接触・wall・stallなしで3周完走

</div>
</div>

---

## Closed-loop評価

| Gate | 条件 | 結果 |
|---|---|---:|
| packaged単車 | 3周、既定設定 | **252.230秒 / penalty 0 / stall 0** |
| NPC seed 2035 | 3周、学習未使用の開始seed | **256.488秒 / 1位 / penalty 0** |
| NPC seed 2036 | 3周、学習未使用の開始seed | **255.873秒 / 1位 / penalty 0** |
| packaged NPC seed 2037 | 環境overrideなし | **255.648秒 / 1位 / penalty 0** |
| controllable peer | 2 MPC peer + 1 E2E student | **不合格: low-speed 54.914秒** |

- NPC 3 seedの3周合計タイム最大–最小幅は`0.840秒`
- runtime推論error/staleなし、モデルSHA一致
- peer不合格は提出で隠さず、動的相互作用の残課題として扱う

---

## 工夫・独自性

1. **Frozen-base spatial residual**<br>
   通常走行を固定し、障害物回避だけを追加学習
2. **Outcome-gated supervision**<br>
   「教師が出した指令」ではなく、走り切ったrunだけを証拠化
3. **Run-disjoint evaluation**<br>
   同一走行のsample漏洩を禁止し、未見seedでclosed-loop評価
4. **Immutable artifact identity**<br>
   source/install/提出tarのSHAを照合
5. **Failure-first audit**<br>
   成功動画だけでなく、接触・stallを上流原因まで分類

---

## 学んだことと残課題

### 得られたこと

- 単一scanの距離閾値だけでは、動く相手のside選択を安定化できない
- 時系列モデルは容量を増やすだけでは改善せず、教師の成功証明が重要
- NPC回避は3 seedで再現したが、対称なpeer相互作用は別の難しさを持つ

### 次に取り組むこと

- course-following successorを持つ成功回避実演の収集
- 反実仮想の時系列占有を表現できる教師設計
- peer混走をFinish／penalty 0／stall 0で独立評価

**現時点の判定: 単車・NPC候補は合格、peer混走は未合格。**

---

<!-- _class: lead -->

# 走行動画

**[動画URL／QRコード／代表フレームを配置]**

- packaged production
- 2D LiDAR + wheel speedのみ
- ML lateral authority
- NPC seed 2037、3周、1位、penalty/stall 0

<p class="small muted">数値の正本: output/20260902-e2e-bounded-pace-packaged-seed2037</p>
