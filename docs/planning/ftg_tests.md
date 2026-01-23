# FTG Safety Regression Test Plan

この文書では、現行の `perception/ScanPreprocessor` → `planning/Planner*` → `control/*` パイプラインに対して、
**安全優先 FTG を壊さずに回帰を防ぐ**ためのテスト設計を示します。
テストのゴールはアルゴリズムの正しさではなく、**安全策の不変条件と境界条件**を守ることです。

---

## 1. テスト方針（何を保証するか）

以下の不変条件は最優先でテストに残す必要があります。

1. **Stop判定**：`front_min_distance_mm <= stop_distance_mm` なら `PlanOutput::stop` が true であり、速度指示が 0 になること。
2. **車幅（inflation）の効果**：`vehicle_width_mm`/`safety_margin_mm` を増やすと gap が狭まり、操舵が中心寄りか停止気味になる。
3. **欠測/INF/外れ値耐性**：入力に欠測点や INF（未観測）値が含まれてもクラッシュや範囲外アクセスが起きない。
4. **速度上限の縛り**：`d_front_min` や `|steer|` に基づく `SpeedPolicy` の上限が必ず効いて暴走しない。
5. **ステア急変の抑制**：`SteeringSmoother` が IIR＋レート制限で急変を抑える。

---

## 2. テストレイヤ構成（単体→結合→回帰）

| レイヤ | 対象 | 狙い |
|--------|------|------|
| 単体（Layer A） | PlannerFTG/PlannerBaseline/ScanPreprocessor/SpeedPolicy/SteeringSmoother | 各モデルの境界・仕様を固定し、微調整で安全性が崩れないようにする |
| 結合（Layer B） | `ScanPreprocessor -> PlannerFTG -> SteeringSmoother -> SpeedPolicy`（SenderはFake） | end-to-end で stop・steer・speed が一貫することを確認 |
| 回帰（Layer C） | 保存済み「ゴールデン」スキャン | 代表的な状況で steer/stop/speed が逸脱していないか監視 |

---

## 3. テストフレームワーク候補

- **推奨**：`doctest`（ヘッダオンリーで導入が軽く、既存CMakeに組み込みやすい）
- **代替**：`Catch2 v3`（やや重いが充実したマクロ群）

※GoogleTestは依存が多いため現状ではやや重い。doctestだと `third_party/doctest/doctest.h` を置くだけで librosa できる。

---

## 4. テストディレクトリ + CMake 案

```
rpi/tests/
  CMakeLists.txt
  test_scan_preprocessor.cpp
  test_planner_baseline.cpp
  test_planner_ftg_stop.cpp
  test_planner_ftg_gaps.cpp
  test_planner_ftg_inflation.cpp
  test_speed_policy.cpp
  test_steering_smoother.cpp
  test_pipeline_integration.cpp
  fixtures/
    scans/
      corridor.json
      chicane.json
      left_turn.json
      open_space.json
```

CMakeは `add_executable(rpi_tests ...tests...)` → `target_link_libraries(rpi_tests robo_racer_core)` のように本体ロジックを静的ライブラリ化したほうが望ましい。
現在は `src` に `main()` もあるので、`core`（ライブラリ）と `app`（実行ファイル）に切り分けてテストで `core` をリンクするのがベスト。

---

## 5. 単体テスト設計（クラス別）

### 5.1 ScanPreprocessor
- bin数・角度範囲が `params` に従う
- 同じビンに複数点→最小距離を採用
- `range_min`・`range_max` で除外され INF になる
- FOV外の点は無視
- 未観測ビンは `planning::kInvalidDistance` でマークし、gap 抽出時には free として扱わない

### 5.2 PlannerBaseline
- 最大距離が `target_distance_mm`
- 最短距離が steer に反映（符号込み）
- 全て INF → stop

### 5.3 PlannerFTG: Stop 判定
- 前方±`front_sector_deg` の最小距離≤`stop_distance_mm` → stop
- それ以上 → ただし gap が存在する限り stop=false
- `front_sector_deg` の角度範囲が正しく反映される

### 5.4 PlannerFTG: Gap 検出/選択
- 一つの大きな gap で best_idx は最大距離
- 2つの gap で span が大きい方を選ぶ
- 距離スコアが勝つケース（span + best_distance/1000）
- gap が存在しない → stop

### 5.5 PlannerFTG: Inflation
- `vehicle_width`/`margin` を増やすと gap が減少/stop になりやすい
- 近距離障害物ほどビン展開が大きい（短い距離で塞がれる範囲が広い）

### 5.6 SpeedPolicy
- `PlanOutput::stop` で 0
- `front_min` 制約が効く
- `|steer|` による減速
- `accel_limit` で Δv が制限
- `SPEED_MM_S_MAX`/`SPEED_INPUT_LIMIT` で clamp

### 5.7 SteeringSmoother
- 初回は raw を即追従
- IIR (`beta`) で段階的追従
- rate_limit（例：10 deg）で Δθ を制限
- 符号反転時も極端な跳ね返りがない

---

## 6. 結合テスト設計

### 6.1 Pipeline（Senderなし）
- `ScanPreprocessor -> PlannerFTG -> SteeringSmoother -> SpeedPolicy`
- 期待：stop なら speed=0、steerはFOV内
- パラメータ（width/margin）を安全側に寄せたケースでも speed が増えない

### 6.2 Pipeline（Fake Sender）
- `FakeSender` を作り最後に送ろうとした speed/steer を保持
- 期待：出力が `cfg::SPEED_INPUT_LIMIT`/`cfg::STEER_CDEG_MAX` を越えない

---

## 7. 回帰テスト（Golden Scan Fixtures）

代表的スキャンを fixtures に保存し、出力が想定から逸脱しないかを常時比較。

- corridor.json（直線）
- chicane.json（S字）
- left_turn.json（90°ターン）
- open_space.json（幅広い空間）

出力監視項目：`steer_deg`/`stop`/`target_distance_mm`/`front_min_distance_mm`/`speed_input`

### 形式（暫定）
テスト導入を軽くするため、当面は `*.txt` を使い、以下の形式で読み込む。

- `# angle_min=...` / `# angle_step=...` / `# expected_stop=...` / `# expected_steer=...` / `# expected_speed_hint=...`
- 以降は1行1距離（mm）
※現在の fixtures は `angle_min=-90` / `angle_step=1` の 181 ビン（-90〜+90）で統一。

---

## 8. 実装上の細かな工夫

- `PlannerFTG` 内の `inflateObstacles`/`findGaps` を `detail` 名前空間の free function に切り出すと単体テストが楽
- 現在は `plan()` だけでブラックボックステストする形でも十分。ただし inflation の効果を正確に捉えるには内部関数へのアクセスがあると望ましい

---

## 9. 確認事項：速度上限値

`cfg::SPEED_MM_S_MAX` と `cfg::SPEED_INPUT_LIMIT` の値を教えてください。テストの期待値（speed_input が 0～limit に収まるなど）に使います。変動する場合は「limit レンジ内」の検証に留める方針も可能です。

---

必要なら次の返答で `doctest` 前提の `test_planner_ftg_stop.cpp` および `test_speed_policy.cpp` の雛形コードまで用意します。
