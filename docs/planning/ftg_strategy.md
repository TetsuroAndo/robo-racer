# FTG戦略（安全優先・競技用設計）

本書は、現行アルゴリズム（`Process::proc()` が「指定角度範囲内の最遠/最短で速度・舵角を決定」）を前提に、
**FTG（Follow-The-Gap）を中核に据えた競技実装**へ自然に移行するための設計案をまとめたものです。
安全策（壁ヒット／スタック／蛇行の最小化）を最優先とし、最短で走り切るための構成とパラメータを定義します。

---

## 1. 前提とスコープ

- 走行は **1周目はSLAM無し**（ローカル判断のみ）
- LiDAR 取り付け高さ **約11cm**
- FOV は **-90°〜+90°** を当面固定
- 車幅は **可変**（TT-02Dのシャーシ幅は 182/190mm 可変）
- 現行パイプライン：`LidarReceiver -> Process -> Sender`

---

## 2. 結論：3層分離でFTGを差し替え可能にする

現行構造を壊さず、以下の3層で責務分離します。

- **Perception**：点群 → 角度ビン距離配列
- **Planning**：FTG等の意思決定（plan）
- **Commanding**：速度/舵角 → 送信プロトコル

これにより、
- 1周目：FTG
- 2周目以降：SLAM + Global + PurePursuit
への移行が最小変更で可能になります。

---

## 3. 推奨ディレクトリ構成（RPi側）

最小の移動で責務分割する設計です。

```
rpi/src/
  app/
    main.cpp                # ループ、signal、依存の組み立て
  perception/
    LidarReceiver.{h,cpp}   # LiDAR SDKアクセス（現状維持）
    ScanPreprocessor.{h,cpp}# 点群→角度ビン距離配列、外れ値処理
  planning/
    IPlanner.h              # interface: plan(scan)->Command
    PlannerFTG.{h,cpp}      # FTG本体
    PlannerBaseline.{h,cpp} # 既存 Process 相当（比較用）
    planning_types.h        # Scan/Command/Debug 等の型
  control/
    SpeedPolicy.{h,cpp}     # d/前方余裕/操舵で速度決定
    SteeringSmoother.{h,cpp}# ヒステリシス/レート制限
  comm/
    Sender.{h,cpp}          # 送信のみ（現状維持）
  config/
    Config.h                # 既存
    Params.h                # 競技パラメータ（車幅、FOV、閾値等）
```

ポイント：
- `Process` は **PlannerBaseline** として残し、FTGと比較できるようにする
- 車幅やFOV等の調整値は **`Params` に集約**して運用で変更可能にする

---

## 4. データモデル（角度ビン距離配列へ正規化）

FTGを安定して実装するため、入力は必ず **等角度配列**に正規化します。

### Scan（Planning入力）

- `angle_min_deg = -90`
- `angle_max_deg = +90`
- `angle_step_deg = 1.0`（例）
- `ranges_mm[i]`：その角度ビンの代表距離（未観測は `planning::kInvalidDistance` でマークし、FTG内では障害物扱いにして “疑わしい自由領域” を吐かない）

正規化により以下が O(N) で実装可能になります。
- smoothing（移動平均/中央値）
- obstacle inflation（車幅反映）
- gap detection（連続区間探索）

---

## 5. 車幅を可変にする核心：Inflation（角度方向膨張）

車幅が未確定でも、**角度方向の障害物膨張**として扱うのが最も合理的です。

- `half_width = vehicle_width_mm/2 + safety_margin_mm`
- 距離 `r` の障害物に対して塞ぐべき角度幅
  `delta = atan2(half_width, r)`
- `delta` をビン数へ変換し、`i±k` を危険扱いにする

これにより **`vehicle_width_mm` を変えるだけで挙動が変わる**構成になります。

---

## 6. PlannerFTG の内部ステップ（安全策込み）

### (1) 前処理
- 角度ビン化（最小距離採用）
- 欠測は `planning::kInvalidDistance` でマークし free 判定から除外（保守的な安全策）
- 平滑化（移動中央値/平均）

### (2) 前方安全距離
- `d_front_min` を前方±`front_sector_deg`から取得
- `stop_distance_mm` を下回れば即停止

### (3) Safety Bubble
- 最短距離点の周囲を一定半径でマスク

### (4) Disparity Extender（安全側に強め）
- `abs(r[i]-r[i+1]) > disparity_threshold` を検出
- 近い側の距離で左右に塗りつぶす

### (5) Gap抽出
- `ranges_mm >= free_threshold_mm` の連続区間を抽出
- 最大区間（または前方優先）を選ぶ

### (6) Aim Point 選択
- gap中心寄りの **最遠点** を選ぶ
- 中心から外れ過ぎる場合は中心寄りへ補正

### (7) Steering 安定化
- IIR（例：`beta=0.25`）
- 1周期の操舵変化量に上限
- 前回ターゲット近傍を優先

### (8) 速度政策
- `d_target`（狙い方向の距離）だけでなく、
  `d_front_min` と `|steer|` で上限を切る

---

## 7. 速度政策（安全側・d中心の設計）

基本方針：
- **安全上限**：`d_front_min` で決定
- **攻め**：`d_target` で上げる
- **操舵制限**：`|steer|` が大きいほど減速

例：

- `v_limit_front = clamp(k1 * (d_front_min - stop_distance_mm), 0, V_MAX)`
- `v_limit_steer = clamp(V_MAX - k2 * |steer|, 0, V_MAX)`
- `v_cmd = min(k3 * d_target, v_limit_front, v_limit_steer)`

さらに、急加速/急減速を避けるため **速度変化量を制限**します。

---

## 8. 初期パラメータ（安全側スタート）

安全優先の初期値です。ここから攻めに寄せます。

### Geometry
- `fov_min_deg = -90`
- `fov_max_deg = +90`
- `angle_step_deg = 1.0`
- `vehicle_width_mm = 190`
- `safety_margin_mm = 30`

### Ranges
- `range_min_mm = 150`
- `range_max_mm = 6000`

### Gap / Inflation
- `free_threshold_mm = 700`
- `bubble_radius_mm = 250`
- `disparity_threshold_mm = 300`

### Speed / Safety
- `stop_distance_mm = 300`
- `front_sector_deg = 15`
- `steer_slowdown_gain = 強め`
- `accel_limit = 有効（急加速禁止）`

---

## 9. テスト戦略

### Planner 単体テスト
- `test/*/test_planner_ftg.cpp`
- 入力：人工 `ranges_mm`（壁、廊下、斜め障害物）
- 期待：gap選択・停止判定・車幅変更での挙動差

### 実機テスト（走行前）
- `seriald` 起動
- `serialctl drive ...` で操舵・速度疎通
- `serialctl listen` でSTATUS確認

---

## 10. チューニング順（安全→攻め）

1. `vehicle_width_mm=190` + `safety_margin_mm=30` で安定走行
2. 速度を上げたい場合：
   - `free_threshold_mm` を下げる（例：700→600）
   - `safety_margin_mm` を下げる（例：30→20）
   - `vehicle_width_mm` を 190→182 にする（最後）

---

## 11. 追加で確認したいこと

- **ESP32側の速度上限値**（`SPEED_MM_S_MAX` 相当）
- **LiDARの左右オフセット**（中心線上かどうか）

これが分かれば、安全策のまま **FTG実装詳細の差分設計**まで一気に確定できます。

---

## 12. 次の作業（差分ベースの実装案）

- `Process` を `PlannerBaseline` へ移植
- `PlannerFTG` / `ScanPreprocessor` / `SpeedPolicy` の雛形追加
- `main` で `--planner=baseline|ftg` の切替

必要なら、この方針を **コード差分として具体化**して提示します。
