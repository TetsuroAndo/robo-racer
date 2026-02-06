# ABSロジック簡略化プラン

## 1. 現状の問題

### 1.1 報告されている症状

| 症状 | 説明 |
|------|------|
| 壁が近くないのに abs=1 | TSD20 が障害物を検出していない状況でも ABS が発動する |
| 手で動かしただけでも abs=1 | 手動で車体を動かしただけで ABS が発動する |
| 後退し続ける | 逆転ブレーキにより後退 → ループして後方にずっと進む |

### 1.2 根本原因の整理

現在の ABS 発動条件は **IMU のみ** に依存している：

```
want_brake = (v_est > v_cmd + ABS_SPEED_MARGIN_MM_S)
```

- **TSD20 は発動条件に使っていない**（近いときの逆転抑制・スキップにのみ使用）
- `v_est` は IMU の積分推定で、ノイズ・ドリフト・手動移動で容易に上振れする
- `v_cmd` が 0 や低いとき、わずかな `v_est` の上振れで `want_brake` が true になる
- 逆転（`-ABS_REVERSE_MM_S`）が強く、後退が過剰になりやすい

---

## 2. 簡略化の方針

### 2.1 設計原則

1. **TSD20 必須**: ABS は TSD20 が障害物を検出しているときのみ発動する
2. **逆転を廃止または極限まで弱くする**: 後退ループを防ぐ
3. **条件を少なくする**: 複雑なスキップ・hold ロジックを削減

### 2.2 発動条件の変更

| 項目 | 現状 | 変更後 |
|------|------|--------|
| 主トリガ | `v_est > v_cmd + margin`（IMU のみ） | **TSD20 が近距離を検出** かつ `v_est > v_cmd` |
| TSD20 | 逆転抑制・スキップ用 | **発動の必須条件** |
| 逆転 | 断続的に -1200 mm/s 出力 | **廃止** または 惰行（0）のみ |

---

## 3. 実装プラン

### Phase 1: TSD20 必須条件の追加（最小変更）

**目的**: 壁が近くないときの誤発動を防ぐ

**変更内容**:
- ABS 発動前に `tsd.valid && tsd.mm <= 閾値` を必須にする
- 閾値例: `TSD20_MARGIN_MM + 200`（例: 320mm 以下で発動許可）

**効果**: 壁が近くないのに abs=1 になる問題を解消

---

### Phase 2: 逆転の廃止または惰行のみ

**目的**: 後退し続ける問題を解消

**オプション A（推奨）: 逆転を廃止**
- ABS 発動時は **惰行（speed=0）** のみ
- `brake_mode` は維持（PWM を 0 にする等のブレーキ挙動はそのまま）
- 逆転出力（`-ABS_REVERSE_MM_S`）を完全に削除

**オプション B: 逆転を極限まで弱く**
- 逆転を残す場合、`ABS_REVERSE_MM_S` を 200〜400 程度に下げる
- または TSD20 が非常に近い（例: 150mm 以下）ときのみ逆転、それ以外は惰行

**推奨**: オプション A。後退ループの根本を断つ。

---

### Phase 3: 手動移動時の誤発動防止

**目的**: 手で動かしただけでも abs=1 になる問題を防ぐ

**対策**:
1. **v_cmd の下限チェック**: `v_cmd` が 0 または非常に小さい（例: 200 mm/s 未満）ときは ABS を発動させない
   - 手動移動時は通常 `v_cmd=0` なので、この条件で大部分をカットできる
2. **v_est の最小閾値**: `v_est` が一定以上（例: 300 mm/s）のときのみ発動
   - ノイズや微小な手動移動による誤検出を抑制

---

### Phase 4: ロジックの整理・削減

**削除・簡略化する要素**:

| 要素 | 現状 | 変更後 |
|------|------|--------|
| `_hold_until_ms` | want_brake から 200ms hold | 削除または大幅短縮（50ms 程度） |
| `ABS_SKIP_V_CMD_MM_S` / `ABS_SKIP_DECEL_MM_S2` | 複雑なスキップ条件 | 削除（Phase 3 の v_cmd チェックで代替） |
| `ABS_REVERSE_DISABLE_NEAR_MM` / `ABS_REVERSE_DISABLE_MARGIN_MM` | 逆転強度の段階的調整 | 逆転廃止なら削除 |
| PI 制御（duty, a_target, decel） | 複雑な duty 計算 | 逆転廃止なら固定 duty または削除 |
| `ABS_PERIOD_MS` による断続 | 逆転の on/off 周期 | 逆転廃止なら不要 |

---

## 4. 簡略化後の擬似コード

```
apply():
  if (!ABS_ENABLE || !allow_abs) return speed_mm_s;
  if (!imu_valid || (REQUIRE_CALIB && !imu.calibrated)) return speed_mm_s;
  if (speed_mm_s < 0) return speed_mm_s;

  v_cmd = max(0, speed_mm_s)
  v_est = max(0, imu.v_est_mm_s)

  // 発動条件を厳格化
  if (v_est <= 0) return speed_mm_s;
  if (v_cmd >= ABS_V_CMD_MIN_MM_S) return speed_mm_s;  // 目標が十分あるときは発動しない
  if (v_est < ABS_V_EST_MIN_MM_S) return speed_mm_s;   // 微小速度は無視

  // TSD20 必須: 障害物が近いときのみ
  if (!tsd.valid || tsd.mm > ABS_TSD_TRIGGER_MM) return speed_mm_s;

  // 減速したいか
  want_brake = (v_est > v_cmd + ABS_SPEED_MARGIN_MM_S)
  if (!want_brake) return speed_mm_s;

  // 惰行のみ（逆転なし）
  *active_out = true;
  return 0;  // brake_mode は true、速度は 0
```

---

## 5. Config の変更案

| 定数 | 現状 | 変更後 |
|------|------|--------|
| `ABS_REVERSE_MM_S` | 1200 | 0（逆転廃止）または削除 |
| `ABS_TSD_TRIGGER_MM` | （新規） | 320（TSD20_MARGIN + 200 相当） |
| `ABS_V_CMD_MIN_MM_S` | （新規） | 200（これ以上なら発動しない） |
| `ABS_V_EST_MIN_MM_S` | （新規） | 300（これ未満は無視） |
| 削除候補 | `ABS_REVERSE_DISABLE_*`, `ABS_SKIP_*`, `ABS_PERIOD_MS`, `ABS_DUTY_*`, `ABS_BRAKE_HOLD_MS` 等 | 逆転廃止なら多数削除可能 |

---

## 6. 実装順序

1. **Phase 1**: TSD20 必須条件を追加 → 壁なし誤発動を解消
2. **Phase 2**: 逆転を廃止（惰行のみ） → 後退ループを解消
3. **Phase 3**: v_cmd / v_est の最小閾値を追加 → 手動移動誤発動を解消
4. **Phase 4**: 不要な定数・ロジックを削除してコードを整理

---

## 7. テスト観点

- 壁が遠い状態で手で動かす → abs=0 のまま
- 壁が近い状態で減速指令 → abs=1、惰行で停止（後退しない）
- 壁が近い状態で前進指令 → TSD20 クランプが先に効く想定、ABS は補助

---

## 8. ステータス

| Phase | 内容 | 状態 |
|-------|------|------|
| 1 | TSD20 必須条件 | 完了 |
| 2 | 逆転廃止 | 完了 |
| 3 | v_cmd / v_est 閾値 | 完了 |
| 4 | ロジック整理 | 完了 |
