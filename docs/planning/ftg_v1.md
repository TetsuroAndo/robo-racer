# FTG 実装計画 v1（タスクB）

## 目的
- LiDAR入力から **安全かつ決定論的** に舵角/速度を生成する
- TTL前提の安全設計に合わせて **一定周期で DRIVE_CMD を生成**する

## 正本/参照
- IPCトピック: `docs/interfaces/ipc_topics_v1.md`
- IPC payload定義: `docs/interfaces/ipc_payloads_v1.md`
- RPi↔ESP32プロトコル: `docs/proto/overview.md`

---

## 入力
- `IPC_LIDAR_SUMMARY`（10-50Hz）
- `IPC_LIDAR_SCAN`（5-20Hz、補助）
- `IPC_VEHICLE_STATUS`（10-50Hz）
- `IPC_IMU_SAMPLE`（50-200Hz、任意）

**優先順位**
1. `LIDAR_SUMMARY` があればそれを使用
2. 無い場合は `LIDAR_SCAN` から gap を計算

---

## 出力
- `IPC_DRIVE_CMD`（20-100Hz）
  - `steer_cdeg`, `speed_mm_s`, `ttl_ms`
  - `source=SOURCE_FTG`

**TTL/周期**
- 送信周期: 20-50ms（=20-50Hz）
- `ttl_ms`: 送信周期の2-3倍（例: 50ms周期なら ttl=100-150ms）

---

## 処理フロー（決定論）
1. **入力スナップショット**
   - 最新の `LIDAR_SUMMARY` もしくは `LIDAR_SCAN` を固定
   - `VEHICLE_STATUS` の `auto_active` / fault を確認

2. **前処理**
   - 距離0や閾値未満点を除外
   - 角度範囲を `PROCESS_ANGLE_MIN/MAX` に制限

3. **gap抽出**（scanを使う場合）
   - 一定距離以上の連続領域をgapとして抽出
   - gap中心の角度を `best_heading_cdeg` とする

4. **操舵・速度決定**
   - `steer_cdeg = clamp(best_heading_cdeg)`
   - `speed_mm_s` は距離や角度に応じて減速

5. **安全判定**
   - confidenceが低い/障害物が近い場合は `speed_mm_s=0`
   - AUTOでない場合は **出力しない**（または0指令）

6. **平滑化**
   - 舵角・速度にスルーレート制限を適用

---

## パラメータ（v1）
`rpi/src/config/Config.h` のデフォルト値を基準とする。

| パラメータ | 目的 | 既定値/根拠 |
| --- | --- | --- |
| `PROCESS_ANGLE_MIN_DEG` | 走行視野の左限 | -70.0 deg |
| `PROCESS_ANGLE_MAX_DEG` | 走行視野の右限 | 70.0 deg |
| `SPEED_MM_S_MAX` | 速度上限 | 2000 mm/s |
| `STEER_CDEG_MAX` | 舵角上限 | 2500 cdeg |
| `AUTO_TTL_MS` | TTL既定 | 100 ms |

**追加パラメータ（FTG側で定義）**
- `min_clearance_mm`（gap判定閾値）
- `slowdown_distance_mm`（減速開始距離）
- `stop_distance_mm`（停止距離）
- `steer_slew_cdeg_per_s`（舵角スルーレート）
- `speed_slew_mm_s2`（速度スルーレート）

---

## 失敗時の挙動
- gap無し/低信頼 → `speed_mm_s=0`、`steer_cdeg=0`
- 入力欠損が一定期間続く → **STOP** を維持
- `VEHICLE_STATUS` に fault が立っている場合は **保守的な停止**

---

## DoD
- 동일入力で**決定論的**に同じ出力が出る
- 20-50ms周期で `DRIVE_CMD` を生成できる
- TTLが常に有効（失効前に更新）
- `LIDAR_SUMMARY` あり/なし両方で動作

---

## テスト
- 単体: 既知のscan/summary入力に対して出力が固定
- 統合: sim_esp32d + seriald + racerd で机上E2E
