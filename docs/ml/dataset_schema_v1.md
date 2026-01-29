# Dataset Schema v1（タスクC）

## 目的
- ログから**再現可能に学習データを生成**する
- 学習/評価の入力が**同じフォーマット**になることを保証する

## 正本/参照
- IPC payload: `docs/interfaces/ipc_payloads_v1.md`
- ログ仕様: `docs/observability/logging_v1.md`
- プロトコル: `docs/proto/overview.md`

---

## 入力ソース
- `IPC_LIDAR_SCAN` / `IPC_LIDAR_SUMMARY`
- `IPC_DRIVE_CMD`（教師ラベル）
- `IPC_VEHICLE_STATUS`
- `IPC_IMU_SAMPLE`
- `LOG_RECORD`（イベント/状態遷移）

---

## データセット構成（推奨）
```
training/data/datasets/<dataset_id>/
  meta.json
  frames.parquet
  lidar_scan.parquet   (任意)
  events.parquet       (任意)
```

### meta.json（必須）
- `dataset_id`
- `schema_version: "v1"`
- `source_logs`: 入力ログの一覧
- `time_range_ms`: [start, end]
- `lidar_frame`: 角度基準（0=前方）
- `notes`

---

## frames.parquet（必須）
**基準タイムスタンプ**: `ts_ms`（monotonic）

| カラム | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | フレーム時刻 |
| `best_heading_cdeg` | i16 | 0.01° | LIDAR_SUMMARY | 
| `best_distance_mm` | u16 | mm | LIDAR_SUMMARY |
| `min_distance_mm` | u16 | mm | LIDAR_SUMMARY |
| `min_distance_heading_cdeg` | i16 | 0.01° | LIDAR_SUMMARY |
| `confidence` | u8 | - | LIDAR_SUMMARY |
| `ax_mg..gz_mdps` | i16 | mg/mdps | IMU |
| `auto_active` | u8 | - | VEHICLE_STATUS |
| `faults` | u16 | bit | VEHICLE_STATUS |
| `speed_mm_s` | i16 | mm/s | VEHICLE_STATUS |
| `steer_cdeg` | i16 | 0.01° | VEHICLE_STATUS |
| `age_ms` | u16 | ms | VEHICLE_STATUS |
| `cmd_steer_cdeg` | i16 | 0.01° | DRIVE_CMD |
| `cmd_speed_mm_s` | i16 | mm/s | DRIVE_CMD |
| `cmd_ttl_ms` | u16 | ms | DRIVE_CMD |
| `cmd_source` | u8 | enum | DRIVE_CMD |

**欠損の扱い**
- センサ欠損は `null` にする
- 学習時は **前方補間 or 直近値ホールド**を選択

**サンプリング**
- `DRIVE_CMD` を基準（20-50Hz）
- 近傍 `ts_ms` のセンサ値を結合（許容遅延は±1周期）

---

## lidar_scan.parquet（任意）
- scan をチャンク再構成した**生データ**
- 1フレーム1スキャンを行として保存

| カラム | 型 | 説明 |
| --- | --- | --- |
| `ts_ms` | u32 | スキャン時刻 |
| `angles_cdeg[]` | i16[] | 角度配列 |
| `ranges_mm[]` | u16[] | 距離配列 |

---

## events.parquet（任意）
- MODE_SET/KILL/状態遷移などのイベントログ

| カラム | 型 | 説明 |
| --- | --- | --- |
| `ts_ms` | u32 | 時刻 |
| `event` | string | 例: MODE_SET, KILL |
| `value` | string | 付随情報 |

---

## 学習用ラベル
- `DRIVE_CMD` を教師信号とする
- `cmd_source` により **manual/ftg/ai** を区別

---

## バージョニング
- `schema_version` を固定（v1）
- 互換性を壊す変更は v2 へ
