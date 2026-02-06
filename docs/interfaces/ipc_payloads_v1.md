# RPi内 IPC Payload 定義 v1（Interface Freeze）

## 目的
RPi内IPCで運搬する payload の**フィールド/単位/意味**を凍結し、
タスクB（ルールベース/SLAM/安全）とタスクC（AI/データ/推論入口）の実装で解釈違いを防止する。

## 正本
- `shared/proto/include/mc_proto.h`（Single Source of Truth）
- `docs/interfaces/ipc_topics_v1.md`（トピック/頻度/Drop方針）

## 共通ルール（v1固定）
- IPCは **mcprotoフレーム（COBS+CRC）** をそのまま運ぶ
- 各payloadのサイズは **MAX_PAYLOAD=64** を超えない
- `*_le` のフィールドは**little-endian前提**（IPC内でも変換関数経由）
- `ts_ms` は **monotonic time**（`mc::core::Time::ms()`）
- 予約フィールド/予約ビットは **0固定**（v1）

---

## LIDAR_SCAN: `IPC_LIDAR_SCAN`
### `LidarScanChunkPayload`（14 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | 取得時刻（monotonic） |
| `scan_id` | u16 | - | 同一スキャンを識別（wrap可） |
| `angle_start_cdeg` | i16 | 0.01° | チャンク先頭角度（車体前方=0） |
| `angle_step_cdeg` | i16 | 0.01° | 連続点の角度ステップ |
| `chunk_index` | u8 | - | 0..chunk_count-1 |
| `chunk_count` | u8 | - | スキャン内の総チャンク数 |
| `point_count` | u8 | - | このチャンクに含まれる点数 |
| `encoding` | u8 | - | 0=uint16 mm |

### 後続データ
- `point_count` 個の `uint16` 距離（mm）
- 角度は `angle_start_cdeg + angle_step_cdeg * i`

**角度の符号**
- v1: 車体前方=0、左回り（反時計回り）を正とする
- 実機座標が異なる場合は **lidard側で補正**する

---

## LIDAR_SUMMARY: `IPC_LIDAR_SUMMARY`
### `LidarSummaryPayload`（14 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | 取得時刻（monotonic） |
| `best_heading_cdeg` | i16 | 0.01° | 推奨進行方向（操舵角と同符号） |
| `best_distance_mm` | u16 | mm | 推奨方向の余裕距離 |
| `min_distance_mm` | u16 | mm | 視野内の最小距離 |
| `min_distance_heading_cdeg` | i16 | 0.01° | 最小距離方向 |
| `confidence` | u8 | 0-255 | 推奨方向の信頼度（0=無効） |
| `flags` | u8 | bit | 予約（v1は0固定） |

---

## IMU_SAMPLE: `IPC_IMU_SAMPLE`
### `ImuSamplePayload`（16 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | 取得時刻（monotonic） |
| `ax_mg` | i16 | mg | 加速度X |
| `ay_mg` | i16 | mg | 加速度Y |
| `az_mg` | i16 | mg | 加速度Z |
| `gx_mdps` | i16 | mdps | 角速度X |
| `gy_mdps` | i16 | mdps | 角速度Y |
| `gz_mdps` | i16 | mdps | 角速度Z |

---

## DRIVE_CMD: `IPC_DRIVE_CMD`
### `DriveCmdPayload`（12 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | 生成時刻（monotonic） |
| `steer_cdeg` | i16 | 0.01° | 目標舵角 |
| `speed_mm_s` | i16 | mm/s | 目標速度（前進正。物理一致は前進のみ。範囲は `mc_config::SPEED_MAX_MM_S` にクランプ） |
| `ttl_ms` | u16 | ms | 有効期限（ESP32受理時刻基準） |
| `source` | u8 | enum | 生成元（下表） |
| `flags` | u8 | bit | 予約（v1は0固定） |

**source（v1固定）**
- 0: `SOURCE_UNKNOWN`
- 1: `SOURCE_FTG`
- 2: `SOURCE_AI`
- 3: `SOURCE_SHADOW`
- 4: `SOURCE_REPLAY`
- 5: `SOURCE_MANUAL`

---

## VEHICLE_STATUS: `IPC_VEHICLE_STATUS`
### `VehicleStatusPayload`（14 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | 受信時刻（monotonic） |
| `status` | `StatusPayload` | - | ESP32 STATUS をそのまま格納 |

### `StatusPayload`（10 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `seq_applied` | u8 | - | 最後に適用した `DRIVE.seq` 下位8bit |
| `auto_active` | u8 | - | 1=AUTO, 0=MANUAL |
| `faults_le` | u16 | bit | fault bitfield（LE） |
| `speed_mm_s_le` | i16 | mm/s | 現在速度（LE） |
| `steer_cdeg_le` | i16 | 0.01° | 現在舵角（LE） |
| `age_ms_le` | u16 | ms | 最終DRIVE受理からの経過（LE） |

---

## METRICS: `IPC_METRICS`
### `MetricsPayload`（16 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | 取得時刻（monotonic） |
| `cpu_temp_cdeg` | u16 | 0.01°C | CPU温度 |
| `cpu_usage_permille` | u16 | ‰ | CPU使用率（0-1000） |
| `mem_used_kb` | u32 | KB | 使用メモリ |
| `mem_total_kb` | u32 | KB | 総メモリ |

---

## LOG_RECORD: `IPC_LOG_RECORD`
### `LogRecordPayload`（8 bytes）
| フィールド | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `ts_ms` | u32 | ms | 生成時刻（monotonic） |
| `level` | u8 | - | ログレベル |
| `text_len` | u8 | bytes | 後続テキスト長 |
| `flags` | u8 | bit | 予約（v1は0固定） |
| `reserved` | u8 | - | 予約（0固定） |

### 後続データ
- `text_len` bytes の UTF-8 テキスト
- NUL終端はしない
