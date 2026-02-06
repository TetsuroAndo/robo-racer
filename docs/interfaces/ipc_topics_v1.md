# RPi内 IPC トピック v1（Interface Freeze）

## 方針
- RPi内IPCは **mcprotoフレーム（COBS+CRC）をそのまま運搬**する
- エンコード/デコード資産を共通化するため、`shared/proto` の型を正本とする
- **MAX_PAYLOAD=64** を前提に、サイズ超過データはチャンク化して送る

## 送受信経路（推奨）
- UDS `SOCK_SEQPACKET`
- sender/receiver は `mc::proto::PacketWriter/PacketReader` を利用

## トピック一覧（v1）
| Topic | mcproto Type | payload | 方向 | 目安頻度 | Drop方針 |
| --- | --- | --- | --- | --- | --- |
| LIDAR_SCAN | `IPC_LIDAR_SCAN` | `LidarScanChunkPayload` + data | lidard -> consumers | 5-20Hz | 古いscan_idを破棄 |
| LIDAR_SUMMARY | `IPC_LIDAR_SUMMARY` | `LidarSummaryPayload` | lidard/ftg -> consumers | 10-50Hz | 最新優先 |
| IMU_SAMPLE | `IPC_IMU_SAMPLE` | `ImuSamplePayload` | imu -> consumers | 50-200Hz | 最新優先 |
| DRIVE_CMD | `IPC_DRIVE_CMD` | `DriveCmdPayload` | racerd -> seriald | **100Hz（10ms）** | 最新優先 |
| VEHICLE_STATUS | `IPC_VEHICLE_STATUS` | `VehicleStatusPayload` | seriald -> consumers | **100Hz（10ms）** | 最新優先 |
| METRICS | `IPC_METRICS` | `MetricsPayload` | metricsd -> consumers | 1-5Hz | 最新優先 |
| LOG_RECORD | `IPC_LOG_RECORD` | `LogRecordPayload` | any -> logd | 1-100Hz | drop可（非同期） |

## チャンク仕様（LIDAR_SCAN）
- `LidarScanChunkPayload` の `chunk_index/chunk_count` で同一scanを再構成する
- `point_count` は **このチャンク内の点数**（1点=uint16 mm）
- `encoding=0` は `uint16` の距離配列（mm）
- 再構成は **同一scan_id** をキーに結合する

## サイズ制約
- 1フレームあたり `payload_len <= 64`
- チャンク化時の推奨: 1チャンク 20〜25点

## 互換性ルール
- 未知のtypeは破棄（必要ならログ）
- `payload_len` 不一致は破棄
- v1で追加するpayload構造体のサイズを変えない
