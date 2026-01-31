# ROS2 Topic Spec v0.1

## 方針
- ROS2 topic は mc_proto と意味一致させる
- B/C が bag 前提で並行開発できるように固定する

## Topics

### /scan
- 型: `sensor_msgs/LaserScan`
- frame_id: `laser`
- angle: -pi..pi（推奨）
- range_min/max: センサ仕様に合わせる

### /mc/drive_cmd
- 型: `mc_msgs/DriveCmd`（仮）
- 意味: `mc::proto::DrivePayload` と同等
- フィールド:
  - `steer_cdeg` (int16, 0.01deg)
  - `speed_mm_s` (int16, mm/s)
  - `ttl_ms` (uint16, ms)
  - `dist_mm` (uint16, mm)

### /mc/status
- 型: `mc_msgs/Status`（仮）
- 意味: `mc::proto::StatusPayload` と同等
- フィールド:
  - `seq_applied` (uint8)
  - `auto_active` (uint8)
  - `faults` (uint16)
  - `speed_mm_s` (int16)
  - `steer_cdeg` (int16)
  - `age_ms` (uint16)

### /mc/hils_state
- 型: `mc_msgs/HilsState`（仮）
- 意味: `mc::proto::HilsStatePayload` と同等
- フィールド:
  - `timestamp` (uint32)
  - `throttle_raw` (int16)
  - `steer_cdeg` (int16)
  - `flags` (uint8)

### /mc/log
- 型: `mc_msgs/LogRecord`（仮）
- 意味: ログ統一仕様
- フィールド:
  - `ts_ms` (uint32)
  - `level` (uint8)
  - `text` (string)
  - `run_id` (string)

