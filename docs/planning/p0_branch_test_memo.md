# P0 ブランチ別 最低限テスト手順メモ

## 目的
- P0-1〜P0-3 の **最低限動作確認** を短時間で回せるようにする。
- 手元・机上（HILS）で「壊れていない」ことを確認する。

## 共通前提
- ブランチを切り替えたら `uv sync` もしくは `make pysync` を実施。
- ROS2/rviz の起動は `tools/ros2` を利用（必要なら `make ros2-rviz`）。

---

## P0-1: `feat/seriald-telemetry-split`

### 目的
- control/telemetry 分離が機能し、telemetry 送信が UART に流れないこと。

### 手順（机上E2E）
1. ブランチ切替
   - `git checkout feat/seriald-telemetry-split`
2. 机上E2E 起動
   - `make hils-local`
3. control ソケットで送信が通ることを確認
   - `rpi/apps/seriald/seriald_client.py --sock /run/roboracer/seriald.sock --mode auto`
4. telemetry ソケットで送信が遮断されることを確認
   - `rpi/apps/seriald/seriald_client.py --sock /run/roboracer/seriald.telemetry.sock --mode auto`
   - 期待: WARN ログが出て切断される / UART へは流れない
5. telemetry 多重接続
   - 複数ターミナルで `seriald_client.py --sock /run/roboracer/seriald.telemetry.sock --recv`
   - 期待: STATUS が複数クライアントに届く

### 判定
- telemetry 送信が UART に **一切** 流れない
- telemetry 複数接続でも control が落ちない

---

## P0-2: `feat/mc-bridge-telemetry`

### 目的
- telemetry.sock → ROS2 topic publish が成立すること。

### 手順
1. ブランチ切替
   - `git checkout feat/mc-bridge-telemetry`
2. ROS2 build
   - `tools/ros2/scripts/ros2_build.sh`
3. run_id publish
   - `tools/ros2/scripts/publish_run_id.sh`（run_id が無いと /mc/log は出ない）
4. mc_bridge 起動
   - UDS（RPi上）: `ros2 run mc_bridge mc_bridge`
   - TCP（PC側）: `ros2 run mc_bridge mc_bridge --ros-args -p telemetry_tcp_host:=<RPi_IP> -p telemetry_tcp_port:=<PORT>`
5. topic 確認
   - `ros2 topic echo /mc/status`
   - `ros2 topic echo /mc/drive_cmd`
   - `ros2 topic echo /mc/hils_state`
   - `ros2 topic echo /mc/log`

### 判定
- telemetry の STATUS/DRIVE/HILS が /mc/* に出る
- /mc/run_id が無い場合 /mc/log が抑止される
- decode/CRC 異常で WARN が出る

---

## P0-3: `feat/lidar-bridge-scan`

### 目的
- `/dev/shm/lidar_scan` → `/scan` の LaserScan が RViz で見えること。

### 手順
1. ブランチ切替
   - `git checkout feat/lidar-bridge-scan`
2. ROS2 build
   - `tools/ros2/scripts/ros2_build.sh`
3. lidar_received 起動（SHM 作成）
   - `rpi/apps/lidar_received`（手元環境に合わせて）
4. lidar_bridge 起動
   - `ros2 run lidar_bridge lidar_bridge`
5. RViz
   - `make ros2-rviz`
   - Fixed Frame = `base_link` で `/scan` が表示されること

### 判定
- `/scan` が更新され、RViz に描画される
- `base_link -> laser` の tf_static で破綻しない

---

## 補足
- `/run/roboracer` が無い環境では `/tmp/roboracer` の互換パスを利用。
- 机上E2E で不安定なら、まず `seriald` のログを確認。
