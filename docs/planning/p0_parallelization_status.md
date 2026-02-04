# P0 並走安全化・ROS 実データ化 ステータス（Day2-3）

## 目的
- Day1で凍結したI/Fを **安全に並走できる状態**へ引き上げる。
- 観測の誤送信リスクを遮断し、ROSへ実データを通す入口を作る。

## 前提
- `/run/roboracer` を正、`/tmp/roboracer` を互換パスとして維持（symlink もしくは環境変数で切替）。
- `docs/contracts/ipc_uds_v0.1.md` / `mc_bridge_v0.1.md` / `ros_topics_v0.1.md` の凍結内容に準拠する。

## ブランチ方針（分割）
- P0-1: `feat/seriald-telemetry-split`
- P0-2: `feat/mc-bridge-telemetry`
- P0-3: `feat/lidar-bridge-scan`
- 統合: `integration/p0-parallelization`（必要なら）

## 依存関係
- P0-1 → P0-2（telemetry.sock が前提）
- P0-3 は P0-1 非依存（ROS / SHM 側のみ）

## ステータス
| 項目 | 状態 | 期限 | 備考 |
| --- | --- | --- | --- |
| P0-1 seriald control/telemetry 分離 | 実装済み（未検証） | 2026-02-03 | 互換パス維持・送信遮断・ミラー配信 |
| P0-2 mc_bridge 実働化 | 実装済み（未検証） | 2026-02-04 | telemetry decode → /mc/* publish |
| P0-3 lidar_bridge 追加 | 実装済み（未検証） | 2026-02-04 | SHM → /scan publish |

## P0-1 詳細（seriald 分離）
- 対象: `rpi/apps/seriald/src/main.cpp`, `rpi/apps/seriald/config/Config.h` ほか
- 受け入れ基準（DoD）
  - control 運転中でも telemetry に複数クライアント接続できる
  - telemetry からの送信が UART に **一切流れない**
  - 既存 `hils-local` / 机上E2E が壊れない

## P0-2 詳細（mc_bridge 実働）
- 対象: `rpi/ros2_ws/src/mc_bridge`
- 受け入れ基準（DoD）
  - telemetry.sock から STATUS/DRIVE/HILS を decode して ROS publish
  - `/mc/run_id` が無い場合 `/mc/log` を publish しない
  - decode/CRC 失敗カウント or ログを出す

## P0-3 詳細（lidar_bridge）
- 対象: `rpi/ros2_ws/src/lidar_bridge`（新規）
- 受け入れ基準（DoD）
  - `tools/ros2/rviz/default.rviz` で `/scan` 表示できる
  - `base_link -> laser` の `/tf_static` と組み合わせて破綻しない

## 進行メモ
- 2026-02-01: ステータス管理を開始。P0-1〜P0-3 をブランチ分割で実装する方針を確定。
- 2026-02-01: seriald を control/telemetry に分離（送信遮断・ミラー配信・/tmp 互換listen）。
- 2026-02-01: mc_bridge に telemetry decode を追加（/mc/status /mc/drive_cmd /mc/hils_state /mc/log）。
- 2026-02-01: lidar_bridge を追加（/dev/shm/lidar_scan → /scan）。
