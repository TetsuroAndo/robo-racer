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
| P0-1 seriald control/telemetry 分離 | 実装済み（未検証） | 2026-02-03 | `feat/seriald-telemetry-split` で実装 |
| P0-2 mc_bridge 実働化 | 実装済み（未検証） | 2026-02-04 | `feat/mc-bridge-telemetry` で実装 |
| P0-3 lidar_bridge 追加 | 実装済み（未検証） | 2026-02-04 | `feat/lidar-bridge-scan` で実装 |

## P0-3 詳細（lidar_bridge）
- 対象: `rpi/ros2_ws/src/lidar_bridge`
- 受け入れ基準（DoD）
  - `tools/ros2/rviz/default.rviz` で `/scan` 表示できる
  - `base_link -> laser` の `/tf_static` と組み合わせて破綻しない

## 進行メモ
- 2026-02-01: ステータス管理を開始。P0-1〜P0-3 をブランチ分割で実装する方針を確定。
- 2026-02-01: lidar_bridge を追加（/dev/shm/lidar_scan → /scan）。
