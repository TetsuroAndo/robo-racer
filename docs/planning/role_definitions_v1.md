# タスクA/B/C 定義書 v1

## 目的
A/B/C 表記の意味を固定し、タスク範囲の解釈違いを防ぐ。

## 共通方針
- A/B/C は**責務のグルーピング**であり、個人名ではない
- Interface Freeze（v1）を前提に並行開発する
- 変更は v2 として追記する

---

# A: インターフェース/テスト/HILS/ロギング

## タスク領域
- RPi↔ESP32 プロトコルの凍結と整合
- RPi内IPC/ログの契約化と共通化
- sim_esp32d を含む机上HILSと統合テスト

## 主要成果物（ドキュメント）
- `docs/interfaces/protocol_rpi_esp32_v1.md`
- `docs/interfaces/ipc_topics_v1.md`
- `docs/interfaces/ipc_payloads_v1.md`
- `docs/observability/logging_v1.md`
- `docs/testing/hils_v1.md`

## 主要成果物（実装）
- `shared/proto` の v1 型定義固定
- sim_esp32d（HILS代替）
- seriald/racerd 机上E2E

## 非対象（v1）
- ルールベース走行ロジックの詳細
- 学習データ/推論のパイプライン

---

# B: ルールベース（FTG）/ SLAM / セーフティ

## タスク領域
- LiDAR入力からの FTG 走行
- RPi内IPCでの LIDAR_SCAN/SUMMARY 連携
- 安全停止条件の強化（single-point/IMU 等）
- ROS2/SLAM の再現可能な運用手順

## 主要成果物（ドキュメント）
- `docs/planning/ftg_v1.md`
- `docs/slam/ros2_setup_v1.md`

## 主要成果物（実装）
- `lidard` のIPC publish
- `racerd` の FTG 統合（DRIVE_CMD生成）

## 非対象（v1）
- AI推論・学習パイプライン
- プロトコル/ログ契約の変更

---

# C: AI/データ/推論入口

## タスク領域
- 走行ログから dataset 生成
- imitation 学習パイプライン
- racerd への policy plugin（FTG/AI/Shadow切替）

## 主要成果物（ドキュメント）
- `docs/ml/dataset_schema_v1.md`
- `docs/ml/policy_plugin_v1.md`

## 主要成果物（実装）
- dataset 生成ツール
- 学習/評価の入口（make train/eval 等）

## 非対象（v1）
- ESP32ファームの安全制御
- ルールベース走行の詳細
