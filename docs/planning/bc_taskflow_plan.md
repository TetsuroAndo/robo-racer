# タスクB（ルールベース/SLAM/安全）・タスクC（AI/データ/推論入口） タスクフロー & 実装計画（10日版）

## 目的
タスクB（ルールベース/SLAM/センサー安全）とタスクC（AI/データ/推論入口）の
**タスクフロー**と**実装計画**を明文化し、
Interface Freeze 後に**並行開発**できる状態を作る。

## 前提となる契約（Interface Freeze）
以下を**契約（v1）として凍結**し、タスクB/タスクCはこの前提で実装する。

- RPi↔ESP32 プロトコル: `docs/proto/overview.md`
- RPi内IPCトピック: `docs/interfaces/ipc_topics_v1.md`
- ログ方針/ログ運用: `docs/proto/protocol_logging_plan.md`
- 固定責務の前提: `docs/architecture/overview.md`

重要:
- **DRIVE は TTL 前提の安全設計**。タスクB/タスクCは送信周期・TTL を厳守する。
- **AUTO中は MANUAL を無視**、介入は KILL のみ。

---

# タスクB: ルールベース（FTG）/ SLAM / セーフティ

## 役割
- LiDAR入力からの**FTG走行**を成立させる
- 走行の**安全停止条件**を強化する
- ROS2 + SLAM で**再現可能な地図生成手順**を整備する

## 関連ドキュメント
- `docs/planning/ftg_v1.md`
- `docs/slam/ros2_setup_v1.md`

## タスクフロー（論理フロー）
```
LiDAR raw -> (lidard) -> LIDAR_SCAN
                     -> LIDAR_SUMMARY

LIDAR_SCAN/LIDAR_SUMMARY -> (racerd) -> FTG -> DRIVE_CMD
DRIVE_CMD -> (seriald) -> DRIVE (ttl_ms付き) -> ESP32

ESP32 STATUS -> RPi -> VEHICLE_STATUS (IPC)
```

### 主要データ（v1想定 / IPC凍結）
- LIDAR_SCAN: `IPC_LIDAR_SCAN` / 5-20Hz / LidarScanChunkPayload + data
- LIDAR_SUMMARY: `IPC_LIDAR_SUMMARY` / 10-50Hz / LidarSummaryPayload
- IMU_SAMPLE: `IPC_IMU_SAMPLE` / 50-200Hz / ImuSamplePayload
- DRIVE_CMD: `IPC_DRIVE_CMD` / **100Hz（10ms）** / DriveCmdPayload（steer_cdeg/speed_mm_s/ttl_ms）
- VEHICLE_STATUS: `IPC_VEHICLE_STATUS` / **100Hz（10ms）** / VehicleStatusPayload

## 実装計画（10日）

### Day1-2: FTGコアの土台
- FTGアルゴリズム v0 実装
  - 入力: LIDAR_SUMMARY or LIDAR_SCAN（仮データOK）
  - 出力: steer_cdeg / speed_mm_s
- FTGパラメータ表（閾値/窓幅/速度係数）作成

### Day3-4: IPC連携 + ルールベース統合
- `lidard` の IPC publish（SCAN/SUMMARY）
- `racerd` v0: subscribe → FTG → DRIVE_CMD 生成
- TTL/送信周期の固定（例: **10ms周期、ttl=20-30ms**）

### Day5-6: 安全系の強化
- single-point LiDAR の停止条件（ESP32側が最優先）
- RPiに距離・状態を載せてログ可視化
- IMU 取得のログ化（ブラインド区間の速度制御に備える）

### Day7-8: ROS2/SLAM の再現手順
- docker + slam_toolbox の手順書（オフラインでも再現）
- 地図生成 → 中心線 → 簡易パスの手順確立

### Day9-10: 安定化 & 最終固定
- FTGのパラメータ固定
- E2Eで再現可能な走行手順を記録

## DoD（受け入れ条件）
- `lidard` が v1 IPC を publish できる
- `racerd` が FTG で DRIVE を安定生成できる（TTL遵守）
- replay入力で**決定論的**に同じ出力
- ROS2/SLAM が別PCでも再現可能

## テスト/検証
- FTG単体: 入力→出力が固定となるユニットテスト
- 統合: sim_esp32d + seriald + racerd で机上E2E
- replay: 既存ログで同じ結果が出るか比較

---

# タスクC: AI/データ/推論入口

## 役割
- 走行ログからの**dataset生成**
- imitation学習の**最小ループ**
- racerd への**policy plugin**（FTG/AI切替）

## 関連ドキュメント
- `docs/ml/dataset_schema_v1.md`
- `docs/ml/policy_plugin_v1.md`

## タスクフロー（論理フロー）
```
LOG_RECORD/IPC topics
   -> (dataset builder) -> training/data
   -> (train/eval) -> models/production
   -> (policy plugin) -> racerd
```

### 主要データ（v1想定 / IPC凍結）
- LiDAR: `IPC_LIDAR_SCAN`（5-20Hz）or `IPC_LIDAR_SUMMARY`（10-50Hz）
- コマンド: DRIVE / MODE_SET / KILL（RPi↔ESP32プロトコル）
- 状態: `IPC_VEHICLE_STATUS`（10-50Hz）/ `IPC_IMU_SAMPLE`（50-200Hz）

## 実装計画（10日）

### Day1-2: データ設計
- dataset schema（timestamp整列/欠損処理/正規化）
- ログ→dataset変換ツール v0（CSV/Parquet どちらか）

### Day3-4: 学習パイプライン v0
- imitation 学習（入力: summary、出力: steer/speed）
- 評価指標（追従誤差/安定性）定義

### Day5-6: 推論差し替え口
- racerd に policy plugin 仕様を追加
- FTG/AI/Shadow の切替
- AI 失敗時は FTG へフェイルセーフ

### Day7-8: 自動化
- `make train` / `make eval` 等の入口
- モデル/データの履歴管理（最低限のメタ情報）

### Day9-10: 実ログで1周
- 実ログから dataset → 学習 → 推論評価
- “どのデータが足りないか” を明文化

## DoD（受け入れ条件）
- 1コマンドで dataset 生成できる
- 小モデルで学習→推論が再現できる
- racerd で FTG/AI 切替が安全に動作

## テスト/検証
- dataset 生成の再現性テスト
- 学習/評価のスモークテスト
- Shadow mode でのログ比較（AI vs FTG）

---

# 依存関係・リスク
- Interface Freeze（proto/log/IPC）が遅れるとタスクB/タスクCがブロック
- TTL/周期が守れないと安全設計が崩れる
- dataset schema が曖昧だと学習が再現不能

# 次のアクション（タスクB/タスクC同時並行）
- B: FTG v0 と lidard IPC を先行着手
- C: dataset schema と変換ツール v0 を先行着手
