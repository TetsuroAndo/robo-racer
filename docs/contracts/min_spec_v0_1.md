# A側 最小仕様 v0.1（B/C 並行開発ブロッカー解除用）

## 目的
本ドキュメントは **インターフェース確定 → 実装計画 → 受け入れ基準** の順でまとめる。

---

## 0. ゴール定義（最小）

### Aが満たすべき必須条件
1. **インターフェース凍結（契約）**
   - `mc_proto`（フィールド・単位・時刻・seq運用・互換性）
   - UDS の役割分離（**制御系と観測系の分離**）
   - ログの最低限構造（後で rosbag / 学習データ化できる粒度）
2. **B/Cが“実機なし”で動かせる入力源**
   - HILS最小（Drive→Status 返却）またはログリプレイ
3. **ROS2開発基盤（Docker/Make導線/rosbag雛形/RViz）**
   - Ubuntu を主、Mac も同等に対応
   - bridge 実装前でも **ROS2側の型・運用規格が固まっている**

---

## 1. まず確定すべき「契約（Contract）」最小セット

### 1.1 `mc_proto` 契約（既存を仕様書化して凍結）
**凍結対象（v1）**
- Header（9 bytes）
  - `magic[2]='M','C'` / `ver=1` / `type` / `flags` / `seq_le` / `len_le`
- Type（最低限）
  - `DRIVE(0x01)`, `KILL(0x02)`, `MODE_SET(0x03)`, `PING(0x04)`
  - `LOG(0x10)`, `STATUS(0x11)`, `HILS_STATE(0x12)`, `ACK(0x80)`
- Payload（単位込みで固定）
  - `DrivePayload { steer_cdeg, speed_mm_s, ttl_ms_le, dist_mm_le }`
  - `StatusPayload { seq_applied, auto_active, faults, speed_mm_s, steer_cdeg, age_ms }`
  - `HilsStatePayload { timestamp, throttle_raw, steer_cdeg, flags }`
- **互換性ルール**
  - v1の拡張は **新Type追加のみ**（既存payloadを壊さない）
  - 既存payload変更が必要なら `ver` を上げる（v2）

**作業**
- `docs/contracts/mc_proto_v1.md` を作成して **コードから起こす**

---

### 1.2 UDS（IPC）契約：**制御(Write) と 観測(Read) を分離**
現状 `seriald` は「受信したら UART へすべて forward」。
観測専用クライアントが誤送信すると事故るため、**2系統に分離**する。

**最小仕様**
- **Control Socket（送信権あり）**
  - `/run/roboracer/seriald.sock`（推奨）
  - ルール：**送信者は1プロセスのみ**（= racerd）
  - 例外：`serialctl` はデバッグ時のみ
- **Telemetry Socket（観測専用）**
  - `/run/roboracer/seriald.telemetry.sock`
  - ルール：`send()` したら **即切断**（または無視＋WARN）
  - UART受信フレームを **telemetryへbroadcast**

**注意**
- 現状デフォルトは `/tmp/roboracer/seriald.sock`（`rpi/apps/seriald/config/Config.h`）。
  本仕様では `/run/roboracer` への移行を前提に、**移行計画を明記**する。

**作業**
- `docs/contracts/ipc_uds.md` を作成
- `seriald` を **control/telemetry の2ソケット**に改修

---

### 1.3 ログ契約（後で使える最小構造）
**最小ログ要件（推奨）**
- JSONL 形式
- 1行に最低限：
  - `ts_us`（epoch か monotonic を固定）
  - `proc`（seriald / racerd / lidard …）
  - `level`
  - `event`（例: `rx_frame`, `tx_frame`, `plan`, `cmd`, `fault`）
  - `mc`（該当するなら: `type, seq, payload_summary`）
- **run_id**（走行セッションID）で横断可能にする

**作業**
- `docs/contracts/log_schema_v1.md` を作成
- run_id を必須にする

---

### 1.4 mc_bridge 契約（定義のみ）
**方針**
- **実装は次フェーズ**だが、B/C が並行開発できるよう **定義は今フェーズで凍結**する

**最小仕様**
- 入力: `seriald.telemetry.sock` の mc_proto フレーム
  - UART受信に加えて **RPi→ESP32 送信フレームも telemetry にミラー**（/mc/drive_cmd 可視化のため）
- 出力: ROS2 topic
  - `/mc/status` `/mc/hils_state` `/mc/log` `/mc/drive_cmd`
- 例外: decode/CRC/seq異常は `/mc/log` に WARN 出力

**作業**
- `docs/contracts/mc_bridge_v0.1.md` を作成

---

## 2. 実装計画（タスク分解）

### 2.1 P0: Telemetry Socket 追加（seriald改修）
**目的**：観測系（ROS/SLAM/学習）を安全に並行接続できる

**タスク**
1. `seriald` に `telemetry_ipc` を追加
2. listen path：
   - control: `/run/roboracer/seriald.sock`
   - telemetry: `/run/roboracer/seriald.telemetry.sock`
3. UART→UDS broadcast を
   - control clients にも流す（既存維持）
   - telemetry clients にも流す（新規）
4. **RPi→ESP32 送信フレームも telemetry にミラー**（DRIVE/MODE/KILL 可視化）
5. telemetry 側の `recv()` は **即切断**（送信禁止を強制）
6. docs に接続方針を明記

**受け入れ基準**
- racerd が control で運転中でも、観測系が telemetry で並行接続できる
- telemetry クライアントが誤送信しても車体へ流れない
- `/mc/drive_cmd` に **送信命令が可視化**される（telemetry ミラー前提）

---

### 2.2 P0: run_id 導入
**目的**：B/C が bag・学習データ・SLAMログを同一単位で扱える

**タスク**
1. 起動時に run_id を生成
2. 全ログに run_id を付与
3. 出力ディレクトリ規約を固定
   - `./logs/<run_id>/...`
   - `./bags/<run_id>/...`

**受け入れ基準**
- どのログ/成果物も run_id で束ねられる
- 再起動でも引き継げる（環境変数 or run_id.txt）

---

### 2.3 P0: ROS2開発基盤（Docker/Make導線/rosbag雛形/RViz）
**目的**：bridge 前でも ROS2 側の型と運用を固定する

**タスク**
1. `tools/ros2/` を追加（Ubuntu/Mac 両対応）
2. compose で RViz が動く形を標準化
3. Make導線（ros2-up/shell/build/bag-record/bag-play）
4. rosbag スクリプト（record/play）
5. RViz config 雛形（必要なら）

**受け入れ基準**
- Ubuntu/Mac で RViz が起動できる
- bag record/play の型が固定される

---

### 2.4 P1: ROS2メッセージ定義（mc_msgs）
**目的**：bridge 未実装でも msg/トピック仕様を固定

**タスク**
- `ros2_ws/src/mc_msgs/` に msg 定義を追加

**受け入れ基準**
- `/mc/status` `/mc/drive_cmd` 等が型付きで扱える

---

### 2.5 P1: HILS 最小（Drive→Status）
**目的**：実機なしで「命令→反映」を検証可能にする

**タスク**
- 擬似ESP32（sim）またはログリプレイで Status を生成
- `serialctl` で Drive を送ると Status が返る

**受け入れ基準**
- 実機なしで「コマンド→状態」が回る

---

## 3. 受け入れ基準（まとめ）
- **UDS分離**：control/telemetry が分離され誤送信が車体へ流れない
- **run_id**：ログ/バッグ/成果物が run_id で紐づく
- **ROS2基盤**：Ubuntu/Mac 両方で RViz が動く
- **mc_msgs**：ROS2トピックの型が固定される
- **HILS最小**：実機なしで Drive→Status が確認できる

---

## 4. 確認事項（最小）
1. ROS2 ディストロは Humble 固定でよいか
2. `ts_us` は epoch / monotonic のどちらを一次にするか（両方出す方針でも可）
