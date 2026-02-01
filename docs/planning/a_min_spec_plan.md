# A Minimal Spec Plan (v0.1) — ROS 2 Humble / RViz / bag / Contracts

このドキュメントは、A（基盤）担当が **B/C をブロックせずに並行開発可能にするため**の「最小仕様（Minimal Spec）」を **10日計画のDay1〜Day3相当**で完了させる実装計画書である。

- ROS 2 distro: **Humble**（Dockerは `ros:humble-desktop`）
- 座標系（最短）：**`base_link` / `laser` を凍結**（`map/odom` は次フェーズ）
- 可視化：**RViz（GUI）必須**
- ホスト：**Mac上Dockerが主、RPi上Dockerも同等に動作**

---

## 0. ゴール（Definition of Done）

Aの最小仕様 v0.1 が “Done” である条件：

1. `make ros2-up` で ROS2 環境が起動し、**MacでもRPiでもRVizが表示できる**
2. `docs/contracts/ros_topics_v0.1.md` が存在し、**topic/型/frame_id が凍結**されている
3. `rpi/ros2_ws/src/mc_msgs` が存在し、**B/C が依存できる msg 定義が凍結**されている
4. bridge 未実装でも `mc_demo_pub` で `/scan` が出て、**RVizでLaserScanが見える**
5. `bag_record.sh` / `bag_play.sh` があり、**run_id規約 + metadata.json 付きで記録/再生できる**
6. （推奨）`seriald.telemetry.sock` の契約（＋可能なら実装）により、**観測クライアントが安全に接続できる**

---

## 1. Day1で凍結する最小契約（Contracts v0.1）

### 1.1 ROS2 Topics（凍結対象）
- `/scan` : `sensor_msgs/msg/LaserScan`
  - `header.frame_id = "laser"`（固定）
- `/tf_static` : `tf2_msgs/msg/TFMessage`
  - `base_link -> laser`（static transformを publish）
- `/mc/status` : `mc_msgs/msg/Status`
- `/mc/drive_cmd` : `mc_msgs/msg/DriveCmd`
- `/mc/log` : `mc_msgs/msg/McLog`
- （任意）`/mc/bridge_diag` : `diagnostic_msgs/msg/DiagnosticArray`

### 1.2 Frame（凍結対象）
- `base_link`（固定）
- `laser`（固定）
- ※`map/odom` は次フェーズで追加・凍結する

### 1.3 bag運用（凍結対象）
- 保存先：`training/data/bags/<run_id>/`
- `metadata.json` を同階層に必ず生成
- `run_id` 生成規則はスクリプトで統一（例：`YYYYMMDD_HHMMSS_<short>` or UUID）

---

## 2. 実装の全体像（成果物ツリー）

```

docs/
contracts/
ros_topics_v0.1.md
bag_ops_v0.1.md
mc_bridge_v0.1.md            # 定義のみ（実装は次フェーズでOK）
ipc_uds_v0.1.md              # (推奨) telemetry socket 契約
docker/
ros2/
Dockerfile
docker-compose.yml
tools/
ros2/
rviz/default.rviz
scripts/
run_id.sh
bag_record.sh
bag_play.sh
rpi/ros2_ws/
src/
mc_msgs/
msg/DriveCmd.msg
msg/Status.msg
msg/McLog.msg
mc_tf_static/                # base_link->laser の static TF
mc_demo_pub/                 # bridge無しで /scan を出す

```

---

## 3. PR計画（A1〜A5）

### PR-A1: ROS2基盤（Humble desktop）+ RViz GUI（Mac/RPi同等）
**目的**：`make ros2-up` で “誰でも” GUI まで到達する。

**作業**
- `docker/ros2/Dockerfile` を `ros:humble-desktop` ベースで作成
- `docker/ros2/docker-compose.yml` を追加
- RViz表示方式を確定（Mac/RPi同等が最優先）
  - 推奨：**noVNC**（ブラウザ表示。XQuartz不要で事故りにくい）
- Makeターゲット追加
  - `ros2-up`, `ros2-down`, `ros2-shell`, `ros2-build`, `ros2-rviz`

**受け入れ基準**
- Mac: `make ros2-up` → ブラウザでRViz画面が見える
- RPi: `make ros2-up` → 同様にRViz画面が見える
- `make ros2-shell` で `ros2 topic list` が実行できる

**チェック**
- [ ] Macで起動確認
- [ ] RPiで起動確認
- [ ] README/手順が `docs/` にある

---

### PR-A2: Contracts（ROS topics）+ mc_msgs + static TF
**目的**：B/Cが依存する “型” と “topic契約” をDay1で凍結する。

**作業**
- `docs/contracts/ros_topics_v0.1.md` を追加（ここで凍結宣言）
- `rpi/ros2_ws/src/mc_msgs/` を追加（msg定義）
  - `DriveCmd.msg`, `Status.msg`, `McLog.msg`
- `rpi/ros2_ws/src/mc_tf_static/` を追加
  - `base_link -> laser` を `/tf_static` に publish
  - transform値は `rpi/config/frames.yaml`（または同等）から読む

**受け入れ基準**
- `/tf_static` が publish される
- RVizで `Fixed Frame = base_link` にできる（TFが生きている）
- `mc_msgs` が `colcon build` で通る

**チェック**
- [ ] docs/contracts/ros_topics_v0.1.md がレビュー済み
- [ ] msg定義が最小で過不足ない
- [ ] TFが1回だけpublishされ、重複しない

---

### PR-A3: demo publisher（bridge無しで /scan をRViz表示）
**目的**：bridgeが無くてもBがFTG/SLAMのROS側実装に着手できる。

**作業**
- `rpi/ros2_ws/src/mc_demo_pub/` を追加
  - `/scan` を疑似生成して publish（簡単でOK）
  - `header.frame_id="laser"`
  - `angle_min/max`, `angle_increment`, `range_min/max` を正しく設定
- `tools/ros2/rviz/default.rviz` を追加
  - LaserScan表示を保存
- `make ros2-rviz` が `default.rviz` を開く導線を持つ

**受け入れ基準**
- bridge無しで RViz に `/scan` が表示される
- bag record/play の前提として最低限のtopicが存在する

**チェック**
- [ ] `/scan` が一定周期で出る
- [ ] `frame_id` が `laser` で固定
- [ ] RViz configが保存されている

---

### PR-A4: bag運用雛形（record/play）+ run_id + metadata
**目的**：Mac/RPiで同一のデータ収集・再生フローを固定し、Cの学習データ基盤も同時に作る。

**作業**
- `docs/contracts/bag_ops_v0.1.md` を追加（運用凍結）
- `tools/ros2/scripts/run_id.sh` を追加（run_id生成）
- `tools/ros2/scripts/bag_record.sh` を追加
  - 記録topicは固定（`/scan`, `/tf_static`, `/mc/status`, `/mc/drive_cmd`, `/mc/log`）
  - 出力先 `training/data/bags/<run_id>/`
  - `metadata.json` 生成（run_id, git_sha, host, notes）
- `tools/ros2/scripts/bag_play.sh` を追加
  - `BAG=<path>` を受け、`--clock` で再生（将来 `use_sim_time` と連動）
- Makeターゲット追加
  - `ros2-bag-record`, `ros2-bag-play`

**受け入れ基準**
- demo publisher稼働中に record → play が成立する
- `metadata.json` が出る（run_id と git_sha が入る）

**チェック**
- [ ] Macで record/play 確認
- [ ] RPiで record/play 確認
- [ ] 生成物の保存先が規約通り

---

### PR-A5（推奨 / P0）: seriald telemetry socket（観測の安全確保）
**目的**：B/C/可視化/学習収集が “安全に並行接続” できるようにする。

**作業**
- `docs/contracts/ipc_uds_v0.1.md` を追加（control/telemetryの契約）
- 可能なら `seriald` に以下を実装：
  - `/run/roboracer/seriald.sock`（control：送信権あり。原則 racerd のみ）
  - `/run/roboracer/seriald.telemetry.sock`（telemetry：観測専用。送信禁止）
  - telemetryクライアントが送信したら切断 or 無視＋WARN（安全）

**受け入れ基準**
- control運転中に telemetry が複数接続できる
- telemetry から送信しても UART に流れない

**チェック**
- [ ] telemetry接続クライアントが増えても安定
- [ ] “誤送信” を確実に防げる

---

## 4. mc_bridge の扱い（定義はDay1、実装は次フェーズ）
**方針**：今フェーズは “基盤” のみ。実データをROSへ流す bridge は次フェーズで実装する。  
ただし、B/Cの並行開発のため、**mc_bridge の契約だけはDay1で凍結**する。

### docs/contracts/mc_bridge_v0.1.md（Day1で追加）
- 入力：`seriald.telemetry.sock`（mc_protoフレーム）
- 出力：
  - `/mc/status`, `/mc/log`, `/mc/hils_state`, `/mc/ack`
- エラー：CRC/COBS不正は `/mc/bridge_diag` へ
- stamp：受信時刻を `header.stamp` とする（最初はこれで十分）
- run_id：bag/ログと同じ run_id で束ねる

---

## 5. 実行手順（スモークテスト）

### ROS2
- `make ros2-up`
- `make ros2-shell`
- `make ros2-build`
- `make ros2-rviz`

### demo publisher
- コンテナ内で `mc_demo_pub` 起動（コマンドはPR内のREADMEに固定）
- RVizで `/scan` が見えることを確認

### bag
- `make ros2-bag-record`
- `BAG=training/data/bags/<run_id> make ros2-bag-play`

---

## 6. 完了判定（Day1 Closeout Checklist）

- [ ] `docs/contracts/ros_topics_v0.1.md` がある（凍結宣言）
- [ ] `mc_msgs` があり、B/Cが依存できる
- [ ] `base_link -> laser` の `/tf_static` が出ている
- [ ] demo `/scan` がRVizで見える
- [ ] rosbag record/play が規約通りに動く（run_id + metadata.json）
- [ ] （推奨）telemetry socket の契約（＋可能なら実装）が入っている

---

## 7. リスクと回避策

- **RViz GUI（Mac/RPi同等）**
  - X11は環境差で詰まりやすい → **noVNC方式を標準化**して事故を減らす
- **topic/msgの後出し変更**
  - B/Cの手戻りが発生 → v0.1凍結後は “追加のみ” を原則
- **telemetry未整備**
  - 観測ツールがcontrolへ誤送信する事故 → `telemetry.sock` を最優先で導入

---

## 8. 次フェーズ（参考）
- `mc_bridge` 実装（telemetryからdecodeしてROS publish）
- `lidar_bridge` 実装（実LiDAR or IPC → `/scan`）
- `log_to_bag`（既存ログ → rosbag2変換）
- 座標系拡張（`odom` / `map` 凍結、REP-105への移行）
