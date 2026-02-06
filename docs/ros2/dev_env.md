# ROS2 開発環境（Docker/RViz）v0.1

## 目的
- Ubuntu/Mac の両方で ROS2 + RViz を再現可能にする
- bag 運用や SLAM 検証をコンテナ内で統一する

## 前提
- ROS2: Humble
- Docker/compose を利用
- リポジトリを `/ws` にマウント
- RPi は **64bit OS (arm64)** が必須（32bit だと ROS2 Docker が動かない）
- RPi では **headless (record only)** を推奨（`ros2-record` サービス）
  - 既定は自動判定（Raspberry Pi 判定時に `ros2-record` を選択）
  - 明示指定したい場合は `ROS2_SERVICE=ros2` / `ROS2_SERVICE=ros2-record`

## 1) Docker イメージのビルド
```bash
docker compose -f tools/ros2/compose.yml build
```

## 2) シェルに入る
```bash
docker compose -f tools/ros2/compose.yml run --rm ros2 bash
```
RPi で headless を使う場合:
```bash
ROS2_SERVICE=ros2-record make ros2-shell
```

## 3) RViz 起動（GUI）
### 事前注意（DISPLAY）
`docker compose up` は **DISPLAY を自動設定しない**。ホストの環境変数に従うため、
Ubuntu は `DISPLAY=:0`、Mac は `DISPLAY=host.docker.internal:0` を **事前に設定**してから起動すること。
一方で `make ros2-up` / `make ros2-rviz` は Mac の場合に DISPLAY などを自動設定する。

### Ubuntu（X11）
```bash
xhost +local:root
export DISPLAY=:0
```
コンテナ内で：
```bash
rviz2
```

### Mac（Docker Desktop + XQuartz）
1. XQuartz を起動し「接続を許可」を有効化
2. ホスト側で以下を設定
```bash
export DISPLAY=host.docker.internal:0
xhost + 127.0.0.1
```
3. コンテナ内で `rviz2`

### Mac（noVNC 推奨）
XQuartz 不要でブラウザ表示に切り替える。
```bash
make ros2-novnc
```
ブラウザで以下を開く。
```bash
http://localhost:6080/vnc.html
```
#### LAN 公開が必要な場合
デフォルトは `127.0.0.1` バインドのため、LAN からはアクセス不可。
LAN 公開する場合は **明示的に** `NOVNC_BIND=0.0.0.0` を指定すること。

#### Compose での注意
`tools/ros2/compose.yml` の `ros2-novnc` はコンテナ内で 0.0.0.0 にバインドさせる。
ホスト側の公開範囲は `ports: 127.0.0.1:6080:6080` でローカル限定のまま。

例:
```bash
NOVNC_BIND=0.0.0.0 make ros2-novnc
```

### Mac 自動設定（Make 経由）
`make ros2-rviz` / `make ros2-up` は Mac の場合に以下の環境変数を自動設定する：
- `DISPLAY=host.docker.internal:0`
- `LIBGL_ALWAYS_SOFTWARE=1`
- `QT_XCB_GL_INTEGRATION=none`
- `XDG_RUNTIME_DIR=/tmp/runtime-root`
XQuartz の設定と `xhost` は必要。

### RViz デフォルト設定
```bash
make ros2-rviz
```
`tools/ros2/rviz/default.rviz` を使用して起動する。

## 4) rpi/ros2_ws のビルド
```bash
./tools/ros2/scripts/ros2_build.sh
```
`rpi/ros2_ws/src/mc_msgs` がメッセージ定義の雛形です。
ビルド成果物は `rpi/ros2_ws/install`、colcon のビルドログは `rpi/ros2_ws/colcon_log` に出力されます。
RPi で headless を使う場合は:
```bash
ROS2_SERVICE=ros2-record make ros2-build
```

**注意:** リポジトリ直下で `colcon build` を実行すると `./install/` や `./log/` が作られて混乱するため、
ROS2 のビルドは必ず `make ros2-build` もしくは `./tools/ros2/scripts/ros2_build.sh` を使ってください。
ランタイム成果物の `logs/`（`./logs/<run_id>/...`）とは別物です。

## 5) demo publisher（/scan）と static TF
```bash
ros2 run mc_tf_static mc_tf_static --ros-args --params-file /ws/rpi/config/frames.yaml
ros2 run mc_demo_pub mc_demo_pub
```
`/scan` は `frame_id=laser` で publish される。

## 6) bag の record/play
- record: `./tools/ros2/scripts/bag_record.sh`
- session: `./tools/ros2/scripts/session_up.sh`（run_id 伝播を固定したい場合）
- play: `./tools/ros2/scripts/bag_play.sh <bag_path> [rate]`
RPi で headless を使う場合:
```bash
ROS2_SERVICE=ros2-record make ros2-bag-record
```

## 7) RPi から bag をコピーして再生（推奨運用）
RPi 側で bag を保存し、PC 側へコピーして再生する。

例（RPi → PC）:
```bash
scp -r pi@<rpi-host>:/ws/training/data/bags/<run_id> ./training/data/bags/
```

Make 経由（推奨）:
```bash
make ros2-bag-fetch RPI_HOST=<rpi-host> RUN_ID=<run_id>
```

Make で取得→再生まで一括:
```bash
make ros2-bag-play RPI_HOST=<rpi-host> RUN_ID=<run_id>
```

コピー後の再生例:
```bash
./tools/ros2/scripts/bag_play.sh ./training/data/bags/<run_id>
```
