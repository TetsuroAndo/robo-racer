# LiDAR 共有メモリ連携 状態管理

## 目的
- rpi/src のメイン処理が LiDAR デバイスを直接制御せず、`lidar_received` の共有メモリを参照して処理する。

## 状態一覧
- [完了] 共有メモリ読取クラスの追加（open/close、sem保護、コピー）
- [完了] rpi/src/main.cpp を共有メモリ読取に切り替え
- [完了] `lidar_received` 側の seq 運用見直し（インクリメント化）
- [完了] エラー時のリトライ/起動順の運用メモ

## メモ
- `lidar_received` の共有メモリ名: `/lidar_scan`、セマフォ名: `/lidar_scan_sem`
- `seq` はフレーム更新ごとにインクリメント（`UINT32_MAX` 到達時は 1 に戻す）
- 起動順: `lidar_received` → `robo-racer`
- `robo-racer` は共有メモリ未作成時に待機して再試行（200ms間隔）
- 共有メモリ/セマフォが残留する場合は `lidar_received` を停止後に `/dev/shm/lidar_scan` と `/dev/shm/sem.lidar_scan_sem` を削除して復旧する
