# IMU/TSD20 UI表示（実測値中心） ステート

## 目的
- テレメトリUIに表示される値の誤読を減らす（特に `tsd20 v/r` の意味）
- UIは可能な限り **実測値**（センサ生値・観測値）を表示し、推定値/コマンド値と明確に区別する

## 関連資料
- `docs/interfaces/protocol_rpi_esp32_v1.md`
- `docs/interfaces/telemetry_transport_v2_100hz.md`
- `shared/proto/include/mc_proto.h`

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 関連資料の確認 | 完了 | 2026-02-06 | 上記を参照 |
| UI: tsd20 の v/r 表記修正 | 完了 | 2026-02-06 | `valid/ready` に変更 |
| IMU_STATUS: IMU生値の追加 | 未着手 | 2026-02-06 | ax/ay/az/gx/gy/gz をペイロードに追加 |
| IMU_STATUS: IMU生値の追加 | 完了 | 2026-02-06 | `ImuStatusPayload` を 24B に拡張し raw(ax/ay/az/gx/gy/gz) を追加 |
| seriald: IMU生値のデコード反映 | 完了 | 2026-02-06 | 新ペイロードをデコードして MotionState に反映 |
| UI/JSON: IMU推定値と生値を併記 | 完了 | 2026-02-06 | `imu.est(...)` と `imu.raw(...)` で明確化 |
| ビルド/テスト | 部分完了 | 2026-02-06 | RPiはNinjaでビルド成功。ESP32はPlatformIOがホームキャッシュ書き込み権限によりこの環境では未確認 |
