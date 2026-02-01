# dir_utils 共有化ステータス

## 目的
- sim_esp32d / serialctl / metricsd / seriald に重複している `ensure_dir_` と `dir_of_` を共通ユーティリティへ移動する。
- DRY 原則に沿って保守性を上げる。

## 状態
- 完了: 共通ユーティリティを `rpi/lib/mc_core` に追加
- 完了: 各アプリの参照先を `mc::core::ensure_dir` / `mc::core::dir_of` に切替
- 未着手: 影響範囲のビルド確認
- 進行中: レビュー後の最終更新

## 変更方針メモ
- 共有先は `rpi/lib/mc_core` 配下にヘッダを追加する。
- API は既存利用と同等の振る舞いを維持する。
