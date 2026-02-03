# process 意思決定テレメトリ 状態管理

## 目的
- `Process::proc()` の意思決定過程（best角/候補/override）を観測可能にする
- ターミナルでリアルタイムに状況把握できる最小UIを提供する

## 状態一覧
- [完了] `Process` のテレメトリログ（best/min/path/cmd/override）の追加
- [完了] ASCIIコンパス + top-3 + override表示の追加
- [完了] JSONL 形式の telemetry/event 出力の追加
- [完了] run_id/tick/scan_id 付与（run_id は起動時生成）
- [完了] ログ頻度の間引き/レート制御（既定10Hz、CLIで変更可）
- [完了] TTY 固定UI（フレーム化/追加情報/終了時カーソル復帰）
- [完了] UIとログの混線回避（console sink 無効化）
- [完了] テレメトリ出力のモジュール分離（TelemetryEmitter）
- [完了] rpi/src の標準出力/標準エラーを Logger へ移行（Sender/LidarReceiver/main）
- [完了] UI/JSONL の追加項目（mode/latency/score/metrics/ESP status など）
- [完了] metricsd.log の最新値を UI に反映（JSONLは除外）
- [完了] テレメトリのレート制限（既定10Hz）と出力レベル（basic/full）
- [完了] full candidates はイベント時のみ出力
- [完了] metricsd.log 読み込みを別スレッド化
- [完了] フルダッシュボードUI（スパークライン/バー/3段階色）
- [完了] MAPを最上段に配置（リッチUI拡張）
- [完了] ミニヒートマップ/イベント表示/品質指標/安定性指標を追加
- [完了] バッジ判定・イベント条件をチューニング（閾値/品質/TTL/遅延）
- [完了] TSAN ビルドオプション追加（開発時の競合検出用）
- [完了] コンパス表示を別枠化してリッチ化（B/Sマーカー付き）
- [完了] 扇形ファン内のマーカー固定化（B/T/Aを1行で表示）
- [完了] B/T/Aマーカーの色分け（コンパス行＋角度行）
- [完了] リッチコンパス拡張（LiDAR距離の扇形可視化＋進行方向の可視化）
- [完了] リッチコンパス用の距離ビン生成（Processで角度→距離プロファイルを生成）
- [完了] UI配置の再設計（コンパス枠の高さ増加・他行の圧縮）

## メモ
- 参照資料: `docs/observability/logging_v1.md`
