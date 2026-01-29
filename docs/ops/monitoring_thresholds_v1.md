# 性能監視の閾値 v1

## 目的
- 温度/CPU使用率の異常をログで検知できるようにする
- 運用時の暫定アラート基準を明文化する

## デフォルト閾値（metricsd）
- CPU温度
  - WARN: 70.00°C（`temp_warn_cdeg=7000`）
  - CRIT: 80.00°C（`temp_crit_cdeg=8000`）
- CPU使用率（permille）
  - WARN: 80%（`cpu_warn_permille=800`）
  - CRIT: 95%（`cpu_crit_permille=950`）

## 変更方法
`metricsd` 起動時にオプションを指定する。

```
metricsd \
  --interval-ms 1000 \
  --temp-warn-cdeg 6500 \
  --temp-crit-cdeg 7500 \
  --cpu-warn-permille 700 \
  --cpu-crit-permille 900
```

## 運用メモ
- WARNは減速や検証を促すための目安
- CRITは即時停止/再起動を検討する目安
- 実機での温度分布を観測した後、必ず再調整する

