# systemd運用・ログローテ手順 v1

## 目的
- RPi上で常駐プロセスを自動起動・再起動できるようにする
- ログローテーションを外部で管理し、ディスク肥大化を防ぐ

## 対象プロセス（例）
- `seriald`
- `metricsd`
- `racerd`（今後の本体）

## 前提
- バイナリは `rpi/build/apps/*/` に配置されていること
- ログ出力は `/var/log/robo-racer/` に集約すること

## systemd unit例

### seriald
`/etc/systemd/system/seriald.service`
```
[Unit]
Description=Robo Racer seriald
After=network.target

[Service]
Type=simple
ExecStart=/home/pi/robo-racer/rpi/build/apps/seriald/seriald \
  --dev /dev/ttyAMA0 \
  --baud 921600 \
  --sock /tmp/seriald.sock \
  --log /var/log/robo-racer/seriald.log
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

### metricsd
`/etc/systemd/system/metricsd.service`
```
[Unit]
Description=Robo Racer metricsd
After=network.target

[Service]
Type=simple
ExecStart=/home/pi/robo-racer/rpi/build/apps/metricsd/metricsd \
  --interval-ms 1000 \
  --log /var/log/robo-racer/metricsd.log \
  --sock /tmp/metrics.sock
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

### racerd（将来）
`/etc/systemd/system/racerd.service`
```
[Unit]
Description=Robo Racer racerd
After=network.target seriald.service

[Service]
Type=simple
ExecStart=/home/pi/robo-racer/rpi/build/apps/racerd/racerd \
  --sock /tmp/seriald.sock
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

## 有効化/起動
```
sudo systemctl daemon-reload
sudo systemctl enable seriald metricsd
sudo systemctl start seriald metricsd
```

## 状態確認
```
systemctl status seriald
journalctl -u seriald -f
```

## ログローテ（logrotate）
`/etc/logrotate.d/robo-racer`
```
/var/log/robo-racer/*.log {
  daily
  rotate 7
  missingok
  compress
  delaycompress
  notifempty
  copytruncate
}
```

## 注意点
- `copytruncate` はプロセスを止めずにローテするための暫定策
- 高頻度ログは `interval-ms` を調整して量を抑える
- `seriald` のデバイス名は機体構成に合わせて調整する

