#ifndef RPI_UART_H
# define RPI_UART_H

// dev: "/dev/serial0" など
// baud: 例 115200
// 戻り値: fd（>=0） / 失敗は -1（errnoが設定される）
int uart_open_writeonly(const char *dev, int baud);
int uart_open_readwrite(const char *dev, int baud);

#endif
