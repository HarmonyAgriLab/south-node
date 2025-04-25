#ifndef SOFT_UART_H
#define SOFT_UART_H

#include "pinctrl.h"
#include <stdint.h>

#define SOFT_UART_TRANSFER_SIZE 19          // 软串口传输大小
#define CONFIG_SOFT_UART_TXD_PIN 11         // 软串口TXD引脚
#define CONFIG_SOFT_UART_RXD_PIN 12         // 软串口RXD引脚
#define UART_DELAY_us 208                   // UART延迟时间
#define TIMER1_DELAY_US UART_DELAY_us - 4   // 定时器延迟时间

#define state_start 1   // 接收状态标志
#define state_stop 0    // 接收状态标志
#define TIMER_INDEX 1   // 定时器索引
#define TIMER_PRIO 1    // 定时器优先级

int soft_uart_write(char* str, int len);
int soft_uart_read(char* str, int len);
void soft_uart_init(void);

#endif