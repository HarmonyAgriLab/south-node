#include "app_init.h"
#include "chip_core_irq.h"
#include "common_def.h"
#include "gpio.h"
#include "hal_gpio.h"
#include "pinctrl.h"
#include "soc_osal.h"
#include "soft_uart.h"
#include "tcxo.h"
#include "timer.h"

static uint8_t cache_len = 0;                    // 缓存中存储的接收字节数
static uint8_t rx_state = state_stop;            // 当前接收状态
timer_handle_t timer_index = { 0 };              // 定时器句柄
char rx_buff[SOFT_UART_TRANSFER_SIZE] = { 0 };   // 接收缓冲区
// static char print_buff[256] = { 0 };             // 测试打印用字符串

static void gpio_callback_func(pin_t pin, uintptr_t param);   // 声明外部中断回调函数
static void timer_timeout_callback(uintptr_t data);           // 声明定时器超时回调函数

void app_soft_uart_init_pin(void)   // 初始化软串口引脚（TXD和RXD）
{
    // 配置TXD引脚为GPIO功能，并设置为输出
    uapi_pin_set_mode(CONFIG_SOFT_UART_TXD_PIN, HAL_PIO_FUNC_GPIO);
    uapi_gpio_set_dir(CONFIG_SOFT_UART_TXD_PIN, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(CONFIG_SOFT_UART_TXD_PIN, GPIO_LEVEL_HIGH);   // 初始化TXD为高电平

    // 配置RXD引脚为GPIO功能，并设置为输入
    uapi_pin_set_mode(CONFIG_SOFT_UART_RXD_PIN, HAL_PIO_FUNC_GPIO);
    uapi_gpio_set_dir(CONFIG_SOFT_UART_RXD_PIN, GPIO_DIRECTION_INPUT);
    uapi_pin_set_pull(CONFIG_SOFT_UART_RXD_PIN, PIN_PULL_TYPE_UP);   // 启用上拉电阻

    // 注册外部中断，监听RXD引脚的下降沿，触发gpio_callback_func
    uapi_gpio_register_isr_func(CONFIG_SOFT_UART_RXD_PIN, GPIO_INTERRUPT_FALLING_EDGE, gpio_callback_func);
    uapi_gpio_enable_interrupt(CONFIG_SOFT_UART_RXD_PIN);   // 启用RXD引脚
}   // app_soft_uart_init_pin

void send_byte(uint8_t data)   // 发送一个字节的数据
{
    uapi_gpio_set_val(CONFIG_SOFT_UART_TXD_PIN, GPIO_LEVEL_LOW);   // 将TXD设置为低电平，表示起始位
    uapi_tcxo_delay_us(UART_DELAY_us);                             // 延迟，保证波特率

    uint8_t count = 0;
    while (count < 8)
    {                                                                       // 发送8位数据
        if (data & 0x01)                                                    // 判断当前最低位是1还是0
            uapi_gpio_set_val(CONFIG_SOFT_UART_TXD_PIN, GPIO_LEVEL_HIGH);   // 发送1
        else
            uapi_gpio_set_val(CONFIG_SOFT_UART_TXD_PIN, GPIO_LEVEL_LOW);   // 发送0

        uapi_tcxo_delay_us(UART_DELAY_us);   // 每位之间的延迟
        data >>= 1;                          // 右移数据，准备发送下一位
        count++;
    }

    uapi_gpio_set_val(CONFIG_SOFT_UART_TXD_PIN, GPIO_LEVEL_HIGH);   // 发送停止位（高电平）
    uapi_tcxo_delay_us(UART_DELAY_us);                              // 延迟停止位的时间
}   // send_byte

int soft_uart_write(char* str, int len)   // 发送字符串（逐字节调用send_byte）
{
    int cnt = 0;
    while (cnt < len)
    {
        send_byte(*str);   // 逐字符发送
        str++;
        cnt++;
    }

    return cnt;
}   // soft_uart_write

int soft_uart_read(char* str, int len)   // 接收字符串
{
    if ((!rx_state) && (cache_len > 0))   // 如果接收完成且缓存中有数据
    {
        int is_full = 0;
        // sprintf(print_buff, "isfull = %d, cache_len = %d, len = %d\n", is_full, cache_len, len);
        // printf("%s", print_buff);
        uapi_gpio_disable_interrupt(CONFIG_SOFT_UART_RXD_PIN);   // 关闭接收中断，防止干扰
        for (int i = 0; i < cache_len; i++)
        {
            str[i] = rx_buff[i];
            // sprintf(print_buff, "isfull = %d, cache_len = %d, len = %d, i = %d\n", is_full, cache_len, len, i);
            // printf("%s", print_buff);
            if (i + 1 == len)
            {
                is_full = 1;
                break;
            }
        }
        // sprintf(print_buff, "isfull = %d, cache_len = %d, len = %d\n", is_full, cache_len, len);
        // printf("%s", print_buff);
        memset(rx_buff, 0, SOFT_UART_TRANSFER_SIZE);   // 清空接收缓冲区
        if (!is_full)
            len = cache_len;   // 保存接收的字节数
        cache_len = 0;         // 重置接收字节计数
        // sprintf(print_buff, "isfull = %d, cache_len = %d, len = %d\n", is_full, cache_len, len);
        // printf("%s", print_buff);
        uapi_gpio_enable_interrupt(CONFIG_SOFT_UART_RXD_PIN);   // 重新启用RX中断

        return len;   // 返回接收到的字符数
    }
    return 0;
}

void soft_uart_init(void)
{
    uapi_tcxo_init();                                            // 初始化温控晶振（TCXO）
    uapi_timer_init();                                           // 初始化定时器
    uapi_timer_adapter(TIMER_INDEX, TIMER_1_IRQN, TIMER_PRIO);   // 设置定时器中断优先级
    uapi_timer_create(TIMER_INDEX, &timer_index);                // 创建定时器
    app_soft_uart_init_pin();                                    // 初始化引脚
}

void send_buff(void)   // 发送缓冲区中的数据，发送完毕后关闭RX外部中断
{
    uapi_gpio_disable_interrupt(CONFIG_SOFT_UART_RXD_PIN);   // 关闭接收中断，防止干扰
    soft_uart_write((char*)rx_buff, sizeof(rx_buff));        // 发送接收到的数据
    memset(rx_buff, 0, SOFT_UART_TRANSFER_SIZE);             // 清空接收缓冲区
    cache_len = 0;                                           // 重置接收字节计数
    uapi_gpio_enable_interrupt(CONFIG_SOFT_UART_RXD_PIN);    // 重新启用RX中断
}   // send_buff

static void gpio_callback_func(pin_t pin, uintptr_t param)   // RXD引脚的下降沿触发外部中断，开始定时器采样RX电平
{
    UNUSED(pin);   // 忽略未使用的参数
    UNUSED(param);

    rx_state = state_start;                                                      // 设置为开始状态
    uapi_timer_start(timer_index, TIMER1_DELAY_US, timer_timeout_callback, 0);   // 启动定时器
    uapi_gpio_disable_interrupt(CONFIG_SOFT_UART_RXD_PIN);                       // 禁用接收中断，防止接收过程中再次触发中断
}   // gpio_callback_func

static void timer_timeout_callback(uintptr_t data)   // 定时器回调函数，用于逐位读取RXD引脚的数据
{
    UNUSED(data);

    static uint8_t rx_byte = 0;                                   // 当前接收到的字节
    static uint8_t cnt = 0;                                       // 当前接收到的数据位数
    uint8_t temp = uapi_gpio_get_val(CONFIG_SOFT_UART_RXD_PIN);   // 读取RXD引脚的电平

    if (rx_state == state_start && cnt < 8)
    {   // 如果处于接收状态，且尚未接收完8位
        if (temp)
            rx_byte |= (0x01 << cnt);   // 如果RXD电平为高，将对应位设置为1
        else
            rx_byte &= ~(0x01 << cnt);   // 如果RXD电平为低，将对应位设置为0

        cnt++;                                                                       // 增加已接收的位数
        uapi_timer_start(timer_index, TIMER1_DELAY_US, timer_timeout_callback, 0);   // 继续启动定时器
    }
    else if (cnt == 8 && temp)
    {                                     // 当接收到8位数据且停止位为高电平
        rx_state = state_stop;            // 设置为停止状态
        rx_buff[cache_len++] = rx_byte;   // 将接收到的字节存入接收缓冲区
        if (cache_len > SOFT_UART_TRANSFER_SIZE)
        {   // 如果接收缓冲区已满，重置缓存
            cache_len = 0;
        }
        rx_byte = 0;                                            // 清空当前接收到的字节
        cnt = 0;                                                // 重置位计数
        uapi_gpio_enable_interrupt(CONFIG_SOFT_UART_RXD_PIN);   // 重新启用接收中断
    }
    else
    {                                                           // 如果接收出错（如接收到的停止位为低电平）
        rx_state = state_stop;                                  // 停止接收
        rx_byte = 0;                                            // 清空接收到的字节
        cnt = 0;                                                // 重置位计数
        uapi_gpio_enable_interrupt(CONFIG_SOFT_UART_RXD_PIN);   // 重新启用接收中断
    }
}   // timer_timeout_callback
