#include "./aht_20/aht_20.h"       // AHT20温湿度传感器
#include "app_init.h"              // 应用程序初始化
#include "chip_core_irq.h"         // 芯片中断处理
#include "cmsis_os2.h"             // CMSIS操作系统接口
#include "common_def.h"            // 通用
#include "gpio.h"                  // GPIO操作
#include "hal_gpio.h"              // 硬件抽象层GPIO操作
#include "i2c.h"                   // I2C通信
#include "mac_addr.h"              // MAC地址操作
#include "osal_debug.h"            // 操作系统调试功能
#include "pinctrl.h"               // 引脚控制
#include "securec.h"               // 安全C库
#include "soc_osal.h"              // SoC操作系统适配层
#include "soft_uart/soft_uart.h"   // soft_uart
#include "std_def.h"               // 标准
#include "stdlib.h"                // 标准库
#include "string.h"                // 字符串操作
#include "timer.h"                 // 定时器操作
#include "uart.h"                  // UART操作
#include "watchdog.h"              // 看门狗操作
#include "wifi_hotspot.h"          // WiFi热点操作
#include <stdarg.h>                // 可变参数操作
#include <stdio.h>                 // 标准输入输出
#include <stdlib.h>                // 标准库
#include <string.h>                // 字符串操作
#include <sys/time.h>              // 时间操作
#include "hal_reboot.h"

#define ENABLE_LOG 0   // 日志宏开关
#if ENABLE_LOG         //  LOG 宏，输出文件名、行号、日志内容
    #define LOG(fmt, ...) printf("LOG [%s:%d]: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
    #define LOG(fmt, ...) \
        do                \
        {                 \
            UNUSED(fmt);  \
        } while (0)
#endif

#define TASK_PRIO 24   // 任务优先级

#define UART_TRANSFER_SIZE 18        // UART传输数据大小
#define SOFT_UART_TRANSFER_SIZE 19   // 软串口传输大小
#define CONFIG_UART2_TXD_PIN 8       // UART2的TXD引脚
#define CONFIG_UART2_RXD_PIN 7       // UART2的RXD引脚
#define CONFIG_UART2_PIN_MODE 1      // UART2的引脚模式
#define CONFIG_UART2_BUS_ID 2        // UART2的总线ID
#define CONFIG_UART_INT_WAIT_MS 5    // UART中断等待时间

#define RELAY1_GPIO 15                // 继电器1引脚
#define RELAY2_GPIO 16                // 继电器2引脚

#define NB_DETECT_CNT 60     // 周期性检测NB模块是否具有上报能力（可能出现信号不好、流量卡没流量、NB模块损坏等情况）
#define SOIL_DETECT_CNT 60   // 如果土壤传感器多次无法收到正确数据，认为Soil模块不可用
#define COLLECT_DATA_CNT 2   // 数据采集周期，以秒为单位

// static float air_temp = 0,
//              air_humi = 0;                                               // 空气温度和湿度
static float moisture_value = 0, conductivity_value = 0, ph_value = 0;   // 土壤湿度、电导率和pH值
static double temperature_value = 0;                                     // 温度值
uint16_t nitrogen, phosphorus = 0, potassium = 0;                        // 土壤中的氮、磷、钾含量

int8_t mac_address[6];   // MAC地址(STA)

uint8_t soil_tx_buff[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08 };   // 土壤发送缓冲区
uint8_t soil_rx_buff[SOFT_UART_TRANSFER_SIZE] = { 0 };                          // 土壤接收缓冲区
uint8_t nb_tx_buff[UART_TRANSFER_SIZE] = { 0 };                                 // NB发送缓冲区
uint8_t nb_rx_buff[UART_TRANSFER_SIZE] = { 0 };                                 // NB接收缓冲区

int8_t aht20_flag = 0;        // 温湿度传感器可用标识
int8_t soil_flag = 0;         // 土壤传感器可用标识
int8_t soil_detect_cnt = 0;   // 土壤传感器周期检测能力标识
int8_t nb_flag = 0;           // NB模块可用标识
int8_t nb_send_flag = 0;      // NB是否正在进行发送
int8_t nb_detect_cnt = 0;     // NB周期检测能力标识

char client_id[25] = { 0 };
char pub_payload_air[100] = { 0 };    // 空气数据报文
char pub_payload_soil[200] = { 0 };   // 土壤数据报文

int Check_Crc(uint8_t* data, uint16_t length)   //! CRC-16-MODBUS校验函数
{
    uint16_t crc = 0xFFFF;   // 初始CRC值
    uint8_t i, j;

    // 计算CRC
    for (i = 0; i < length - 2; i++)
    {                     // 最后两个字节是CRC，不参与CRC计算
        crc ^= data[i];   // 将数据与CRC值异或

        for (j = 8; j > 0; j--)
        {   // 每个字节处理8次
            if (crc & 0x0001)
            {                    // 检查最低位
                crc >>= 1;       // 右移1位
                crc ^= 0xA001;   // 与多项式0xA001异或
            }
            else
            {
                crc >>= 1;   // 右移1位
            }
        }
    }

    // 获取计算出来的CRC低字节和高字节
    uint8_t crc_low = crc & 0xFF;
    uint8_t crc_high = (crc >> 8) & 0xFF;

    // 检查CRC值是否与数据中的CRC匹配
    if (crc_low == data[length - 2] && crc_high == data[length - 1])
    {
        return 1;   // 校验成功
    }
    else
    {
        return 0;   // 校验失败
    }
}   // Check_Crc

void Uart_Init_NB(void)   //! 初始化NB模组串口
{
    uapi_pin_set_mode(CONFIG_UART2_TXD_PIN, CONFIG_UART2_PIN_MODE);
    uapi_pin_set_mode(CONFIG_UART2_RXD_PIN, CONFIG_UART2_PIN_MODE);
    uart_attr_t attr2 = { .baud_rate = 9600, .data_bits = 3, .stop_bits = 1, .parity = 0 };
    uart_pin_config_t pin_config2 = { .tx_pin = S_MGPIO0, .rx_pin = S_MGPIO1, .cts_pin = PIN_NONE, .rts_pin = PIN_NONE };
    uart_buffer_config_t g_app_uart_buffer_config2 = { .rx_buffer = nb_rx_buff, .rx_buffer_size = UART_TRANSFER_SIZE };
    uapi_uart_deinit(CONFIG_UART2_BUS_ID);
    int res = uapi_uart_init(CONFIG_UART2_BUS_ID, &pin_config2, &attr2, NULL, &g_app_uart_buffer_config2);
    if (res != 0)
    {
        printf("[%s] Uart_NB Init Failed.\n", __func__);
    }

    return;
}   // Uart_Init_NB

void Uart_Init_Soil(void)   //! 初始化土壤传感器软串口
{
    soft_uart_init();   // 暂未做错误处理，一般也不能有问题
    soil_flag = 1;

    return;
}   // Uart_Init_Soil

void Soil_StartMeasure(void)   //! 测量并解析土壤数据
{
    int cnt = soft_uart_write((char*)soil_tx_buff, sizeof(soil_tx_buff));   // 发送请求数据的命令
    if (cnt == 8)
    {
        cnt = soft_uart_read((char*)soil_rx_buff, sizeof(soil_rx_buff));
        if (cnt == 19)   // 如果接收完成
        {
            if (Check_Crc(soil_rx_buff, sizeof(soil_rx_buff)))   // 检查CRC校验
            {
                soil_flag = 1;
                // 解析接收到的土壤传感器数据
                uint16_t moisture = (soil_rx_buff[3] << 8) | soil_rx_buff[4];       // 湿度值
                uint16_t temperature = (soil_rx_buff[5] << 8) | soil_rx_buff[6];    // 温度值
                uint16_t conductivity = (soil_rx_buff[7] << 8) | soil_rx_buff[8];   // 电导率值
                uint16_t pH = (soil_rx_buff[9] << 8) | soil_rx_buff[10];            // pH值

                // 解析数据存储
                moisture_value = moisture / 10.0;                          // 湿度值
                temperature_value = temperature / 10.0;                    // 温度值
                conductivity_value = conductivity;                         // 电导率值
                ph_value = pH / 10.0;                                      // pH值
                nitrogen = (soil_rx_buff[11] << 8) | soil_rx_buff[12];     // 氮含量
                phosphorus = (soil_rx_buff[13] << 8) | soil_rx_buff[14];   // 磷含量
                potassium = (soil_rx_buff[15] << 8) | soil_rx_buff[16];    // 钾含量
            }
            else
            {
                soil_detect_cnt++;
                if (soil_detect_cnt == SOIL_DETECT_CNT)
                {
                    soil_flag = 0;
                    soil_detect_cnt = 0;
                }
            }
        }
        else
        {
            soil_detect_cnt++;
            if (soil_detect_cnt == SOIL_DETECT_CNT)
            {
                soil_flag = 0;
                soil_detect_cnt = 0;
            }
        }
    }
    else
    {
        soil_detect_cnt++;
        if (soil_detect_cnt == SOIL_DETECT_CNT)
        {
            soil_flag = 0;
            soil_detect_cnt = 0;
        }
    }
}


void NB_reboot(void){
    unsigned char ATCommand0[] = "AT+NRB\r\n";                                  // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);    // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                         // 打印发送的内容
    osal_mdelay(10000);                                                          // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                              // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                   // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                      // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
    (void)uapi_watchdog_kick();                                                  // 触发看门狗
}

void NB_ATI(void){
    unsigned char ATCommand0[] = "ATI\r\n";                                     // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);    // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                         // 打印发送的内容
    osal_mdelay(1000);                                                           // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                              // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                   // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                      // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
}

void NB_CIMI(void){
    unsigned char ATCommand0[] = "AT+CIMI\r\n";                                 // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);    // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                         // 打印发送的内容
    osal_mdelay(1000);                                                           // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                              // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                   // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                      // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
}

void NB_QIDNSCFG(void){
    unsigned char ATCommand0[] = "AT+QIDNSCFG=\"8.8.8.8\",\"8.8.4.4\"\r\n";     // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);    // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                         // 打印发送的内容
    osal_mdelay(1000);                                                           // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                              // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                   // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                      // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
}

void NB_QMTOPEN(void){
    unsigned char ATCommand0[] = "AT+QMTOPEN=0,\"189.1.245.236\",1883\r\n";     // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);    // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                         // 打印发送的内容
    osal_mdelay(4000);                                                           // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                              // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                   // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                      // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
}

void NB_QMTCONN(void){
    unsigned char ATCommand0[60] = { 0 };
    sprintf((char*)ATCommand0, "AT+QMTCONN=0,\"%s\",\"user\",\"Sztu1034\"\r\n", client_id);// 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);     // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                          // 打印发送的内容
    osal_mdelay(2000);                                                            // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                               // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                    // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                       // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
}

void NB_QMTSUB(void){
    unsigned char ATCommand0[] = "AT+QMTSUB=0,1,\"/Agriculture/Controll/Curtain\",1\r\n";    // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);     // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                          // 打印发送的内容
    osal_mdelay(2000);                                                            // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                               // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                    // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                       // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
}

void NB_Disable_PSM_eDRX(void) {
    unsigned char ATCommand0[] = "AT+CPSMS=0\r\n";                              // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand0, sizeof(ATCommand0), 0);    // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand0);                                         // 打印发送的内容
    osal_mdelay(300);                                                           // 延时等待重启
    uint8_t rec_buff[100] = { 0 };                                              // 创建接收缓冲区
    memset(rec_buff, 0, 100);                                                   // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                      // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容

    unsigned char ATCommand1[] = "AT+CEDRXS=3,5\r\n";                             // 设置AT指令
    uapi_uart_write(CONFIG_UART2_BUS_ID, ATCommand1, sizeof(ATCommand1), 0);    // 串口发送指令
    printf("发送NB:\n%s\n",ATCommand1);                                         // 打印发送的内容
    osal_mdelay(300);                                                           // 延时等待重启
    memset(rec_buff, 0, 100);                                                   // 清零
    uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);                      // 读取内容
    printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容

}


void NB_Init(void)   //! NB初始化
{
    (void)uapi_watchdog_kick();   // 触发看门狗
    osal_mdelay(500);

    // 软复位NB模组
    NB_reboot();
    // 看基本信息
    NB_ATI();
    // 关闭省电模式
    NB_Disable_PSM_eDRX();
    // 查询IMSI号码
    NB_CIMI();
    // 配置DNS服务器
    NB_QIDNSCFG();
    // 发起MQTT请求
    NB_QMTOPEN();
    (void)uapi_watchdog_kick();   // 触发看门狗
    // 发起MQTT请求
    NB_QMTOPEN();
    // 连接MQTT服务器
    NB_QMTCONN();
    // 订阅MQTT服务器
    NB_QMTSUB();
    
}   // NB_Init

void NB_Upload_Air_Data(char* pub_payload_air, int len)   //! NB上传空气数据
{
    (void)uapi_watchdog_kick();   // 触发看门狗

    if (nb_send_flag)
    {                                                         // 如果NB模块正在发送数据，终止当前发送，以免破坏本次上传
        unsigned char pub_end_air[] = { 0x1A, 0x0D, 0x0A };   // 发送结束标志
        uapi_uart_write(CONFIG_UART2_BUS_ID, pub_end_air, sizeof(pub_end_air), 0);
        osal_mdelay(200);
    }

    nb_send_flag = 1;   // 设置NB发送标志位为1，表示正在发送数据

    // 开始MQTT发布
    unsigned char pub_header_air[] = "AT+QMTPUB=0,0,0,0,\"/Agriculture/Air\"\r\n";
    uapi_uart_write(CONFIG_UART2_BUS_ID, pub_header_air, sizeof(pub_header_air), 0);
    osal_mdelay(200);

    // 发送空气数据
    uapi_uart_write(CONFIG_UART2_BUS_ID, (unsigned char*)pub_payload_air, len, 0);
    osal_mdelay(200);

    // 发送结束标志
    unsigned char pub_end_air[] = { 0x1A, 0x0D, 0x0A };
    uapi_uart_write(CONFIG_UART2_BUS_ID, pub_end_air, sizeof(pub_end_air), 0);
    osal_mdelay(200);

    nb_send_flag = 0;   // 清除NB发送标志位
}   // NB_Upload_Air_Data

void NB_Upload_Soil_Data(char* pub_payload_soil, int len)   //! NB上传土壤数据
{
    (void)uapi_watchdog_kick();   // 触发看门狗

    if (nb_send_flag)
    {                                                         // 如果NB模块正在发送数据，终止当前发送，以免破坏本次上传
        unsigned char pub_end_air[] = { 0x1A, 0x0D, 0x0A };   // 发送结束标志
        uapi_uart_write(CONFIG_UART2_BUS_ID, pub_end_air, sizeof(pub_end_air), 0);
        osal_mdelay(200);
    }

    nb_send_flag = 1;   // 设置NB发送标志位为1，表示正在发送数据

    // 开始MQTT发布
    unsigned char pub_header_soil[] = "AT+QMTPUB=0,0,0,0,\"/Agriculture/Soil\"\r\n";
    uapi_uart_write(CONFIG_UART2_BUS_ID, pub_header_soil, sizeof(pub_header_soil), 0);
    osal_mdelay(200);

    // 发送土壤数据
    uapi_uart_write(CONFIG_UART2_BUS_ID, (unsigned char*)pub_payload_soil, len, 0);
    osal_mdelay(200);

    // 发送结束标志
    unsigned char pub_end_soil[] = { 0x1A, 0x0D, 0x0A };
    uapi_uart_write(CONFIG_UART2_BUS_ID, pub_end_soil, sizeof(pub_end_soil), 0);
    osal_mdelay(200);

    nb_send_flag = 0;   // 清除NB发送标志位
}   // NB_Upload_Soil_Data

void parse_qmtrecv(const char* input)
{
    int tcp_id, msg_id, qos, payload_value;
    char topic[64] = {0};

    int ret = sscanf(input, "+QMTRECV: %d,%d,\"%63[^\"]\",%d,%d",
                     &tcp_id, &msg_id, topic, &qos, &payload_value);

    if (ret == 5) {
        printf("解析成功：\n");
        printf("tcp_id = %d\n", tcp_id);
        printf("msg_id = %d\n", msg_id);
        printf("topic = %s\n", topic);
        printf("qos = %d\n", qos);
        printf("payload = %d\n", payload_value);
        if(payload_value==0){
            uapi_gpio_set_val(RELAY1_GPIO, GPIO_LEVEL_HIGH);
            uapi_gpio_set_val(RELAY2_GPIO, GPIO_LEVEL_LOW);
            osal_mdelay(130000);
            uapi_gpio_set_val(RELAY1_GPIO, GPIO_LEVEL_LOW);
            uapi_gpio_set_val(RELAY2_GPIO, GPIO_LEVEL_LOW);
        }else if(payload_value==1){
            uapi_gpio_set_val(RELAY2_GPIO, GPIO_LEVEL_HIGH);
            uapi_gpio_set_val(RELAY1_GPIO, GPIO_LEVEL_LOW);
            osal_mdelay(130000);
            uapi_gpio_set_val(RELAY1_GPIO, GPIO_LEVEL_LOW);
            uapi_gpio_set_val(RELAY2_GPIO, GPIO_LEVEL_LOW);
        }
    } else {
        printf("解析失败，返回值 = %d\n", ret);
    }
}

void debug_print_raw(const uint8_t* buf, int len)
{
    printf("Raw [HEX]: ");
    for (int i = 0; i < len; ++i)
    {
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

void Relay_Init(void)
{
    uapi_pin_set_mode(RELAY1_GPIO, HAL_PIO_FUNC_GPIO);
    uapi_gpio_set_dir(RELAY1_GPIO, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(RELAY1_GPIO, GPIO_LEVEL_LOW);
    uapi_pin_set_mode(RELAY2_GPIO, HAL_PIO_FUNC_GPIO);
    uapi_gpio_set_dir(RELAY2_GPIO, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(RELAY2_GPIO, GPIO_LEVEL_LOW);
}

static void* Smart_Agriculture_Task(const char* arg)   //! 执行数据采集与上报功能
{
    unused(arg);   // 忽略未使用的参数

    // Uart_Init_Soil();   // 初始化土壤传感器串口
    Uart_Init_NB();     // 初始化NB模组串口
    NB_Init();          // 初始化NB模组
    Relay_Init();

    printf("\n初始化结束\n");
    uint8_t rec_buff[100] = { 0 };                                               // 创建接收缓冲区  

    while (1)
    {
        osal_msleep(COLLECT_DATA_CNT * 1000);   // 这里设置采集周期，休眠时间执行softmesh
        memset(rec_buff, 0, 100);                                                    // 清零
        int len = uapi_uart_read(CONFIG_UART2_BUS_ID, rec_buff, 100, 0);             // 读取内容
        osal_mdelay(200);
        if (len <= 0) {
            printf("串口未收到任何数据。\n");
            continue;
        }
        printf("收到NB:\n%s\n",rec_buff);                                            // 打印收到的内容
        // debug_print_raw(rec_buff,len);
        const char* start = strstr((const char*)rec_buff, "+QMTRECV:");
        if (start) {
            parse_qmtrecv(start);
        } else {
            printf("收到非 MQTT 消息，执行设备重启！\n");
            osal_mdelay(200);         // 给打印一点缓冲时间
            hal_reboot_chip();        // 直接重启芯片
        }
        (void)uapi_watchdog_kick();   // 触发看门狗
    }

    return NULL;
}   // Smart_Agriculture_Task

static void Smart_Agriculture_Entry(void)   //! 设备初始化与线程创建
{
    wifi_softap_get_mac_addr(mac_address, 6);   // 获取MAC地址

    sprintf(client_id, "client-%02x-%02x-%02x-%02x-%02x-%02x", (unsigned char)mac_address[0], (unsigned char)mac_address[1], (unsigned char)mac_address[2], (unsigned char)mac_address[3], (unsigned char)mac_address[4], (unsigned char)mac_address[5]);   // 构造上传id
    printf("[%s] client_id: %s\n", __func__, client_id);

    uint32_t ret = 0;    // 存储返回值
    osal_task* taskid;   // 存储任务句柄

    osal_kthread_lock();   // 创建线程之前加锁

    // 创建数据采集与上报线程
    taskid = osal_kthread_create((osal_kthread_handler)Smart_Agriculture_Task, NULL, "Smart_Agriculture_Task", 0x6000);
    ret = osal_kthread_set_priority(taskid, TASK_PRIO);
    if (ret != OSAL_SUCCESS)
    {
        printf("[%s] Create Smart_Agriculture_Task Failed.\n", __func__);
    }

    osal_kthread_unlock();   // 创建线程之后解锁
}   // Smart_Agriculture_Entry

app_run(Smart_Agriculture_Entry);   //! 调用app_run函数，启动应用程序
