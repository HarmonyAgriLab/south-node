#include "cmsis_os2.h"
#include "hal_wireless.h"   // 调用hal层接口
#include "network_fsm.h"
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>   // 引入动态内存分配
#include <string.h>

// wifi连接事件标志对象
extern osEventFlagsId_t wireless_event_flags;
#define WIRELESS_CONNECT_BIT (1 << 0)
#define WIRELESS_DISCONNECT_BIT (1 << 1)

// 路由传输层开启标志位
osEventFlagsId_t route_transport_event_flags;

#define ROUTE_TRANSPORT_START_BIT (1 << 0)
#define ROUTE_TRANSPORT_STOP_BIT (1 << 1)

// 定义宏开关，打开或关闭日志输出
#define ENABLE_LOG 0   // 1 表示开启日志，0 表示关闭日志

// 定义 LOG 宏，如果 ENABLE_LOG 为 1，则打印日志，并输出文件名、行号、日志内容; 如果为0，则不输出日志，unused传进来的变量
#if ENABLE_LOG
    #define LOG(fmt, ...) printf("LOG [%s:%d]: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
    #define LOG(fmt, ...)
#endif

#define MAX_SCAN_RESULTS 64   // 最大扫描数量
#define MAC_STR_LEN 18        // MAC 地址字符串长度 (17 个字符 + 1 个 '\0')
#define MESH_MAC_LEN 6        // Mesh MAC 地址长度

// 定义当前状态变量
static NetworkState current_state;

// 定义全局 Mesh 网络配置
MeshNetworkConfig g_mesh_config;

// 定义连接配置
static WirelessSTAConfig sta_config;

// 是否是根节点标记位
static bool is_root = false;

// 初始化状态机
int network_fsm_init(const MeshNetworkConfig* config)
{
    // 保存传入的 Mesh 网络配置到全局变量中
    if (config != NULL)
    {
        strncpy(g_mesh_config.mesh_ssid, config->mesh_ssid, sizeof(g_mesh_config.mesh_ssid) - 1);
        strncpy(g_mesh_config.password, config->password, sizeof(g_mesh_config.password) - 1);
        g_mesh_config.mesh_ssid[sizeof(g_mesh_config.mesh_ssid) - 1] = '\0';   // 确保字符串以 '\0' 结尾
        g_mesh_config.password[sizeof(g_mesh_config.password) - 1] = '\0';     // 确保字符串以 '\0' 结尾
        g_mesh_config.tree_level = -1;                                         // -1 表示节点还未完成组网
        LOG("Mesh Network Config: SSID = %s, Password = %s\n", g_mesh_config.mesh_ssid, g_mesh_config.password);
    }
    else
    {
        LOG("MeshNetworkConfig is NULL!\n");
        return -1;
    }
    route_transport_event_flags = osEventFlagsNew(NULL);   // 使用默认属性创建事件标志
    if (route_transport_event_flags == NULL)
    {
        LOG("Failed to create Wi-Fi event flags.\r\n");
    }
    // 设置初始状态为 STATE_STARTUP
    current_state = STATE_STARTUP;
    LOG("Network FSM initialized. Current state: STATE_STARTUP\n");
    return 0;
}

// 状态机的主循环函数
void network_fsm_run(void)
{
    // 持续执行状态机，直到达到终止状态
    while (current_state != STATE_TERMINATE)
    {
        switch (current_state)
        {
            case STATE_STARTUP:
                current_state = state_startup();
                break;
            case STATE_SCANNING:
                current_state = state_scanning();
                break;
            case STATE_JOIN_EXISTING_NETWORK:
                current_state = state_join_existing_network();
                break;
            case STATE_OPEN_AP:
                current_state = state_open_ap();
                break;
            case STATE_CHECK_ROOT_CONFLICT:
                current_state = state_check_root_conflict();
                break;
            case STATE_CREATE_ROOT:
                current_state = state_create_root();
                break;
            case STATE_TERMINATE:
                current_state = state_terminate();
                break;
            default:
                LOG("Unknown state, terminating...\n");
                current_state = STATE_TERMINATE;
                break;
        }
    }
    LOG("State machine terminated.\n");
}

// 启动状态处理函数
NetworkState state_startup(void)
{
    LOG("Node powered on, initializing STA mode...\n");
    // 初始化 Wi-Fi
    if (HAL_Wireless_Init(DEFAULT_WIRELESS_TYPE) == 0)
    {
        LOG("Wi-Fi initialization successfully.\n");
    }
    else
    {
        LOG("Wi-Fi initialization failed.\n");
        return STATE_TERMINATE;
    }

    // 启动 Wi-Fi STA 模式
    if (HAL_Wireless_Start(DEFAULT_WIRELESS_TYPE) == 0)
    {
        LOG("Wi-Fi STA mode started successfully.\n");
    }
    else
    {
        LOG("Failed to start Wi-Fi STA mode.\n");
        return STATE_TERMINATE;
    }

    // 假设初始化完成后进入扫描状态
    return STATE_SCANNING;
}

// 判断是否是有效的树层级字符（0-9, A-Z, a-z）
static int is_valid_tree_level_char(char c)
{
    return (isdigit(c) || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'));
}

// 将树层级转换为数字（0-9对应0-9，A-Z对应10-35，a-z对应36-61）
static int tree_level_to_int(char tree_level_char)
{
    if (isdigit(tree_level_char))
    {
        return tree_level_char - '0';
    }
    else if (tree_level_char >= 'A' && tree_level_char <= 'Z')
    {
        return tree_level_char - 'A' + 10;
    }
    else if (tree_level_char >= 'a' && tree_level_char <= 'z')
    {
        return tree_level_char - 'a' + 36;
    }
    return -1;   // 无效字符
}

// 将数字转换为树层级字符（0-9对应0-9，10-35对应A-Z，36-61对应a-z）
static char int_to_tree_level(int tree_level_int)
{
    if (tree_level_int >= 0 && tree_level_int <= 9)
    {
        return '0' + tree_level_int;
    }
    else if (tree_level_int >= 10 && tree_level_int <= 35)
    {
        return 'A' + tree_level_int - 10;
    }
    else if (tree_level_int >= 36 && tree_level_int <= 61)
    {
        return 'a' + tree_level_int - 36;
    }
    return '\0';   // 无效数字
}

// 从SSID提取MAC地址
static void extract_mac_from_ssid(const char* ssid, size_t prefix_len, uint8_t* mac)
{
    mac[0] = ssid[prefix_len + 1];
    mac[1] = ssid[prefix_len + 2];
    mac[2] = ssid[prefix_len + 3];
    mac[3] = ssid[prefix_len + 4];
    mac[4] = ssid[prefix_len + 5];
    mac[5] = ssid[prefix_len + 6];
}

// 扫描状态处理函数
NetworkState state_scanning(void)
{
    is_root = false;
    LOG("Scanning for existing Mesh networks...\n");

    WirelessScanResult* scan_results = (WirelessScanResult*)malloc(sizeof(WirelessScanResult) * MAX_SCAN_RESULTS);
    if (scan_results == NULL)
    {
        LOG("Memory allocation failed!\n");
        return STATE_TERMINATE;
    }

    printf("sta_config ssid:%s\n", sta_config.ssid);
    // 扫描网络
    int scanned_count = HAL_Wireless_Scan(DEFAULT_WIRELESS_TYPE, &sta_config, scan_results, MAX_SCAN_RESULTS);
    LOG("Found %d networks:\n", scanned_count);

    const char* mesh_prefix = g_mesh_config.mesh_ssid;   // Mesh 自定义 SSID 前缀
    size_t mesh_prefix_len = strlen(mesh_prefix);        // 计算 Mesh 自定义 SSID 的长度
    WirelessScanResult* best_result = NULL;              // 保存最佳网络
    int best_rssi = -9999;                               // 初始化为最小可能的 RSSI
    int best_tree_level = 62;                            // 初始化为最大的树层级（a-z，最大61）
    uint8_t best_mac[6] = { 0 };                         // 保存最大的 MAC 地址

    bool root_mac_defined = memcmp(g_mesh_config.root_mac, (uint8_t[]) { 0, 0, 0, 0, 0, 0 }, sizeof(g_mesh_config.root_mac)) != 0;

    // 遍历所有扫描到的网络
    for (int i = 0; i < scanned_count; i++)
    {
        // 将 bssid 转换为 xx:xx:xx:xx:xx:xx 的格式
        char bssid_str[MAC_STR_LEN];
        snprintf(bssid_str, sizeof(bssid_str), "%02X:%02X:%02X:%02X:%02X:%02X", scan_results[i].bssid[0], scan_results[i].bssid[1], scan_results[i].bssid[2], scan_results[i].bssid[3], scan_results[i].bssid[4], scan_results[i].bssid[5]);

        LOG("  - SSID: %s, RSSI: %d dBm, Channel: %d, BSSID: %s\n", scan_results[i].ssid, scan_results[i].rssi, scan_results[i].channel, bssid_str);

        if (strncmp(scan_results[i].ssid, mesh_prefix, mesh_prefix_len) == 0)
        {
            LOG("Matching Mesh SSID: %s\n", scan_results[i].ssid);
            size_t ssid_len = strlen(scan_results[i].ssid);
            // 提取树层级，假设树层级字符是 SSID 的最后一位
            char tree_level_char = scan_results[i].ssid[ssid_len - 1];

            // 从ssid提取根节点的mac地址
            uint8_t current_mac[6];
            extract_mac_from_ssid(scan_results[i].ssid, mesh_prefix_len, current_mac);

            LOG("  Root node MAC (last 3 bytes) from SSID: %c%c:%c%c:%c%c\n", current_mac[0], current_mac[1], current_mac[2], current_mac[3], current_mac[4], current_mac[5]);

            // 判断是否为有效的树层级字符
            if (is_valid_tree_level_char(tree_level_char))
            {
                int tree_level = tree_level_to_int(tree_level_char);
                LOG("  Tree level: %d, RSSI: %d\n", tree_level, scan_results[i].rssi);

                // 如果 root_mac 已经定义，则只比较相同的 root_mac 节点
                if (root_mac_defined)
                {
                    if (memcmp(g_mesh_config.root_mac, current_mac, MESH_MAC_LEN) == 0)
                    {
                        // 更新最佳结果
                        best_result = &scan_results[i];
                        best_rssi = scan_results[i].rssi;
                        best_tree_level = tree_level;
                        memcpy(best_mac, current_mac, MESH_MAC_LEN);
                        break;   // 找到目标根节点后可以直接跳出循环
                    }
                }
                else
                {
                    // 将扫描到的 Mesh 节点信息保存到数组中
                    if (memcmp(current_mac, best_mac, MESH_MAC_LEN) > 0 || (memcmp(current_mac, best_mac, MESH_MAC_LEN) == 0 && scan_results[i].rssi > best_rssi) || (memcmp(current_mac, best_mac, MESH_MAC_LEN) == 0 && scan_results[i].rssi == best_rssi && tree_level < best_tree_level))
                    {
                        best_result = &scan_results[i];
                        best_rssi = scan_results[i].rssi;
                        best_tree_level = tree_level;
                        memcpy(best_mac, current_mac, 6);
                    }
                }
            }
        }
    }

    if (best_result)
    {
        LOG("Best matching Mesh network found: SSID = %s, RSSI = %d, Tree Level = %d\n", best_result->ssid, best_rssi, best_tree_level);

        // 设置连接的配置
        strncpy(sta_config.ssid, best_result->ssid, sizeof(sta_config.ssid));
        strcpy(sta_config.password, g_mesh_config.password);
        memcpy(sta_config.bssid, best_result->bssid, sizeof(sta_config.bssid));
        memcpy(g_mesh_config.root_mac, best_mac, sizeof(g_mesh_config.root_mac));   // 更新 root_mac
        g_mesh_config.tree_level = best_tree_level + 1;                             // 更新树层级
        sta_config.type = DEFAULT_WIRELESS_TYPE;

        free(scan_results);   // 释放内存
        return STATE_JOIN_EXISTING_NETWORK;
    }
    else
    {
        LOG("No matching Mesh networks found, creating new root node...\n");
        free(scan_results);   // 释放内存
        return STATE_CREATE_ROOT;
    }
}

// 加入已有网络状态处理函数
NetworkState state_join_existing_network(void)
{
    LOG("Joining existing Mesh network...\n");
    // 连接到扫描到的最佳网络
    if (HAL_Wireless_Connect(DEFAULT_WIRELESS_TYPE, &sta_config) == 0)
    {
        LOG("Successfully connected to Mesh network.\n");
        return STATE_OPEN_AP;
    }
    else
    {
        LOG("Failed to connect to Mesh network.\n");
        memset(g_mesh_config.root_mac, 0, MESH_MAC_LEN);               // 清空 root_mac
        memset(sta_config.ssid, 0, sizeof(sta_config.ssid));           // 清空 ssid
        memset(sta_config.password, 0, sizeof(sta_config.password));   // 清空 password
        memset(sta_config.bssid, 0, sizeof(sta_config.bssid));         // 清空 bssid
        return STATE_SCANNING;
    }
}

// 开启 AP 模式状态处理函数
NetworkState state_open_ap(void)
{
    LOG("Opening AP mode...\n");
    char ap_ssid[33];
    char tree_level_char = int_to_tree_level(g_mesh_config.tree_level);
    snprintf(ap_ssid, sizeof(ap_ssid), "%s_%c%c%c%c%c%c_%c", g_mesh_config.mesh_ssid, g_mesh_config.root_mac[0], g_mesh_config.root_mac[1], g_mesh_config.root_mac[2], g_mesh_config.root_mac[3], g_mesh_config.root_mac[4], g_mesh_config.root_mac[5], tree_level_char);

    WirelessAPConfig ap_config;
    strncpy(ap_config.ssid, ap_ssid, sizeof(ap_config.ssid) - 1);
    strncpy(ap_config.password, g_mesh_config.password, sizeof(ap_config.password) - 1);
    ap_config.channel = 6;   // ws63开发板不会生效，设置为0好像会自动筛选一个信道开启，
    ap_config.security = DEFAULT_WIRELESS_SECURITY;
    ap_config.type = DEFAULT_WIRELESS_TYPE;
    if (HAL_Wireless_EnableAP(DEFAULT_WIRELESS_TYPE, &ap_config, g_mesh_config.tree_level) != 0)
    {
        LOG("Failed to start AP mode.\n");
        return STATE_TERMINATE;
    }
    else
    {
        LOG("AP mode started successfully with SSID: %s\n", ap_ssid);
        osEventFlagsSet(route_transport_event_flags, ROUTE_TRANSPORT_START_BIT);
        return STATE_CHECK_ROOT_CONFLICT;
    }
}

// 检查根节点冲突状态处理函数
NetworkState state_check_root_conflict(void)
{
    uint32_t flags = osEventFlagsWait(wireless_event_flags, WIRELESS_CONNECT_BIT | WIRELESS_DISCONNECT_BIT, osFlagsWaitAny, 100);
    LOG("flag:0x%08X\n", flags);
    if (flags & WIRELESS_CONNECT_BIT || flags == osFlagsErrorTimeout)
    {   // 开大节点，关大节点，会导致scan超时，可能是scan接口设计问题，后续将去匹配目标节点的代码去除掉
        LOG("Wi-Fi connected or nothing happen.\n");
        // 成功连接后，继续扫描是否存在其他的mesh网络，如果mesh网络的mac比自己的mac大，则加入该网络
        WirelessScanResult* scan_results = (WirelessScanResult*)malloc(sizeof(WirelessScanResult) * MAX_SCAN_RESULTS);
        if (scan_results == NULL)
        {
            LOG("Memory allocation failed!\n");
            return STATE_TERMINATE;
        }
        int scanned_count = HAL_Wireless_Scan(DEFAULT_WIRELESS_TYPE, &sta_config, scan_results, MAX_SCAN_RESULTS);

        const char* mesh_prefix = g_mesh_config.mesh_ssid;   // Mesh 自定义 SSID 前缀
        size_t mesh_prefix_len = strlen(mesh_prefix);        // 计算 Mesh 自定义 SSID 的长度
        uint8_t current_mac[6];                              // 保存当前节点的 MAC 地址
        memcpy(current_mac, g_mesh_config.root_mac, MESH_MAC_LEN);

        for (int i = 0; i < scanned_count; i++)
        {
            // 只在 ssid_len 大于等于 mesh_prefix_len 时，才进行前缀比较
            if (strncmp(scan_results[i].ssid, mesh_prefix, mesh_prefix_len) == 0)
            {
                // 将 bssid 转换为 xx:xx:xx:xx:xx:xx 的格式
                char bssid_str[MAC_STR_LEN];
                snprintf(bssid_str, sizeof(bssid_str), "%02X:%02X:%02X:%02X:%02X:%02X", scan_results[i].bssid[0], scan_results[i].bssid[1], scan_results[i].bssid[2], scan_results[i].bssid[3], scan_results[i].bssid[4], scan_results[i].bssid[5]);

                LOG("  - SSID: %s, RSSI: %d dBm, Channel: %d, BSSID: %s\n", scan_results[i].ssid, scan_results[i].rssi, scan_results[i].channel, bssid_str);

                // 从ssid提取根节点的mac地址
                uint8_t scanned_mac[6];
                extract_mac_from_ssid(scan_results[i].ssid, mesh_prefix_len, scanned_mac);
                // 打印当前节点的mac地址
                LOG("  Current node MAC (last 3 bytes) from SSID: %c%c:%c%c:%c%c\n", scanned_mac[0], scanned_mac[1], scanned_mac[2], scanned_mac[3], scanned_mac[4], scanned_mac[5]);
                LOG("  Root node MAC (last 3 bytes) from SSID: %c%c:%c%c:%c%c\n", current_mac[0], current_mac[1], scanned_mac[2], current_mac[3], current_mac[4], current_mac[5]);

                // 比较 MAC 地址
                if (memcmp(scanned_mac, current_mac, MESH_MAC_LEN) > 0)
                {
                    LOG("Found a Mesh network with a larger MAC address, updating configuration.\n");
                    // 更新要连接的Mesh网络根节点的MAC标识
                    memcpy(g_mesh_config.root_mac, scanned_mac, MESH_MAC_LEN);
                    strncpy(sta_config.ssid, scan_results[i].ssid, sizeof(sta_config.ssid));
                    strcpy(sta_config.password, g_mesh_config.password);
                    memcpy(sta_config.bssid, scan_results[i].bssid, sizeof(sta_config.bssid));
                    sta_config.type = DEFAULT_WIRELESS_TYPE;
                    // 关闭AP模式
                    HAL_Wireless_DisableAP(DEFAULT_WIRELESS_TYPE);
                    osEventFlagsSet(route_transport_event_flags, ROUTE_TRANSPORT_STOP_BIT);
                    free(scan_results);   // 释放内存
                    return STATE_SCANNING;
                }
            }
        }
        free(scan_results);   // 释放内存
        return STATE_CHECK_ROOT_CONFLICT;
    }
    else if (flags & WIRELESS_DISCONNECT_BIT && is_root == false)
    {
        LOG("Wi-Fi disconnected, taking action.\n");
        memset(g_mesh_config.root_mac, 0, MESH_MAC_LEN);               // 清空 root_mac
        memset(sta_config.ssid, 0, sizeof(sta_config.ssid));           // 清空 ssid
        memset(sta_config.password, 0, sizeof(sta_config.password));   // 清空 password
        memset(sta_config.bssid, 0, sizeof(sta_config.bssid));         // 清空 bssid
        // 关闭AP模式
        HAL_Wireless_DisableAP(DEFAULT_WIRELESS_TYPE);
        osEventFlagsSet(route_transport_event_flags, ROUTE_TRANSPORT_STOP_BIT);
        osDelay(100);   // 延迟 1s，等待断连的WiFi完全消失
        return STATE_SCANNING;
    }
    else
    {
        return STATE_CHECK_ROOT_CONFLICT;
    }
}

// 创建根节点状态处理函数
NetworkState state_create_root(void)
{
    LOG("Creating new root node for Mesh network...\n");
    char root_ssid[33];
    uint8_t ap_mac[6];
    memset(ap_mac, 0, sizeof(ap_mac));
    HAL_Wireless_GetAPMacAddress(DEFAULT_WIRELESS_TYPE, ap_mac);
    // 将mac后三位转换为字符写入g_mesh_config.root_mac
    char temp_mac[7];
    snprintf(temp_mac, sizeof(temp_mac), "%02X%02X%02X", ap_mac[3], ap_mac[4], ap_mac[5]);
    memcpy(g_mesh_config.root_mac, temp_mac, MESH_MAC_LEN);
    snprintf(root_ssid, sizeof(root_ssid), "%s_%02X%02X%02X_0", g_mesh_config.mesh_ssid, ap_mac[3], ap_mac[4], ap_mac[5]);
    WirelessAPConfig ap_config;
    strncpy(ap_config.ssid, root_ssid, sizeof(ap_config.ssid) - 1);
    strncpy(ap_config.password, g_mesh_config.password, sizeof(ap_config.password) - 1);
    g_mesh_config.tree_level = 0;
    ap_config.channel = 6;   // ws63开发板不会生效，设置为0好像会自动筛选一个信道开启，
    ap_config.security = DEFAULT_WIRELESS_SECURITY;
    ap_config.type = DEFAULT_WIRELESS_TYPE;
    if (HAL_Wireless_EnableAP(DEFAULT_WIRELESS_TYPE, &ap_config, 0) != 0)
    {
        LOG("Failed to start AP mode.\n");
        return STATE_TERMINATE;
    }
    else
    {
        LOG("AP mode started successfully with SSID: %s\n", root_ssid);
        osEventFlagsSet(route_transport_event_flags, ROUTE_TRANSPORT_START_BIT);
        is_root = true;
    }
    return STATE_CHECK_ROOT_CONFLICT;
}

NetworkState state_terminate(void)
{
    LOG("Terminating network state machine...\n");
    return STATE_TERMINATE;
}

int network_connected(void)
{
    if (current_state == STATE_CHECK_ROOT_CONFLICT)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}