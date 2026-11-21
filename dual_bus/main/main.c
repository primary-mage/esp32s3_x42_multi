/**
 * @file main.c
 * @brief ESP32-S3 双 UART 总线电机测试固件。
 *
 * 硬件：
 *   总线 A = UART1  TX=GPIO4  RX=GPIO5   （接张大头 X42 闭环步进）
 *   总线 B = UART2  TX=GPIO8  RX=GPIO9   （另一路，地址空间独立）
 *   微动开关 = GPIO15  输入 + 内部下拉（未压到 = 0，压到 = 3.3V）
 *   MOS 控制 = GPIO16  推挽输出，上电默认低
 *
 * 本固件只做「执行器」：不自动运动，等上位机发 ASCII 命令。
 * 上电自动扫描两条总线上的电机，之后可用 SCAN 重新扫描。
 *
 * 全局 ID：总线 A → 1..255，总线 B → 1001..1255
 */
#include "esp_log.h"

#include "console_cmd.h"
#include "motor_bus.h"

static const char *TAG = "app";

void app_main(void)
{
    ESP_ERROR_CHECK(motor_bus_init());
    ESP_ERROR_CHECK(io_init());

    ESP_LOGI(TAG, "双总线电机测试固件就绪，等待上位机命令...");
    console_cmd_start();
}
