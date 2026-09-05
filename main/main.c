/**
 * @file main.c
 * @brief ESP32-S3 一对多控制张大头 X42S_V2.0（Emm42_V5.0 出厂固件）。
 *
 * 总线：UART1，GPIO4=TX（接电机 R/A/H），GPIO5=RX（接电机 T/B/L），115200 8N1。
 * 本固件为“执行器”模式：不做任何自动运动，只等上位机（PC Python UI）通过
 * USB 串口发 ASCII 命令（见 console_cmd.h），解析后调用驱动执行。
 */
#include <stdio.h>
#include "esp_log.h"
#include "console_cmd.h"
#include "pair_home.h"
#include "stall_guard.h"
#include "zdt_x42.h"

static const char *TAG = "app";

/* ================= 硬件配置 ================= */
#define MOTOR_UART    UART_NUM_1
#define PIN_MOTOR_TX  4            /* ESP32-S3 TX -> 电机 R/A/H */
#define PIN_MOTOR_RX  5            /* ESP32-S3 RX -> 电机 T/B/L */
#define MOTOR_BAUD    115200

#define MOTOR_COUNT   3            /* 总线上挂 3 台，地址 1~3 */

static zdt_x42_t g_motors[MOTOR_COUNT];

/* ================= 启动 ================= */

/** 上电自检：读每台固件版本，确认地址/波特率/协议正确 */
static void self_check(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint16_t fw = 0, hw = 0;
        esp_err_t e = zdt_x42_read_version(&g_motors[i], &fw, &hw);
        if (e == ZDT_OK) {
            ESP_LOGI(TAG, "[电机%d] 固件版本=%u, 硬件版本=%u", i + 1, fw, hw);
        } else {
            ESP_LOGW(TAG, "[电机%d] 读版本失败: %d", i + 1, (int)e);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(zdt_x42_bus_init(MOTOR_UART, PIN_MOTOR_TX, PIN_MOTOR_RX, MOTOR_BAUD));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        zdt_x42_motor_init(&g_motors[i], MOTOR_UART, (uint8_t)(i + 1));
    }

    self_check();

    /* 上电默认使能全部电机，上位机也可单独控制 */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        zdt_x42_enable(&g_motors[i], true, false);
    }

    /* 启动上位机命令行任务：解析 ASCII 协议并执行（不自动运动） */
    console_cmd_init(g_motors, MOTOR_COUNT);

    /* 启动堵转自动恢复守护：堵转保护触发后自动解除、反向退一圈、设零点 */
    stall_guard_init(g_motors, MOTOR_COUNT);

    /* 电机2/3 双机回零状态机（刚性龙门） */
    pair_home_init(&g_motors[1], &g_motors[2]);

    /* 电机1（X轴）回零状态机：碰撞找端 -> 退一圈 -> 设零点 */
    xhome_init(&g_motors[0]);

    ESP_LOGI(TAG, "命令模式就绪，等待上位机...");
}
