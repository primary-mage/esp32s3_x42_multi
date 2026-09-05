#pragma once
/**
 * @file console_cmd.h
 * @brief 上位机 ASCII 命令行协议：PC 通过 USB 串口发命令，ESP32 执行电机动作。
 */
#include <stddef.h>
#include "zdt_x42.h"

/**
 * @brief 启动命令行任务（阻塞读 stdin）
 * @param motors 电机句柄数组
 * @param count  电机数量（地址 = 下标 + 1）
 */
esp_err_t console_cmd_init(zdt_x42_t *motors, size_t count);
