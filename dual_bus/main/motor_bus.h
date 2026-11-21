#pragma once
/**
 * @file motor_bus.h
 * @brief 双 UART 总线上的电机自动发现与统一寻址。
 *
 * 硬件：
 *   总线 A = UART1, TX=GPIO4,  RX=GPIO5
 *   总线 B = UART2, TX=GPIO8,  RX=GPIO9
 *   微动开关输入 = GPIO15（内部下拉，未压到=低，压到=3.3V）
 *   MOS 输出     = GPIO16（推挽输出，上电默认低）
 *
 * 全局 ID 编码（无需映射表，自动识别也不影响稳定性）：
 *   ID = 总线号 × 1000 + 电机地址
 *   总线 A → 1..255        总线 B → 1001..1255
 *   例：ID 3  = A 路地址 3；  ID 1001 = B 路地址 1
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "zdt_x42.h"

/* ================= 硬件配置（改这里即可换脚） ================= */
#define BUS_A_UART          UART_NUM_1
#define BUS_A_TX_GPIO       4
#define BUS_A_RX_GPIO       5

#define BUS_B_UART          UART_NUM_2
#define BUS_B_TX_GPIO       8
#define BUS_B_RX_GPIO       9

#define PIN_LIMIT_SW        GPIO_NUM_15   /* 微动开关：输入 + 内部下拉 */
#define PIN_MOS             GPIO_NUM_16   /* 外部 MOS：输出，默认低 */

#define MOTOR_BAUD          115200
#define MOTOR_TIMEOUT_MS    200           /* 正常事务超时 */

/* ================= 扫描参数 ================= */
#define MOTOR_MAX           32            /* 最多登记多少台 */
#define SCAN_ADDR_FROM      1             /* 默认扫描地址范围 */
#define SCAN_ADDR_TO        16
#define SCAN_TIMEOUT_MS     25            /* 探测超时：有电机的地址 1~3ms 就应答 */

/* ================= 全局 ID 编码 ================= */
#define BUS_A_ID_BASE       0
#define BUS_B_ID_BASE       1000

typedef struct {
    uint8_t     addr;      /**< 电机在总线上的地址 */
    uart_port_t bus;       /**< 所属 UART */
    uint16_t    fw;        /**< 固件版本 */
    uint16_t    hw;        /**< 硬件版本 */
} motor_info_t;

/** 初始化两路 UART 总线 */
esp_err_t motor_bus_init(void);

/**
 * @brief 扫描两条总线，重建电机登记表
 * @param from 起始地址（含），实际用法一般从 1 开始
 * @param to   结束地址（含）
 * @return 发现的电机数量
 * @note  会清空原有登记表；地址 0 是广播地址，会被跳过
 */
int motor_scan(int from, int to);

/** 已登记的电机数量 */
size_t motor_count(void);

/** 取第 i 台电机信息（越界返回 NULL） */
const motor_info_t *motor_at(size_t i);

/** 按全局 ID 查电机句柄（未找到返回 NULL） */
zdt_x42_t *motor_find(int id);

/** 全局 ID：总线号×1000 + 地址 */
int motor_id_of(const motor_info_t *info);

/** 向所有在用总线广播同步启动（GO） */
esp_err_t motor_sync_start_all(void);

/* ================= IO ================= */

/** 初始化 GPIO15 输入（下拉）+ GPIO16 输出（默认低） */
esp_err_t io_init(void);

/** 读微动开关电平：0 = 未触发（下拉），1 = 触发 */
int io_limit_level(void);

/** 设置 MOS 电平 */
void io_mos_set(int level);

/** 读 MOS 当前电平 */
int io_mos_level(void);
