#pragma once
/**
 * @file zdt_x42.h
 * @brief 张大头闭环步进驱动 Emm42_V5.0 固件（x42s_v2.0 出厂固件）串口协议驱动，
 *        支持一对多（一条 UART 挂多台，按地址区分）。
 *
 * 注意：Emm42_V5.0 协议与 ZDT_X 系列 V2.0 协议不同！
 *  - 速度单位是 RPM（整数，不是 ×10）；加速度是 1 字节“档位”（0 = 无曲线）；
 *  - 位置命令单位是“脉冲数”（16 细分下 3200 脉冲/圈，与细分设置相关）；
 *  - 位置读取值 0~65535 表示一圈，转角度 = 值 × 360 / 65536。
 *
 * 帧格式：地址 | 功能码 | 数据 | 校验字节（默认固定 0x6B）
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 单帧最大长度（梯形位置命令 16 字节 + 校验） */
#define ZDT_X42_RX_BUF_LEN 64

/** 错误码 */
typedef enum {
    ZDT_OK = 0,          /**< 成功 */
    ZDT_ERR_NULL,        /**< 参数空 */
    ZDT_ERR_TX,          /**< 发送失败 */
    ZDT_ERR_TIMEOUT,     /**< 超时无应答 */
    ZDT_ERR_COND,        /**< 返回 E2：条件不满足（未使能/堵转保护等） */
    ZDT_ERR_REJECT,      /**< 返回 01 00 EE 6B：命令错误（校验/格式不对） */
    ZDT_ERR_FRAME,       /**< 应答帧解析失败 */
} zdt_x42_err_t;

/** 校验方式（与驱动菜单 Checksum 对应） */
typedef enum {
    ZDT_CHK_6B = 0,      /**< 固定 0x6B，出厂默认 */
    ZDT_CHK_XOR,         /**< 全帧异或 */
} zdt_x42_checksum_t;

/** 电机实例句柄：一个句柄 = 总线上一台电机 */
typedef struct {
    uart_port_t uart_num;   /**< 使用的 UART 口 */
    uint8_t     addr;       /**< 电机地址 1~255 */
    zdt_x42_checksum_t checksum; /**< 校验方式 */
    uint32_t    timeout_ms; /**< 单条命令等待应答超时，默认 200ms */
} zdt_x42_t;

/** 状态标志（0x3A 返回字节） */
typedef struct {
    uint8_t enable : 1;     /**< 使能状态 */
    uint8_t inpos  : 1;     /**< 到位 */
    uint8_t stall  : 1;     /**< 堵转 */
    uint8_t clog   : 1;     /**< 堵转保护触发 */
} zdt_x42_status_t;

/**
 * @brief 初始化电机总线（整个总线只调用一次）
 * @param uart_num UART 口（建议 UART_NUM_1，UART0 留给日志）
 * @param tx_gpio  ESP32 TX 引脚（接电机 R/A/H = RX）
 * @param rx_gpio  ESP32 RX 引脚（接电机 T/B/L = TX）
 * @param baud     波特率，默认 115200
 */
esp_err_t zdt_x42_bus_init(uart_port_t uart_num, int tx_gpio, int rx_gpio, int baud);

/** @brief 初始化一个电机句柄（每台电机一个） */
void zdt_x42_motor_init(zdt_x42_t *m, uart_port_t uart_num, uint8_t addr);

/**
 * @brief 底层事务：组帧（补校验）-> 发送 -> 等待本机应答帧
 * @param tx     不含校验字节的帧（tx[0] 必须为地址）
 * @param rx     收到的应答帧（含校验字节）
 * @note 内部有互斥锁，多个任务可安全调用（事务自动串行化）；
 *       tx[0]=0 视为广播，不等待应答。
 */
esp_err_t zdt_x42_transact(zdt_x42_t *m, const uint8_t *tx, size_t tx_len,
                           uint8_t *rx, size_t rx_cap, size_t *rx_len);

/* ================= 控制命令 ================= */

/** 使能/去使能电机（01 F3 AB xx xx 6B） */
esp_err_t zdt_x42_enable(zdt_x42_t *m, bool enable, bool sync);

/**
 * @brief 速度模式（01 F6 ... 6B）
 * @param speed_rpm  目标转速，单位 RPM（例 1500）
 * @param acc_gear   加速度档位 0~255，0 = 不用曲线直接到目标速度；
 *                   曲线加减速时间：每 +1RPM 用时 (256-acc)×50us
 * @param cw         方向：true=顺时针(CW)，false=逆时针(CCW)
 */
esp_err_t zdt_x42_speed_mode(zdt_x42_t *m, bool cw, uint16_t speed_rpm,
                             uint8_t acc_gear, bool sync);

/**
 * @brief 位置模式（01 FD ... 6B，EMM5.0 唯一位置命令，含曲线加减速）
 * @param speed_rpm  转速 RPM
 * @param acc_gear   加速度档位 0~255（同速度模式）
 * @param pulses     目标脉冲数（16 细分下 3200 脉冲 = 1 圈）
 * @param absolute   true=绝对位置，false=相对当前位置
 */
esp_err_t zdt_x42_pos_mode(zdt_x42_t *m, bool cw, uint16_t speed_rpm,
                           uint8_t acc_gear, uint32_t pulses, bool absolute,
                           bool sync);

/** 立即停止（01 FE 98 xx 6B） */
esp_err_t zdt_x42_stop(zdt_x42_t *m, bool sync);

/**
 * @brief 多机同步启动（广播 00 FF 66 6B）
 * @note 先对每台电机下发 sync=true 的运动命令，再调用本函数，全部同时起步。
 *       广播不等待应答（只有地址 1 会回一条，下次事务自动清掉）。
 */
esp_err_t zdt_x42_sync_start(zdt_x42_t *m);

/** 触发回零（01 9A mode xx 6B），mode 见说明书回零模式 */
esp_err_t zdt_x42_home(zdt_x42_t *m, uint8_t home_mode, bool sync);

/** 回零参数（0x4C AE 写 / 0x22 读） */
typedef struct {
    uint8_t  mode;       /**< 0=单圈就近 1=单圈方向 2=多圈碰撞 3=多圈限位开关 */
    bool     dir_cw;     /**< 回零方向 */
    uint16_t speed_rpm;  /**< 回零速度 RPM */
    uint32_t timeout_ms; /**< 回零超时 */
    uint16_t clog_rpm;   /**< 碰撞检测转速 RPM（回零速度应高于它） */
    uint16_t clog_ma;    /**< 碰撞检测电流 mA（正常带载电流与堵转电流之间） */
    uint16_t clog_ms;    /**< 碰撞检测时间 ms */
    bool     auto_home;  /**< 上电自动回零 */
} zdt_x42_home_params_t;

/** 修改回零参数（01 4C AE ...），store=true 掉电保存 */
esp_err_t zdt_x42_set_home_params(zdt_x42_t *m, const zdt_x42_home_params_t *p, bool store);

/** 读取回零参数（01 22 6B） */
esp_err_t zdt_x42_read_home_params(zdt_x42_t *m, zdt_x42_home_params_t *p);

/** 读回零状态标志（01 3B 6B）：bit0编码器就绪 bit1校准表就绪 bit2正在回零 bit3回零失败 */
esp_err_t zdt_x42_read_home_status(zdt_x42_t *m, uint8_t *flags);

/** 当前位置清零（01 0A 6D 6B），仅清零位置计数器，掉电不保存 */
esp_err_t zdt_x42_clear_position(zdt_x42_t *m);

/** 设置单圈回零零点为当前位置（01 93 88 xx 6B），store=true 掉电保存 */
esp_err_t zdt_x42_set_single_zero(zdt_x42_t *m, bool store);

/** 解除堵转保护（01 0E 52 6B） */
esp_err_t zdt_x42_release_stall(zdt_x42_t *m);

/** 触发编码器校准（01 06 45 6B），对应屏幕 Cal 菜单（空载执行） */
esp_err_t zdt_x42_trigger_calibration(zdt_x42_t *m);

/** 恢复出厂设置（01 0F 5F 6B），之后需重新上电并重新校准 */
esp_err_t zdt_x42_restore_factory(zdt_x42_t *m);

/** 修改细分（01 84 8A xx mm 6B），00 表示 256 细分 */
esp_err_t zdt_x42_set_subdivision(zdt_x42_t *m, uint8_t microstep, bool store);

/** 切换开环/闭环模式（01 46 69 xx mm 6B）：1=开环，2=闭环 */
esp_err_t zdt_x42_set_control_mode(zdt_x42_t *m, uint8_t mode, bool store);

/* ================= 读取命令 ================= */

/** 读状态标志（01 3A 6B） */
esp_err_t zdt_x42_read_status(zdt_x42_t *m, zdt_x42_status_t *st);

/**
 * @brief 读实时位置（01 36 6B），返回原始值：±(0~65535) 表示一圈
 * @note 转角度：deg = raw × 360.0 / 65536.0（见 zdt_x42_raw_to_deg）
 */
esp_err_t zdt_x42_read_position(zdt_x42_t *m, int32_t *pos_raw);

/** 读实时转速（01 35 6B），单位 RPM（整数） */
esp_err_t zdt_x42_read_speed(zdt_x42_t *m, int32_t *speed_rpm);

/** 读校准后编码器值（01 31 6B），一圈 0~65535 */
esp_err_t zdt_x42_read_encoder(zdt_x42_t *m, uint16_t *raw);

/** 读总线电压（01 24 6B），单位 mV */
esp_err_t zdt_x42_read_bus_voltage(zdt_x42_t *m, uint16_t *mv);

/** 读相电流（01 27 6B），单位 mA */
esp_err_t zdt_x42_read_phase_current(zdt_x42_t *m, uint16_t *ma);

/** 读固件/硬件版本（01 1F 6B），返回 2 字节版本号，如 fw=201 → V5.0.1 */
esp_err_t zdt_x42_read_version(zdt_x42_t *m, uint16_t *fw, uint16_t *hw);

/** 位置原始值转角度（度）：0~65535 = 一圈 */
static inline double zdt_x42_raw_to_deg(int32_t raw)
{
    return (double)raw * 360.0 / 65536.0;
}

/** 调试开关：为 true 时每次事务打印收发原始字节（HEX）和耗时 */
extern bool zdt_x42_dump_rx;

#ifdef __cplusplus
}
#endif
