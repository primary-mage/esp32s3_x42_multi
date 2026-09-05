#pragma once
/**
 * @file vib.h
 * @brief 时间轴振动（定时驱动）：PC 下发频率/振幅/时长，ESP32 本地按时切换方向，
 *        每半周期发相对位置命令（押住+广播同步起步），不等到位（振幅由物理上限决定）。
 *        偶数个半周期后回到起振中心再停。
 */
#include "zdt_x42.h"

typedef enum {
    VIB_IDLE = 0,     /**< 空闲 */
    VIB_RUNNING,      /**< 振动中 */
    VIB_DONE,         /**< 完成（已回中心停） */
    VIB_FAILED,       /**< 失败（堵转保护/命令被拒） */
    VIB_STOPPED,      /**< 已停止（VIBSTP 优雅停 / STP 0 急停） */
} vib_state_t;

/** 初始化（m1/m2/m3 对应 ID=1/2/3 的句柄） */
void vib_init(zdt_x42_t *m1, zdt_x42_t *m2, zdt_x42_t *m3);

/**
 * @brief 启动振动
 * @param id            1 = X 轴（电机1）；2 = Y 轴龙门（电机2+3）
 * @param freq_dhz      频率 ×10（20 = 2.0Hz，范围 1~100 即 0.1~10Hz）
 * @param amp_tenth_mm  半幅 ×10（50 = 中心±5mm）
 * @param dur_s         持续秒数，0 = 持续到 VIBSTP
 * @param mirror        电机3 镜像（仅 Y 轴生效）
 * @return ESP_OK / ESP_ERR_INVALID_ARG / ESP_ERR_INVALID_STATE(忙)
 */
esp_err_t vib_start(int id, int freq_dhz, int amp_tenth_mm, int dur_s, bool mirror);

/** 优雅停止：当前半周期到位后回中心停（状态 VIB_STOPPED） */
void vib_stop(void);

/** 立即中止（配合 STP 0 急停使用，不回中心） */
void vib_abort(void);

/** 当前状态 */
vib_state_t vib_state(void);

/** 已执行半周期数与运行时长 ms（PC 据此计算实际频率） */
void vib_stats(uint32_t *cycles, uint32_t *elapsed_ms);
