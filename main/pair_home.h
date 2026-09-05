#pragma once
/**
 * @file pair_home.h
 * @brief 电机2/3 双机回零状态机（刚性龙门 Y 轴）。
 *
 * 流程：押住两机回零命令并广播同时起步 -> 各自碰撞找端（先到的等后到的）
 *       -> 两机都触发后解除堵转保护、反向退一圈（退出弹性限位压紧区）
 *       -> 两机位置清零设零点。
 * 执行期间堵转守护任务挂起，避免单边自动恢复扭龙门。
 */
#include "zdt_x42.h"

typedef enum {
    PAIR_HOME_IDLE = 0,     /**< 空闲 */
    PAIR_HOME_HOMING,       /**< 两机回零中（先到的等后到的） */
    PAIR_HOME_BACKOFF,      /**< 两机都触发，反向退圈中 */
    PAIR_HOME_ZEROING,      /**< 设置零点中 */
    PAIR_HOME_DONE,         /**< 完成 */
    PAIR_HOME_FAILED,       /**< 失败（超时/回零失败） */
} pair_home_state_t;

/** 初始化状态机（m2/m3 对应 ID=2/3 的句柄） */
esp_err_t pair_home_init(zdt_x42_t *m2, zdt_x42_t *m3);

/** 启动双机回零；运行中再次调用返回 ESP_ERR_INVALID_STATE */
esp_err_t pair_home_start(void);

/** 当前状态 */
pair_home_state_t pair_home_state(void);

/** 最近一次两机回零状态标志（HSTAT flags，电机2/3） */
void pair_home_flags(uint8_t *f2, uint8_t *f3);

/* ============ 电机1（X轴）单机回零 + 退圈设零点 ============ */

/** 初始化 X 轴回零状态机 */
esp_err_t xhome_init(zdt_x42_t *m1);

/** 启动 X 轴回零：碰撞找端 -> 退一圈 -> 设零点；运行中再调返回 ESP_ERR_INVALID_STATE */
esp_err_t xhome_start(void);

/** X 轴回零状态 */
pair_home_state_t xhome_state(void);

/** 最近一次 X 轴回零状态标志 */
void xhome_flags(uint8_t *f1);
