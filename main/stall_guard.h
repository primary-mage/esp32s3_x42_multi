#pragma once
/**
 * @file stall_guard.h
 * @brief 堵转自动恢复守护：检测到堵转保护后，自动解除、反向退一圈并把当前位置设为零点。
 */
#include <stddef.h>
#include "zdt_x42.h"

/** 启动守护任务（周期轮询各电机堵转状态） */
esp_err_t stall_guard_init(zdt_x42_t *motors, size_t count);

/** 记录电机最近一次运动方向（命令解析层在 POS/SPD 时调用，index = id-1） */
void stall_guard_set_dir(size_t index, bool cw);

/** 挂起/恢复守护（双机回零等成对操作期间应挂起，防止单边恢复扭龙门） */
void stall_guard_bypass(bool bypass);

/** 永久启用/禁用守护（用户 UI 场景建议禁用，由上位机处理堵转） */
void stall_guard_set_enabled(bool enabled);
