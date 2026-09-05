/**
 * @file stall_guard.c
 * @brief 堵转自动恢复守护实现。
 *
 * 触发条件：状态字节 bit3(堵转保护) = 1，且当前不在回零中（碰撞回零的堵转是预期行为）。
 * 恢复流程：停止 -> 解除堵转保护 -> 重新使能 -> 按最近运动方向的
 *           反方向退一圈(3200脉冲@16细分) -> 到位后把当前位置设为零点。
 * 触发后有 3 秒冷却，避免反复触发。
 */
#include "stall_guard.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GUARD_PERIOD_MS      200    /* 轮询周期 */
#define REVERSE_PULSES       3200   /* 退一圈（16 细分；改细分后需同步修改） */
#define REVERSE_SPEED_RPM    100
#define REVERSE_ACCEL        5
#define REVERSE_TIMEOUT_MS   10000  /* 退圈等待到位超时 */
#define COOLDOWN_MS          3000   /* 恢复后的冷却期，防反复触发 */

static zdt_x42_t *s_motors;
static size_t      s_count;
static bool        s_last_cw[16];        /* 每台最近运动方向，默认 CW */
static int64_t     s_last_recover_ms[16];/* 每台最近恢复时刻 */
static volatile bool s_bypass;           /* 临时挂起标志（双机回零期间） */
static volatile bool s_enabled = true;   /* 永久开关（GUARD 命令控制） */

static const char *TAG = "guard";

void stall_guard_set_enabled(bool enabled)
{
    s_enabled = enabled;
    ESP_LOGI(TAG, "堵转守护%s", enabled ? "已启用" : "已禁用（由上位机处理）");
}

void stall_guard_bypass(bool bypass)
{
    s_bypass = bypass;
    if (bypass) {
        ESP_LOGW(TAG, "守护已挂起（成对操作中）");
    } else {
        ESP_LOGI(TAG, "守护已恢复");
    }
}

void stall_guard_set_dir(size_t index, bool cw)
{
    if (index < sizeof(s_last_cw)) {
        s_last_cw[index] = cw;
    }
}

/** 等待"真的动起来再停下来"（避免旧到位标志误判完成） */
static bool wait_move_done(zdt_x42_t *m, int32_t pos_before, uint32_t timeout_ms)
{
    int64_t deadline = (int64_t)esp_timer_get_time() / 1000 + timeout_ms;
    bool started = false;
    while ((int64_t)esp_timer_get_time() / 1000 < deadline) {
        zdt_x42_status_t st = {0};
        int32_t pos = 0;
        if (zdt_x42_read_status(m, &st) == ZDT_OK &&
            zdt_x42_read_position(m, &pos) == ZDT_OK) {
            int32_t delta = pos - pos_before;
            if (delta < 0) {
                delta = -delta;
            }
            if (!started) {
                if (delta > 1000) {
                    started = true;
                }
            } else if (st.inpos) {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

static void recover(size_t idx, zdt_x42_t *m)
{
    ESP_LOGW(TAG, "[电机%d] 堵转保护触发，自动恢复：停止->解除->反向退一圈->设零点", (int)idx + 1);

    zdt_x42_stop(m, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    zdt_x42_release_stall(m);
    vTaskDelay(pdMS_TO_TICKS(100));
    zdt_x42_enable(m, true, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    bool back = !s_last_cw[idx];
    int32_t pos0 = 0;
    zdt_x42_read_position(m, &pos0);
    esp_err_t e = zdt_x42_pos_mode(m, back, REVERSE_SPEED_RPM, REVERSE_ACCEL,
                                   REVERSE_PULSES, false, false);
    if (e != ZDT_OK) {
        ESP_LOGW(TAG, "[电机%d] 反向运动命令失败: %d", (int)idx + 1, (int)e);
        return;
    }
    /* 必须先看到位置开始变化，再等到位 */
    if (wait_move_done(m, pos0, REVERSE_TIMEOUT_MS)) {
        zdt_x42_clear_position(m);
        ESP_LOGI(TAG, "[电机%d] 恢复完成：反向退一圈，当前位置已设为零点", (int)idx + 1);
    } else {
        ESP_LOGW(TAG, "[电机%d] 反向运动未到位（超时）", (int)idx + 1);
    }
}

static void guard_task(void *arg)
{
    while (1) {
        if (!s_enabled || s_bypass) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        for (size_t i = 0; i < s_count; i++) {
            int64_t now = (int64_t)esp_timer_get_time() / 1000;
            if (now - s_last_recover_ms[i] < COOLDOWN_MS) {
                continue;   /* 冷却期内不干预 */
            }
            zdt_x42_status_t st = {0};
            if (zdt_x42_read_status(&s_motors[i], &st) != ZDT_OK) {
                continue;   /* 未连接/通信失败，跳过 */
            }
            if (!st.clog) {
                continue;
            }
            uint8_t hflags = 0;
            if (zdt_x42_read_home_status(&s_motors[i], &hflags) == ZDT_OK &&
                (hflags & 0x04)) {
                continue;   /* 正在回零：碰撞是预期行为，不干预 */
            }
            s_last_recover_ms[i] = now;
            recover(i, &s_motors[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(GUARD_PERIOD_MS));
    }
}

esp_err_t stall_guard_init(zdt_x42_t *motors, size_t count)
{
    if (count > sizeof(s_last_cw)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_motors = motors;
    s_count = count;
    for (size_t i = 0; i < count; i++) {
        s_last_cw[i] = true;
        s_last_recover_ms[i] = -COOLDOWN_MS;   /* 上电即可触发 */
    }
    xTaskCreate(guard_task, "stall_guard", 4096, NULL, 5, NULL);
    return ESP_OK;
}
