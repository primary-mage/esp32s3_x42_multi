/**
 * @file vib.c
 * @brief 时间轴振动实现（见 vib.h）。
 *
 * 运动生成：
 *   - 半周期 T/2 = 1/(2f)，每个半周期起点处下发相对位置命令（REL）
 *   - 序列：+A, -2A, +2A, ... 偶数个半周期后恰好回到中心
 *   - 定时驱动：到点就切方向，不等到位（高频下振幅由电机物理上限决定）
 *   - 速度：振动模式不限速，RPM 用电机上限 3000；加速度档位 255
 *   - 堵转保护触发或命令被拒 -> 立即急停，状态 VIB_FAILED
 *   - 振动期间挂起堵转守护，防止守护擅自运动
 */
#include "vib.h"
#include "stall_guard.h"
#include "esp_timer.h"
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VIB_PULSES_PER_MM 800   /* 16细分 3200脉冲/圈 ÷ 4mm 导程 */
#define VIB_MAX_RPM       3000  /* 振动模式不限速 */
#define VIB_ACC_GEAR      255   /* 最快加速度档位 */

static zdt_x42_t *s_m1, *s_m2, *s_m3;

static volatile vib_state_t s_state = VIB_IDLE;
static volatile uint32_t s_cycles = 0;
static volatile uint32_t s_elapsed_ms = 0;
static volatile bool s_stop_req = false;
static volatile bool s_abort = false;
static TaskHandle_t s_task = NULL;

static int      s_id;        /* 1=X 2=Y对 */
static uint32_t s_a_pul;     /* 半幅脉冲数 */
static uint32_t s_half_ms;   /* 半周期 ms */
static uint32_t s_n_half;    /* 总半周期数（偶数），0=持续到停止 */
static uint8_t  s_mirror;    /* 电机3 镜像 */

void vib_init(zdt_x42_t *m1, zdt_x42_t *m2, zdt_x42_t *m3)
{
    s_m1 = m1;
    s_m2 = m2;
    s_m3 = m3;
}

vib_state_t vib_state(void)
{
    return s_state;
}

void vib_stats(uint32_t *cycles, uint32_t *elapsed_ms)
{
    if (cycles) {
        *cycles = s_cycles;
    }
    if (elapsed_ms) {
        *elapsed_ms = s_elapsed_ms;
    }
}

/** 下发一个半周期运动（Y 轴双机押住+广播同步起步） */
static esp_err_t issue_half(bool cw, uint32_t pulses)
{
    if (s_id == 1) {
        return zdt_x42_pos_mode(s_m1, cw, VIB_MAX_RPM, VIB_ACC_GEAR,
                                pulses, false, false);
    }
    esp_err_t e1 = zdt_x42_pos_mode(s_m2, cw, VIB_MAX_RPM, VIB_ACC_GEAR,
                                    pulses, false, true);
    esp_err_t e2 = zdt_x42_pos_mode(s_m3, s_mirror ? !cw : cw,
                                    VIB_MAX_RPM, VIB_ACC_GEAR,
                                    pulses, false, true);
    if (e1 != ZDT_OK) {
        return e1;
    }
    if (e2 != ZDT_OK) {
        return e2;
    }
    return zdt_x42_sync_start(s_m2);
}

/** 堵转保护检查（只查 clog，瞬时 stall 位在快速反转时不可靠） */
static bool any_clog(void)
{
    zdt_x42_status_t st = {0};
    if (s_id == 1) {
        return zdt_x42_read_status(s_m1, &st) == ZDT_OK && st.clog;
    }
    zdt_x42_status_t st2 = {0}, st3 = {0};
    esp_err_t e2 = zdt_x42_read_status(s_m2, &st2);
    esp_err_t e3 = zdt_x42_read_status(s_m3, &st3);
    if (e2 != ZDT_OK || e3 != ZDT_OK) {
        return false;
    }
    return st2.clog || st3.clog;
}

/** 等待参与电机全部到位（5ms 轮询，堵转保护立即返回错误） */
static esp_err_t wait_inpos(uint32_t timeout_ms)
{
    int64_t deadline = (int64_t)esp_timer_get_time() / 1000 + timeout_ms;
    while ((int64_t)esp_timer_get_time() / 1000 < deadline) {
        if (any_clog()) {
            return ZDT_ERR_COND;
        }
        zdt_x42_status_t st = {0};
        if (s_id == 1) {
            if (zdt_x42_read_status(s_m1, &st) == ZDT_OK && st.inpos) {
                return ZDT_OK;
            }
        } else {
            zdt_x42_status_t st2 = {0}, st3 = {0};
            if (zdt_x42_read_status(s_m2, &st2) == ZDT_OK &&
                zdt_x42_read_status(s_m3, &st3) == ZDT_OK &&
                st2.inpos && st3.inpos) {
                return ZDT_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ZDT_ERR_TIMEOUT;
}

/** 急停参与电机 */
static void stop_motors(void)
{
    if (s_id == 1) {
        zdt_x42_stop(s_m1, false);
    } else {
        zdt_x42_stop(s_m2, false);
        zdt_x42_stop(s_m3, false);
    }
}

static void vib_task(void *arg)
{
    (void)arg;
    const int id = s_id;
    const uint32_t a_pul = s_a_pul;
    const uint32_t n_half = s_n_half;
    const uint32_t half_ms = s_half_ms;

    stall_guard_bypass(true);   /* 振动期间挂起守护 */
    s_state = VIB_RUNNING;
    int64_t t_start = esp_timer_get_time();

    TickType_t period = pdMS_TO_TICKS(half_ms);
    TickType_t wake = xTaskGetTickCount() + period;

    bool cw = true;
    uint32_t done = 0;
    bool failed = false;

    while (!s_abort) {
        /* 半周期起点：+A, -2A, +2A, ... */
        uint32_t pul = (done == 0) ? a_pul : (2 * a_pul);
        if (issue_half(cw, pul) != ZDT_OK) {
            failed = true;
            break;
        }
        done++;
        s_cycles = done;
        s_elapsed_ms = (uint32_t)((esp_timer_get_time() - t_start) / 1000);
        cw = !cw;

        if (any_clog()) {
            failed = true;
            break;
        }

        if (s_stop_req) {
            wait_inpos(3000);   /* 优雅停：等当前半周期到位 */
            break;
        }
        if (n_half && done >= n_half) {
            break;
        }

        vTaskDelayUntil(&wake, period);
    }

    /* 回起振中心再停：偶数半周期后位于 -A 侧，回 +A；奇数侧回 -A */
    if (!failed && !s_abort) {
        bool back_cw = (done % 2 == 0);
        if (issue_half(back_cw, a_pul) == ZDT_OK) {
            wait_inpos(5000);
        }
    }

    if (s_abort) {
        s_state = VIB_STOPPED;      /* STP 0 急停，电机已由上位机停止 */
    } else if (failed) {
        stop_motors();
        s_state = VIB_FAILED;
    } else if (s_stop_req) {
        s_state = VIB_STOPPED;
    } else {
        s_state = VIB_DONE;
    }
    stall_guard_bypass(false);
    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t vib_start(int id, int freq_dhz, int amp_tenth_mm, int dur_s, bool mirror)
{
    if (id != 1 && id != 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (freq_dhz < 1 || freq_dhz > 100) {   /* 0.1 ~ 10.0 Hz */
        return ESP_ERR_INVALID_ARG;
    }
    if (amp_tenth_mm < 1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dur_s < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (vib_state() == VIB_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }

    s_id = id;
    s_a_pul = (uint32_t)((amp_tenth_mm * VIB_PULSES_PER_MM) / 10);
    if (s_a_pul == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_mirror = mirror ? 1 : 0;
    s_half_ms = 5000u / (uint32_t)freq_dhz;   /* T/2 = 1/(2f) */
    if (s_half_ms < 1) {
        s_half_ms = 1;
    }
    if (dur_s == 0) {
        s_n_half = 0;                         /* 持续到手动停 */
    } else {
        uint32_t n = (uint32_t)((dur_s * freq_dhz + 5) / 10) * 2;
        if (n < 2) {
            n = 2;
        }
        if (n % 2) {
            n++;                              /* 保证偶数：结束时回到中心 */
        }
        s_n_half = n;
    }

    s_cycles = 0;
    s_elapsed_ms = 0;
    s_stop_req = false;
    s_abort = false;
    s_state = VIB_IDLE;

    if (s_task) {
        vTaskDelete(s_task);                  /* 清理上一轮已结束任务的句柄 */
        s_task = NULL;
    }
    return xTaskCreate(vib_task, "vib", 4096, NULL, 6, &s_task) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

void vib_stop(void)
{
    if (vib_state() == VIB_RUNNING) {
        s_stop_req = true;
    }
}

void vib_abort(void)
{
    if (vib_state() == VIB_RUNNING) {
        s_abort = true;
    }
}
