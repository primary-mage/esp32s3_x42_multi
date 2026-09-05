/**
 * @file pair_home.c
 * @brief 电机2/3 双机回零状态机实现。
 *
 * 设计要点：
 *  - 两机用 SYNC 押住 + 广播同时起步，各自按自己的回零参数碰撞找端；
 *  - 每台撞到自己的弹性限位后回零判定完成，先到的停下，另一台继续，直到两台都完成；
 *  - 两机都触发后：如有堵转保护则解除 -> 反向各退一圈（退出弹性压紧区）-> 设零点；
 *  - 全程堵转守护挂起，防止单边自动恢复扭龙门。
 */
#include "pair_home.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stall_guard.h"

#define PH_POLL_MS         200    /* 回零等待轮询周期 */
#define PH_BACKOFF_PULSES  3200   /* 反向退一圈（16细分；改细分需同步改） */
#define PH_BACKOFF_SPEED   100    /* 退圈转速 RPM */
#define PH_BACKOFF_ACCEL   5
#define PH_BACKOFF_TIMEOUT 10000  /* 单机退圈到位等待 */
#define PH_MARGIN_MS       10000  /* 回零总超时 = 回零参数超时 + 裕量 */

static const char *TAG = "pair_home";

static zdt_x42_t *s_m2, *s_m3;
static TaskHandle_t s_task;
static volatile pair_home_state_t s_state = PAIR_HOME_IDLE;
static volatile uint8_t s_f2, s_f3;

pair_home_state_t pair_home_state(void)
{
    return s_state;
}

void pair_home_flags(uint8_t *f2, uint8_t *f3)
{
    *f2 = s_f2;
    *f3 = s_f3;
}

/**
 * 等待"真的动起来再停下来"：先等位置开始变化（避免把上一动作的到位标志
 * 误当成本次完成），再等到位标志置位。
 */
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
                    started = true;   /* 位置开始变化，运动真的开始了 */
                }
            } else if (st.inpos) {
                return true;          /* 运动完成 */
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

/** 反向退一圈并清零点（内部会先解除堵转保护、重新使能） */
static bool backoff_and_zero(zdt_x42_t *m, int idx, bool dir_cw)
{
    zdt_x42_status_t st = {0};
    if (zdt_x42_read_status(m, &st) == ZDT_OK && st.clog) {
        ESP_LOGW(TAG, "[电机%d] 存在堵转保护，解除...", idx);
        zdt_x42_release_stall(m);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    zdt_x42_enable(m, true, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    int32_t pos0 = 0;
    zdt_x42_read_position(m, &pos0);

    for (int attempt = 0; attempt < 2; attempt++) {
        esp_err_t e = zdt_x42_pos_mode(m, !dir_cw, PH_BACKOFF_SPEED, PH_BACKOFF_ACCEL,
                                       PH_BACKOFF_PULSES, false, false);
        if (e != ZDT_OK) {
            ESP_LOGW(TAG, "[电机%d] 退圈命令失败: %d", idx, (int)e);
            return false;
        }
        /* 必须先看到位置开始变化，再等到位（防止旧到位标志误判完成） */
        if (wait_move_done(m, pos0, PH_BACKOFF_TIMEOUT)) {
            zdt_x42_clear_position(m);
            return true;
        }
        ESP_LOGW(TAG, "[电机%d] 退圈未动，重试一次", idx);
        vTaskDelay(pdMS_TO_TICKS(300));
        zdt_x42_read_position(m, &pos0);
    }
    ESP_LOGW(TAG, "[电机%d] 退圈未到位", idx);
    return false;
}

static void run_pair_home(void)
{
    stall_guard_bypass(true);   /* 双机回零期间挂起守护 */

    s_state = PAIR_HOME_HOMING;
    ESP_LOGI(TAG, "双机回零开始：使能 + 押住回零命令 + 广播同时起步");

    /* 读电机2的回零参数（方向/超时；两机参数假设已配一致） */
    zdt_x42_home_params_t p = {0};
    bool have_params = (zdt_x42_read_home_params(s_m2, &p) == ZDT_OK);
    if (!have_params) {
        p.mode = 2;           /* 默认多圈碰撞 */
        p.dir_cw = true;
        p.timeout_ms = 30000;
        ESP_LOGW(TAG, "读回零参数失败，用默认值 mode=2 CW 30s");
    }

    zdt_x42_enable(s_m2, true, false);
    zdt_x42_enable(s_m3, true, false);

    /* 押住两机回零命令，广播同时起步 */
    zdt_x42_home(s_m2, p.mode, true);
    zdt_x42_home(s_m3, p.mode, true);
    zdt_x42_sync_start(s_m2);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 等待两机各自碰撞完成：先到的停住，后到的继续，直到两台都完成 */
    int64_t deadline = (int64_t)esp_timer_get_time() / 1000 +
                       (int64_t)p.timeout_ms + PH_MARGIN_MS;
    bool ok = false;
    while ((int64_t)esp_timer_get_time() / 1000 < deadline) {
        uint8_t f2 = 0, f3 = 0;
        bool got2 = (zdt_x42_read_home_status(s_m2, &f2) == ZDT_OK);
        bool got3 = (zdt_x42_read_home_status(s_m3, &f3) == ZDT_OK);
        if (got2) s_f2 = f2;
        if (got3) s_f3 = f3;

        if (got2 && got3 && !(f2 & 0x04) && !(f3 & 0x04) &&
            !(f2 & 0x08) && !(f3 & 0x08)) {
            ok = true;   /* 两机都回零完成且无失败 */
            break;
        }
        if (got2 && got3 && ((f2 & 0x08) || (f3 & 0x08))) {
            ESP_LOGE(TAG, "回零失败标志: m2=0x%02X m3=0x%02X", f2, f3);
            break;       /* 有失败标志，走 FAILED */
        }
        vTaskDelay(pdMS_TO_TICKS(PH_POLL_MS));
    }

    if (!ok) {
        s_state = PAIR_HOME_FAILED;
        stall_guard_bypass(false);
        ESP_LOGE(TAG, "双机回零失败（超时或回零失败）");
        return;
    }

    ESP_LOGI(TAG, "两机均已碰撞到位，开始反向退圈（退出弹性压紧区）");
    vTaskDelay(pdMS_TO_TICKS(500));   /* 等电机完全结束回零状态再发运动命令 */
    s_state = PAIR_HOME_BACKOFF;
    bool ok2 = backoff_and_zero(s_m2, 2, p.dir_cw);
    bool ok3 = backoff_and_zero(s_m3, 3, p.dir_cw);
    if (!ok2 || !ok3) {
        s_state = PAIR_HOME_FAILED;
        stall_guard_bypass(false);
        ESP_LOGE(TAG, "退圈/设零点失败: m2=%d m3=%d", ok2, ok3);
        return;
    }

    s_state = PAIR_HOME_DONE;
    stall_guard_bypass(false);
    ESP_LOGI(TAG, "双机回零完成：两机反向退一圈，当前位置已设为零点");
}

static void pair_home_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        run_pair_home();
        s_state = (s_state == PAIR_HOME_FAILED) ? PAIR_HOME_FAILED : PAIR_HOME_DONE;
    }
}

esp_err_t pair_home_start(void)
{
    pair_home_state_t st = pair_home_state();
    if (st == PAIR_HOME_HOMING || st == PAIR_HOME_BACKOFF || st == PAIR_HOME_ZEROING) {
        return ESP_ERR_INVALID_STATE;   /* 正在运行 */
    }
    s_state = PAIR_HOME_IDLE;
    xTaskNotifyGive(s_task);
    return ESP_OK;
}

esp_err_t pair_home_init(zdt_x42_t *m2, zdt_x42_t *m3)
{
    s_m2 = m2;
    s_m3 = m3;
    s_state = PAIR_HOME_IDLE;
    xTaskCreate(pair_home_task, "pair_home", 4096, NULL, 5, &s_task);
    return ESP_OK;
}

/* ============ 电机1（X轴）单机回零 + 退圈设零点 ============ */

static zdt_x42_t *s_m1;
static TaskHandle_t s_x_task;
static volatile pair_home_state_t s_x_state = PAIR_HOME_IDLE;
static volatile uint8_t s_f1;

pair_home_state_t xhome_state(void)
{
    return s_x_state;
}

void xhome_flags(uint8_t *f1)
{
    *f1 = s_f1;
}

static void run_xhome(void)
{
    stall_guard_bypass(true);   /* 回零期间挂起守护 */

    s_x_state = PAIR_HOME_HOMING;
    ESP_LOGI(TAG, "X轴回零开始：碰撞找端");

    zdt_x42_home_params_t p = {0};
    bool have_params = (zdt_x42_read_home_params(s_m1, &p) == ZDT_OK);
    if (!have_params) {
        p.mode = 2;
        p.dir_cw = true;
        p.timeout_ms = 30000;
        ESP_LOGW(TAG, "读回零参数失败，用默认值 mode=2 CW 30s");
    }

    zdt_x42_enable(s_m1, true, false);
    zdt_x42_home(s_m1, p.mode, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    int64_t deadline = (int64_t)esp_timer_get_time() / 1000 +
                       (int64_t)p.timeout_ms + PH_MARGIN_MS;
    bool ok = false;
    while ((int64_t)esp_timer_get_time() / 1000 < deadline) {
        uint8_t f = 0;
        if (zdt_x42_read_home_status(s_m1, &f) == ZDT_OK) {
            s_f1 = f;
            if (!(f & 0x04) && !(f & 0x08)) {
                ok = true;
                break;
            }
            if (f & 0x08) {
                ESP_LOGE(TAG, "X轴回零失败标志: 0x%02X", f);
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(PH_POLL_MS));
    }

    if (!ok) {
        s_x_state = PAIR_HOME_FAILED;
        stall_guard_bypass(false);
        ESP_LOGE(TAG, "X轴回零失败（超时或回零失败）");
        return;
    }

    ESP_LOGI(TAG, "X轴已碰撞到位，反向退一圈并设零点");
    vTaskDelay(pdMS_TO_TICKS(500));   /* 等电机完全结束回零状态再发运动命令 */
    s_x_state = PAIR_HOME_BACKOFF;
    if (!backoff_and_zero(s_m1, 1, p.dir_cw)) {
        s_x_state = PAIR_HOME_FAILED;
        stall_guard_bypass(false);
        ESP_LOGE(TAG, "X轴退圈/设零点失败");
        return;
    }

    s_x_state = PAIR_HOME_DONE;
    stall_guard_bypass(false);
    ESP_LOGI(TAG, "X轴回零完成：反向退一圈，当前位置已设为零点");
}

static void xhome_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        run_xhome();
    }
}

esp_err_t xhome_start(void)
{
    pair_home_state_t st = xhome_state();
    if (st == PAIR_HOME_HOMING || st == PAIR_HOME_BACKOFF || st == PAIR_HOME_ZEROING) {
        return ESP_ERR_INVALID_STATE;
    }
    s_x_state = PAIR_HOME_IDLE;
    xTaskNotifyGive(s_x_task);
    return ESP_OK;
}

esp_err_t xhome_init(zdt_x42_t *m1)
{
    s_m1 = m1;
    s_x_state = PAIR_HOME_IDLE;
    xTaskCreate(xhome_task, "xhome", 4096, NULL, 5, &s_x_task);
    return ESP_OK;
}
