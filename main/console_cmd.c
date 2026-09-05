/**
 * @file console_cmd.c
 * @brief 上位机 ASCII 命令行协议实现。
 *
 * 协议：一行一条命令，参数空格分隔，回车结束（大小写敏感，统一大写）。
 * ESP 回复行均以 "CMD> " 开头，每条命令处理完回复 "CMD> DONE"。
 *
 *   PC -> ESP:
 *     VER                     读所有电机版本
 *     ENA <id> <0|1>          使能/去使能
 *     POS <id> <CW|CCW> <pulses> <speed_rpm> <accel> <ABS|REL> [SYNC]
 *                              位置模式（带 SYNC = 押住等待广播）
 *     GO                      广播同步启动：所有押住的电机同时开始运动
 *     PHOME                   电机2/3双机回零状态机：同时碰撞找端->退一圈->设零点
 *     PSTATE                  查双机回零状态（0空闲1回零中2退圈中3设零点4完成5失败）
 *     XHOME                   电机1(X轴)回零：碰撞找端->退一圈->设零点
 *     XSTATE                  查X轴回零状态（同 PSTATE 编码）
 *     SPD <id> <CW|CCW> <speed_rpm> <accel>                      速度模式
 *     STP <id|0>              立即停止（id=0 全部）
 *     ZERO <id>               当前位置清零（掉电不保存）
 *     OZERO <id>              存单圈零点为当前位置（掉电保存）
 *     HOME <id> <mode> [SYNC]  触发回零 0-3（带 SYNC = 押住等广播）
 *     HPARAM <id> <mode> <CW|CCW> <speed> <timeout_ms> <clog_rpm> <clog_ma> <clog_ms> <auto> [STORE]
 *                              写回零参数（默认存芯片）
 *     HPARAMR <id>             读回零参数
 *     HSTAT <id>               读回零状态（bit2正在回零 bit3回零失败）
 *     REL <id>                解除堵转保护
 *     GUARD <0|1>             启用/禁用堵转自动恢复守护（用户 UI 应发 GUARD 0）
 *     VIB <id> <freq_dHz> <amp_0.1mm> <dur_s> <mirror>
 *                              振动（定时驱动）：id=1 X轴 / id=2 Y轴双机，半幅±amp，
 *                              dur=0 持续到 VIBSTP；偶数半周期后回中心停
 *     VIBSTP                  优雅停止振动（当前半周期到位后回中心）
 *     VSTATE                  查振动状态
 *     STAT <id|0>             查状态（id=0 全部）
 *
 *   ESP -> PC:
 *     CMD> OK <说明>
 *     CMD> ERR <说明>
 *     CMD> STAT <id> <en> <inpos> <stall> <clog> <pos_raw> <speed_rpm>
 *     CMD> DONE
 */
#include "console_cmd.h"
#include "pair_home.h"
#include "stall_guard.h"
#include "vib.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX_TOKENS 12
#define TOKEN_LEN   24

static zdt_x42_t *s_motors;
static size_t      s_count;

static const char *err_name(esp_err_t e)
{
    switch (e) {
    case ZDT_OK:           return "OK";
    case ZDT_ERR_TIMEOUT:  return "TIMEOUT";
    case ZDT_ERR_COND:     return "COND";
    case ZDT_ERR_REJECT:   return "REJECT";
    case ZDT_ERR_FRAME:    return "FRAME";
    case ZDT_ERR_TX:       return "TX";
    case ZDT_ERR_NULL:     return "BADID";
    default:               return "UNKNOWN";
    }
}

static zdt_x42_t *motor_by_id(int id)
{
    if (id < 1 || id > (int)s_count) {
        return NULL;
    }
    return &s_motors[id - 1];
}

static void cmd_ok(const char *fmt, ...)
{
    va_list ap;
    printf("CMD> OK ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

static void cmd_err(const char *fmt, ...)
{
    va_list ap;
    printf("CMD> ERR ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

static void cmd_done(void)
{
    printf("CMD> DONE\n");
}

static void cmd_stat(int id)
{
    int first = 1, last = (int)s_count;
    if (id > 0) {
        first = last = id;
    }
    for (int i = first; i <= last; i++) {
        zdt_x42_t *m = motor_by_id(i);
        zdt_x42_status_t st = {0};
        int32_t pos = 0, spd = 0;
        esp_err_t e1 = zdt_x42_read_status(m, &st);
        esp_err_t e2 = zdt_x42_read_position(m, &pos);
        esp_err_t e3 = zdt_x42_read_speed(m, &spd);
        if (e1 != ZDT_OK || e2 != ZDT_OK || e3 != ZDT_OK) {
            cmd_err("STAT %d %s", i, err_name(e1 != ZDT_OK ? e1 : (e2 != ZDT_OK ? e2 : e3)));
            continue;
        }
        printf("CMD> STAT %d %d %d %d %d %ld %ld\n",
               i, st.enable, st.inpos, st.stall, st.clog, (long)pos, (long)spd);
    }
}

static void handle_line(char *line)
{
    char tok[MAX_TOKENS][TOKEN_LEN] = {0};
    int n = 0;
    char *t = strtok(line, " \r\n\t");
    while (t && n < MAX_TOKENS) {
        snprintf(tok[n++], TOKEN_LEN, "%s", t);
        t = strtok(NULL, " \r\n\t");
    }
    if (n == 0) {
        cmd_done();
        return;
    }

    if (strcmp(tok[0], "VER") == 0) {
        for (int i = 1; i <= (int)s_count; i++) {
            uint16_t fw = 0, hw = 0;
            esp_err_t e = zdt_x42_read_version(motor_by_id(i), &fw, &hw);
            if (e == ZDT_OK) {
                cmd_ok("VER %d %u %u", i, fw, hw);
            } else {
                cmd_err("VER %d %s", i, err_name(e));
            }
        }
    } else if (strcmp(tok[0], "ENA") == 0 && n >= 3) {
        int id = atoi(tok[1]);
        zdt_x42_t *m = motor_by_id(id);
        esp_err_t e = m ? zdt_x42_enable(m, atoi(tok[2]) != 0, false) : ZDT_ERR_NULL;
        if (e == ZDT_OK) cmd_ok("ENA %d", id); else cmd_err("ENA %d %s", id, err_name(e));
    } else if (strcmp(tok[0], "POS") == 0 && n >= 7) {
        int id = atoi(tok[1]);
        bool cw = (strcmp(tok[2], "CW") == 0);
        uint32_t pulses = (uint32_t)strtoul(tok[3], NULL, 0);
        uint16_t speed = (uint16_t)atoi(tok[4]);
        uint8_t accel = (uint8_t)atoi(tok[5]);
        bool absolute = (strcmp(tok[6], "ABS") == 0);
        bool sync = (n >= 8 && strncmp(tok[7], "SYNC", 4) == 0);
        zdt_x42_t *m = motor_by_id(id);
        esp_err_t e = m ? zdt_x42_pos_mode(m, cw, speed, accel, pulses, absolute, sync)
                        : ZDT_ERR_NULL;
        if (e == ZDT_OK) {
            stall_guard_set_dir((size_t)(id - 1), cw);   /* 记录方向供自动恢复用 */
            cmd_ok("POS %d", id);
        } else {
            cmd_err("POS %d %s", id, err_name(e));
        }
    } else if (strcmp(tok[0], "GO") == 0) {
        /* 广播同步启动：所有押住（sync=1）的电机同时开始运动 */
        esp_err_t e = zdt_x42_sync_start(&s_motors[0]);
        if (e == ZDT_OK) cmd_ok("GO"); else cmd_err("GO %s", err_name(e));
    } else if (strcmp(tok[0], "PHOME") == 0) {
        esp_err_t e = pair_home_start();
        if (e == ESP_OK) cmd_ok("PHOME");
        else cmd_err("PHOME %s", e == ESP_ERR_INVALID_STATE ? "BUSY" : err_name(e));
    } else if (strcmp(tok[0], "PSTATE") == 0) {
        uint8_t f2 = 0, f3 = 0;
        pair_home_flags(&f2, &f3);
        printf("CMD> PSTATE %d %d %d\n", (int)pair_home_state(), f2, f3);
    } else if (strcmp(tok[0], "XHOME") == 0) {
        esp_err_t e = xhome_start();
        if (e == ESP_OK) cmd_ok("XHOME");
        else cmd_err("XHOME %s", e == ESP_ERR_INVALID_STATE ? "BUSY" : err_name(e));
    } else if (strcmp(tok[0], "XSTATE") == 0) {
        uint8_t f1 = 0;
        xhome_flags(&f1);
        printf("CMD> XSTATE %d %d\n", (int)xhome_state(), f1);
    } else if (strcmp(tok[0], "SPD") == 0 && n >= 5) {
        int id = atoi(tok[1]);
        bool cw = (strcmp(tok[2], "CW") == 0);
        uint16_t speed = (uint16_t)atoi(tok[3]);
        uint8_t accel = (uint8_t)atoi(tok[4]);
        zdt_x42_t *m = motor_by_id(id);
        esp_err_t e = m ? zdt_x42_speed_mode(m, cw, speed, accel, false) : ZDT_ERR_NULL;
        if (e == ZDT_OK) {
            stall_guard_set_dir((size_t)(id - 1), cw);
            cmd_ok("SPD %d", id);
        } else {
            cmd_err("SPD %d %s", id, err_name(e));
        }
    } else if (strcmp(tok[0], "STP") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        esp_err_t e = ZDT_OK;
        if (id == 0) {
            vib_abort();   /* 急停联动：终止振动任务（电机随即被停） */
            for (int i = 1; i <= (int)s_count; i++) {
                e = zdt_x42_stop(motor_by_id(i), false);
            }
            if (e == ZDT_OK) cmd_ok("STP 0"); else cmd_err("STP 0 %s", err_name(e));
        } else {
            zdt_x42_t *m = motor_by_id(id);
            e = m ? zdt_x42_stop(m, false) : ZDT_ERR_NULL;
            if (e == ZDT_OK) cmd_ok("STP %d", id); else cmd_err("STP %d %s", id, err_name(e));
        }
    } else if (strcmp(tok[0], "ZERO") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        zdt_x42_t *m = motor_by_id(id);
        esp_err_t e = m ? zdt_x42_clear_position(m) : ZDT_ERR_NULL;
        if (e == ZDT_OK) cmd_ok("ZERO %d", id); else cmd_err("ZERO %d %s", id, err_name(e));
    } else if (strcmp(tok[0], "OZERO") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        zdt_x42_t *m = motor_by_id(id);
        esp_err_t e = m ? zdt_x42_set_single_zero(m, true) : ZDT_ERR_NULL;
        if (e == ZDT_OK) cmd_ok("OZERO %d", id); else cmd_err("OZERO %d %s", id, err_name(e));
    } else if (strcmp(tok[0], "HOME") == 0 && n >= 3) {
        int id = atoi(tok[1]);
        if (id == 1) {
            /* 电机1统一走 X 轴状态机：碰撞找端 -> 退一圈 -> 设零点 */
            esp_err_t e = xhome_start();
            if (e == ESP_OK) cmd_ok("HOME 1"); else cmd_err("HOME 1 %s", e == ESP_ERR_INVALID_STATE ? "BUSY" : err_name(e));
        } else {
            bool sync = (n >= 4 && strncmp(tok[3], "SYNC", 4) == 0);
            zdt_x42_t *m = motor_by_id(id);
            esp_err_t e = m ? zdt_x42_home(m, (uint8_t)atoi(tok[2]), sync) : ZDT_ERR_NULL;
            if (e == ZDT_OK) cmd_ok("HOME %d", id); else cmd_err("HOME %d %s", id, err_name(e));
        }
    } else if (strcmp(tok[0], "HPARAM") == 0 && n >= 10) {
        int id = atoi(tok[1]);
        zdt_x42_home_params_t p = {
            .mode = (uint8_t)atoi(tok[2]),
            .dir_cw = (strcmp(tok[3], "CW") == 0),
            .speed_rpm = (uint16_t)atoi(tok[4]),
            .timeout_ms = (uint32_t)strtoul(tok[5], NULL, 0),
            .clog_rpm = (uint16_t)atoi(tok[6]),
            .clog_ma = (uint16_t)atoi(tok[7]),
            .clog_ms = (uint16_t)atoi(tok[8]),
            .auto_home = (atoi(tok[9]) != 0),
        };
        bool store = true;
        if (n >= 11 && strncmp(tok[10], "NOSTORE", 7) == 0) {
            store = false;
        }
        zdt_x42_t *m = motor_by_id(id);
        esp_err_t e = m ? zdt_x42_set_home_params(m, &p, store) : ZDT_ERR_NULL;
        if (e == ZDT_OK) cmd_ok("HPARAM %d", id); else cmd_err("HPARAM %d %s", id, err_name(e));
    } else if (strcmp(tok[0], "HPARAMR") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        zdt_x42_t *m = motor_by_id(id);
        zdt_x42_home_params_t p = {0};
        esp_err_t e = m ? zdt_x42_read_home_params(m, &p) : ZDT_ERR_NULL;
        if (e == ZDT_OK) {
            printf("CMD> HPARAM %d %d %s %u %lu %u %u %u %d\n", id, p.mode,
                   p.dir_cw ? "CW" : "CCW", p.speed_rpm, (unsigned long)p.timeout_ms,
                   p.clog_rpm, p.clog_ma, p.clog_ms, p.auto_home);
        } else {
            cmd_err("HPARAMR %d %s", id, err_name(e));
        }
    } else if (strcmp(tok[0], "HSTAT") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        zdt_x42_t *m = motor_by_id(id);
        uint8_t flags = 0;
        esp_err_t e = m ? zdt_x42_read_home_status(m, &flags) : ZDT_ERR_NULL;
        if (e == ZDT_OK) {
            printf("CMD> HSTAT %d %d\n", id, flags);
        } else {
            cmd_err("HSTAT %d %s", id, err_name(e));
        }
    } else if (strcmp(tok[0], "REL") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        zdt_x42_t *m = motor_by_id(id);
        esp_err_t e = m ? zdt_x42_release_stall(m) : ZDT_ERR_NULL;
        if (e == ZDT_OK) cmd_ok("REL %d", id); else cmd_err("REL %d %s", id, err_name(e));
    } else if (strcmp(tok[0], "GUARD") == 0 && n >= 2) {
        bool on = (atoi(tok[1]) != 0);
        stall_guard_set_enabled(on);
        cmd_ok("GUARD %d", on ? 1 : 0);
    } else if (strcmp(tok[0], "VIB") == 0 && n >= 6) {
        int id = atoi(tok[1]);
        int f = atoi(tok[2]);
        int a = atoi(tok[3]);
        int dur = atoi(tok[4]);
        bool mirror = (atoi(tok[5]) != 0);
        esp_err_t e = vib_start(id, f, a, dur, mirror);
        if (e == ESP_OK) {
            cmd_ok("VIB %d f=%d.%dHz A=%d.%dmm dur=%ds",
                   id, f / 10, f % 10, a / 10, a % 10, dur);
        } else if (e == ESP_ERR_INVALID_ARG) {
            cmd_err("VIB ARGS");
        } else if (e == ESP_ERR_INVALID_STATE) {
            cmd_err("VIB BUSY");
        } else {
            cmd_err("VIB %s", err_name(e));
        }
    } else if (strcmp(tok[0], "VIBSTP") == 0) {
        vib_stop();
        cmd_ok("VIBSTP");
    } else if (strcmp(tok[0], "VSTATE") == 0) {
        uint32_t c = 0, ms = 0;
        vib_fault_phase_t phase = VIB_FAULT_NONE;
        int fault_code = 0;
        vib_stats(&c, &ms);
        vib_fault_info(&phase, &fault_code);
        printf("CMD> VSTATE %d %lu %lu %s %d\n", (int)vib_state(),
               (unsigned long)c, (unsigned long)ms,
               vib_fault_phase_name(phase), fault_code);
    } else if (strcmp(tok[0], "STAT") == 0 && n >= 2) {
        cmd_stat(atoi(tok[1]));
    } else {
        cmd_err("CMD %s", tok[0]);
    }
    cmd_done();
}

static void cmd_task(void *arg)
{
    char line[128];
    printf("CMD> READY\n");
    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        handle_line(line);
    }
}

esp_err_t console_cmd_init(zdt_x42_t *motors, size_t count)
{
    s_motors = motors;
    s_count = count;
    xTaskCreate(cmd_task, "x42_cmd", 4096, NULL, 10, NULL);
    return ESP_OK;
}
