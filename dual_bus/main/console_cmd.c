/**
 * @file console_cmd.c
 * @brief 上位机 ASCII 命令行解析与执行。
 */
#include "console_cmd.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_bus.h"
#include "zdt_x42.h"

#define MAX_TOKENS 12
#define TOKEN_LEN  24

/* ================= 输出助手 ================= */

static const char *err_name(esp_err_t e)
{
    switch (e) {
    case ZDT_OK:          return "OK";
    case ZDT_ERR_TIMEOUT: return "TIMEOUT";
    case ZDT_ERR_COND:    return "COND";
    case ZDT_ERR_REJECT:  return "REJECT";
    case ZDT_ERR_FRAME:   return "FRAME";
    case ZDT_ERR_TX:      return "TX";
    case ZDT_ERR_NULL:    return "BADID";
    default:              return "UNKNOWN";
    }
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

static const char *bus_letter(uart_port_t bus)
{
    return (bus == BUS_B_UART) ? "B" : "A";
}

/** 打印一台已登记电机（MOTORS / SCAN 共用格式） */
static void print_motor(const motor_info_t *m)
{
    printf("CMD> FOUND %d %s %u %u %u\n",
           motor_id_of(m), bus_letter(m->bus), m->addr, m->fw, m->hw);
}

/* ================= 命令实现 ================= */

static void do_scan(int from, int to)
{
    int n = motor_scan(from, to);
    for (size_t i = 0; i < motor_count(); i++) {
        print_motor(motor_at(i));
    }
    printf("CMD> SCAN %d\n", n);
}

static void do_help(void)
{
    printf("CMD> HELP PING | SCAN [from] [to] | MOTORS | ENA <id> <0|1>\n");
    printf("CMD> HELP POS <id> <CW|CCW> <pulses> <rpm> <accel> <ABS|REL> [SYNC]\n");
    printf("CMD> HELP GO | SPD <id> <CW|CCW> <rpm> <accel> | STP <id|0>\n");
    printf("CMD> HELP ZERO <id> | REL <id> | STAT <id|0> | IO? | IO OUT <0|1>\n");
}

/** 解析并执行一行命令 */
static void handle_line(char *line)
{
    char tok[MAX_TOKENS][TOKEN_LEN] = {{0}};
    int n = 0;

    char *t = strtok(line, " \r\n\t");
    while (t && n < MAX_TOKENS) {
        snprintf(tok[n++], TOKEN_LEN, "%s", t);
        t = strtok(NULL, " \r\n\t");
    }

    if (n == 0) {
        printf("CMD> DONE\n");      /* 空行：串口工具常发多余换行，直接确认 */
        return;
    }

    if (strcmp(tok[0], "PING") == 0) {
        printf("CMD> PONG\n");

    } else if (strcmp(tok[0], "SCAN") == 0) {
        int from = (n >= 2) ? atoi(tok[1]) : SCAN_ADDR_FROM;
        int to   = (n >= 3) ? atoi(tok[2]) : SCAN_ADDR_TO;
        do_scan(from, to);

    } else if (strcmp(tok[0], "MOTORS") == 0) {
        for (size_t i = 0; i < motor_count(); i++) {
            print_motor(motor_at(i));
        }
        printf("CMD> MOTORS %d\n", (int)motor_count());

    } else if (strcmp(tok[0], "ENA") == 0 && n >= 3) {
        zdt_x42_t *m = motor_find(atoi(tok[1]));
        int on = atoi(tok[2]) != 0;
        if (!m) {
            cmd_err("ENA BADID %s", tok[1]);
        } else {
            esp_err_t e = zdt_x42_enable(m, on, false);
            if (e == ZDT_OK) cmd_ok("ENA %s %d", tok[1], on);
            else cmd_err("ENA %s %s", tok[1], err_name(e));
        }

    } else if (strcmp(tok[0], "POS") == 0 && n >= 7) {
        zdt_x42_t *m = motor_find(atoi(tok[1]));
        if (!m) {
            cmd_err("POS BADID %s", tok[1]);
        } else {
            bool cw   = (strcmp(tok[2], "CW") == 0);
            uint32_t pulses = (uint32_t)strtoul(tok[3], NULL, 0);
            uint16_t rpm    = (uint16_t)atoi(tok[4]);
            uint8_t  accel  = (uint8_t)atoi(tok[5]);
            bool abs_mode   = (strcmp(tok[6], "ABS") == 0);
            bool sync       = (n >= 8 && strncmp(tok[7], "SYNC", 4) == 0);
            esp_err_t e = zdt_x42_pos_mode(m, cw, rpm, accel, pulses, abs_mode, sync);
            if (e != ZDT_OK) {
                cmd_err("POS %s %s", tok[1], err_name(e));
            } else if (sync) {
                cmd_ok("POS %s SYNC", tok[1]);
            } else {
                cmd_ok("POS %s", tok[1]);
            }
        }

    } else if (strcmp(tok[0], "GO") == 0) {
        if (motor_count() == 0) {
            cmd_err("GO NO-MOTOR");
        } else {
            esp_err_t e = motor_sync_start_all();
            if (e == ZDT_OK) cmd_ok("GO");
            else cmd_err("GO %s", err_name(e));
        }

    } else if (strcmp(tok[0], "SPD") == 0 && n >= 5) {
        zdt_x42_t *m = motor_find(atoi(tok[1]));
        if (!m) {
            cmd_err("SPD BADID %s", tok[1]);
        } else {
            bool cw = (strcmp(tok[2], "CW") == 0);
            uint16_t rpm   = (uint16_t)atoi(tok[3]);
            uint8_t  accel = (uint8_t)atoi(tok[4]);
            esp_err_t e = zdt_x42_speed_mode(m, cw, rpm, accel, false);
            if (e == ZDT_OK) cmd_ok("SPD %s", tok[1]);
            else cmd_err("SPD %s %s", tok[1], err_name(e));
        }

    } else if (strcmp(tok[0], "STP") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        if (id == 0) {
            esp_err_t last = ZDT_OK;
            for (size_t i = 0; i < motor_count(); i++) {
                last = zdt_x42_stop(motor_find(motor_id_of(motor_at(i))), false);
            }
            if (motor_count() == 0) cmd_err("STP NO-MOTOR");
            else if (last == ZDT_OK) cmd_ok("STP 0");
            else cmd_err("STP 0 %s", err_name(last));
        } else {
            zdt_x42_t *m = motor_find(id);
            if (!m) {
                cmd_err("STP BADID %d", id);
            } else {
                esp_err_t e = zdt_x42_stop(m, false);
                if (e == ZDT_OK) cmd_ok("STP %d", id);
                else cmd_err("STP %d %s", id, err_name(e));
            }
        }

    } else if (strcmp(tok[0], "ZERO") == 0 && n >= 2) {
        zdt_x42_t *m = motor_find(atoi(tok[1]));
        if (!m) {
            cmd_err("ZERO BADID %s", tok[1]);
        } else {
            esp_err_t e = zdt_x42_clear_position(m);
            if (e == ZDT_OK) cmd_ok("ZERO %s", tok[1]);
            else cmd_err("ZERO %s %s", tok[1], err_name(e));
        }

    } else if (strcmp(tok[0], "REL") == 0 && n >= 2) {
        zdt_x42_t *m = motor_find(atoi(tok[1]));
        if (!m) {
            cmd_err("REL BADID %s", tok[1]);
        } else {
            esp_err_t e = zdt_x42_release_stall(m);
            if (e == ZDT_OK) cmd_ok("REL %s", tok[1]);
            else cmd_err("REL %s %s", tok[1], err_name(e));
        }

    } else if (strcmp(tok[0], "STAT") == 0 && n >= 2) {
        int id = atoi(tok[1]);
        if (id == 0) {
            for (size_t i = 0; i < motor_count(); i++) {
                const motor_info_t *info = motor_at(i);
                zdt_x42_t *m = motor_find(motor_id_of(info));
                zdt_x42_status_t st = {0};
                int32_t pos = 0, spd = 0;
                esp_err_t e1 = zdt_x42_read_status(m, &st);
                esp_err_t e2 = zdt_x42_read_position(m, &pos);
                esp_err_t e3 = zdt_x42_read_speed(m, &spd);
                if (e1 != ZDT_OK || e2 != ZDT_OK || e3 != ZDT_OK) {
                    cmd_err("STAT %d %s", motor_id_of(info),
                            err_name(e1 != ZDT_OK ? e1 : (e2 != ZDT_OK ? e2 : e3)));
                    continue;
                }
                printf("CMD> STAT %d %d %d %d %d %ld %ld\n", motor_id_of(info),
                       st.enable, st.inpos, st.stall, st.clog, (long)pos, (long)spd);
            }
        } else {
            zdt_x42_t *m = motor_find(id);
            if (!m) {
                cmd_err("STAT BADID %d", id);
            } else {
                zdt_x42_status_t st = {0};
                int32_t pos = 0, spd = 0;
                esp_err_t e1 = zdt_x42_read_status(m, &st);
                esp_err_t e2 = zdt_x42_read_position(m, &pos);
                esp_err_t e3 = zdt_x42_read_speed(m, &spd);
                if (e1 != ZDT_OK || e2 != ZDT_OK || e3 != ZDT_OK) {
                    cmd_err("STAT %d %s", id,
                            err_name(e1 != ZDT_OK ? e1 : (e2 != ZDT_OK ? e2 : e3)));
                } else {
                    printf("CMD> STAT %d %d %d %d %d %ld %ld\n", id,
                           st.enable, st.inpos, st.stall, st.clog, (long)pos, (long)spd);
                }
            }
        }

    } else if (strcmp(tok[0], "IO") == 0 && n >= 2) {
        if (strcmp(tok[1], "OUT") == 0 && n >= 3) {
            int v = atoi(tok[2]) ? 1 : 0;
            io_mos_set(v);
            cmd_ok("IO OUT %d %d", PIN_MOS, io_mos_level());
        } else if (strcmp(tok[1], "?") == 0) {
            printf("CMD> IO IN %d %d OUT %d %d\n",
                   PIN_LIMIT_SW, io_limit_level(), PIN_MOS, io_mos_level());
        } else {
            cmd_err("IO ARG %s", tok[1]);
        }

    } else if (strcmp(tok[0], "IO?") == 0) {
        printf("CMD> IO IN %d %d OUT %d %d\n",
               PIN_LIMIT_SW, io_limit_level(), PIN_MOS, io_mos_level());

    } else if (strcmp(tok[0], "HELP") == 0) {
        do_help();

    } else {
        cmd_err("CMD %s", tok[0]);
    }

    printf("CMD> DONE\n");
}

/* ================= 任务 ================= */

static void cmd_task(void *arg)
{
    char line[128];

    printf("CMD> READY\n");

    /* 上电自动扫一遍，上位机连上就能拿到电机列表 */
    do_scan(SCAN_ADDR_FROM, SCAN_ADDR_TO);
    printf("CMD> DONE\n");

    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        handle_line(line);
    }
}

esp_err_t console_cmd_start(void)
{
    xTaskCreate(cmd_task, "cmd", 8192, NULL, 10, NULL);
    return ESP_OK;
}
