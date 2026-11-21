/**
 * @file motor_bus.c
 * @brief 双 UART 总线电机自动发现 + IO 控制。
 *
 * 扫描原理：电机的地址是 1~255，逐地址发一条「读版本」(01 1F 6B)，
 * 有电机的地址通常 1~3ms 内就应答，空地址则等到超时。
 * 因此把探测超时压到 SCAN_TIMEOUT_MS(25ms)，扫 16 个地址 × 2 条总线约 0.8 秒，
 * 比用默认 200ms 超时（要扫一分多钟）实用得多。
 */
#include "motor_bus.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "motor_bus";

/* 电机登记表：句柄与信息一一对应，下标同步维护 */
static zdt_x42_t    s_handles[MOTOR_MAX];
static motor_info_t s_info[MOTOR_MAX];
static size_t       s_count;

/* ================= 总线 ================= */

esp_err_t motor_bus_init(void)
{
    esp_err_t e = zdt_x42_bus_init(BUS_A_UART, BUS_A_TX_GPIO, BUS_A_RX_GPIO, MOTOR_BAUD);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "总线 A(UART%d) 初始化失败: %s", BUS_A_UART, esp_err_to_name(e));
        return e;
    }
    e = zdt_x42_bus_init(BUS_B_UART, BUS_B_TX_GPIO, BUS_B_RX_GPIO, MOTOR_BAUD);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "总线 B(UART%d) 初始化失败: %s", BUS_B_UART, esp_err_to_name(e));
        return e;
    }
    ESP_LOGI(TAG, "双总线就绪: A=UART%d(GPIO%d/%d) B=UART%d(GPIO%d/%d) @%d",
             BUS_A_UART, BUS_A_TX_GPIO, BUS_A_RX_GPIO,
             BUS_B_UART, BUS_B_TX_GPIO, BUS_B_RX_GPIO, MOTOR_BAUD);
    return ESP_OK;
}

/* ================= 登记表 ================= */

size_t motor_count(void)
{
    return s_count;
}

const motor_info_t *motor_at(size_t i)
{
    return (i < s_count) ? &s_info[i] : NULL;
}

int motor_id_of(const motor_info_t *info)
{
    if (!info) {
        return 0;
    }
    return (info->bus == BUS_B_UART ? BUS_B_ID_BASE : BUS_A_ID_BASE) + info->addr;
}

zdt_x42_t *motor_find(int id)
{
    if (id <= 0) {
        return NULL;
    }
    for (size_t i = 0; i < s_count; i++) {
        if (motor_id_of(&s_info[i]) == id) {
            return &s_handles[i];
        }
    }
    return NULL;
}

/* ================= 扫描 ================= */

/** 探测一个地址是否有电机应答（用短超时，避免慢） */
static bool probe(uart_port_t bus, uint8_t addr, uint16_t *fw, uint16_t *hw)
{
    zdt_x42_t tmp;
    zdt_x42_motor_init(&tmp, bus, addr);
    tmp.timeout_ms = SCAN_TIMEOUT_MS;
    return zdt_x42_read_version(&tmp, fw, hw) == ZDT_OK;
}

int motor_scan(int from, int to)
{
    /* 防御：地址 0 是广播地址，会让总线上所有电机同时应答，必须跳过 */
    if (from < 1) from = 1;
    if (to > 255) to = 255;
    if (to < from) return 0;

    s_count = 0;

    const uart_port_t buses[2] = { BUS_A_UART, BUS_B_UART };
    for (int b = 0; b < 2; b++) {
        for (int addr = from; addr <= to; addr++) {
            uint16_t fw = 0, hw = 0;
            if (!probe(buses[b], (uint8_t)addr, &fw, &hw)) {
                continue;
            }
            if (s_count >= MOTOR_MAX) {
                ESP_LOGW(TAG, "电机数已达上限 %d，忽略 UART%d 地址 %d",
                         MOTOR_MAX, buses[b], addr);
                continue;
            }
            /* 正式登记时用正常超时，探测用的短超时只用于扫描 */
            zdt_x42_motor_init(&s_handles[s_count], buses[b], (uint8_t)addr);
            s_handles[s_count].timeout_ms = MOTOR_TIMEOUT_MS;
            s_info[s_count].addr = (uint8_t)addr;
            s_info[s_count].bus  = buses[b];
            s_info[s_count].fw   = fw;
            s_info[s_count].hw   = hw;
            ESP_LOGI(TAG, "发现电机 ID=%d (UART%d addr=%d) fw=%u hw=%u",
                     motor_id_of(&s_info[s_count]), buses[b], addr, fw, hw);
            s_count++;
        }
    }
    ESP_LOGI(TAG, "扫描完成：共 %d 台", (int)s_count);
    return (int)s_count;
}

esp_err_t motor_sync_start_all(void)
{
    /* 广播只在单条总线上生效，所以每条在用总线各广播一次 */
    bool done_a = false, done_b = false;
    esp_err_t last = ESP_OK;

    for (size_t i = 0; i < s_count; i++) {
        bool *done = (s_info[i].bus == BUS_B_UART) ? &done_b : &done_a;
        if (*done) {
            continue;
        }
        *done = true;
        last = zdt_x42_sync_start(&s_handles[i]);
        if (last != ZDT_OK) {
            ESP_LOGW(TAG, "UART%d 广播同步启动失败: %d", s_info[i].bus, (int)last);
        }
    }
    return last;
}

/* ================= IO ================= */

esp_err_t io_init(void)
{
    gpio_config_t in_cfg = {
        .pin_bit_mask = 1ULL << PIN_LIMIT_SW,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* 默认下拉：未压到 = 低 */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t e = gpio_config(&in_cfg);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d 输入初始化失败", PIN_LIMIT_SW);
        return e;
    }

    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << PIN_MOS,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    e = gpio_config(&out_cfg);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d 输出初始化失败", PIN_MOS);
        return e;
    }

    gpio_set_level(PIN_MOS, 0);   /* 上电默认低 */
    ESP_LOGI(TAG, "IO 就绪: 限位输入 GPIO%d(下拉) / MOS 输出 GPIO%d(默认低)",
             PIN_LIMIT_SW, PIN_MOS);
    return ESP_OK;
}

int io_limit_level(void)
{
    return gpio_get_level(PIN_LIMIT_SW) ? 1 : 0;
}

void io_mos_set(int level)
{
    gpio_set_level(PIN_MOS, level ? 1 : 0);
}

int io_mos_level(void)
{
    return gpio_get_level(PIN_MOS) ? 1 : 0;
}
