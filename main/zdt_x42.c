/**
 * @file zdt_x42.c
 * @brief ZDT X 系列 V2.0 闭环步进驱动（x42s_v2.0）串口协议驱动实现。
 *
 * 一对多要点：
 *  - 总线初始化一次，每台电机一个 zdt_x42_t 句柄；
 *  - 所有事务用互斥锁串行化，多任务安全；
 *  - 发送前 flush 输入缓冲区，保证读到的第一个字节就是本机应答；
 *  - 应答帧长度不定（控制命令 4 字节，读取命令更长），
 *    采用"命中 地址+功能码 后按 5ms 帧间间隔判定帧结束"。
 */
#include "zdt_x42.h"
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "zdt_x42";

#define ZDT_BROADCAST_ADDR 0x00
#define FRAME_GAP_MS       5    /* 帧间间隔判定 */

static SemaphoreHandle_t s_bus_mutex[UART_NUM_MAX];
static bool s_bus_installed[UART_NUM_MAX];

bool zdt_x42_dump_rx = false;   /* 调试：打印事务收发原始字节 */

/* ---------------- 基础 ---------------- */

esp_err_t zdt_x42_bus_init(uart_port_t uart_num, int tx_gpio, int rx_gpio, int baud)
{
    if (uart_num >= UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_bus_installed[uart_num]) {
        s_bus_mutex[uart_num] = xSemaphoreCreateMutex();
        s_bus_installed[uart_num] = true;
    }

    uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(uart_num, &cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(uart_num, tx_gpio, rx_gpio,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(uart_num, 2048, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");
    ESP_LOGI(TAG, "bus ready: uart%d tx=GPIO%d rx=GPIO%d baud=%d",
             uart_num, tx_gpio, rx_gpio, baud);
    return ESP_OK;
}

void zdt_x42_motor_init(zdt_x42_t *m, uart_port_t uart_num, uint8_t addr)
{
    memset(m, 0, sizeof(*m));
    m->uart_num   = uart_num;
    m->addr       = addr;
    m->checksum   = ZDT_CHK_6B;   /* 出厂默认 */
    m->timeout_ms = 200;
}

static uint8_t calc_checksum(zdt_x42_checksum_t mode, const uint8_t *p, size_t len)
{
    if (mode == ZDT_CHK_6B) {
        return 0x6B;
    }
    uint8_t x = 0;
    for (size_t i = 0; i < len; i++) {
        x ^= p[i];
    }
    return x;
}

/* ---------------- 应答帧接收 ---------------- */

/**
 * @brief 等待本机应答帧：前两个字节匹配 [addr][func]（或错误帧 [addr][00]）
 *        即命中，之后按帧间间隔判结束。
 */
static esp_err_t read_response(zdt_x42_t *m, uint8_t exp_func,
                               uint8_t *out, size_t cap, size_t *len)
{
    uint8_t buf[ZDT_X42_RX_BUF_LEN];
    size_t n = 0;
    bool matched = false;
    int64_t deadline = (int64_t)esp_timer_get_time() / 1000 + (int64_t)m->timeout_ms;

    while (n < sizeof(buf)) {
        int64_t remain = deadline - (int64_t)esp_timer_get_time() / 1000;
        if (remain <= 0) {
            break;                      /* 总超时 */
        }
        /* 命中后用短超时判帧尾，未命中时等到总超时为止。
         * 注意：uart_read_bytes 的超时参数单位是 RTOS tick，不是毫秒！ */
        TickType_t wait = matched ? pdMS_TO_TICKS(FRAME_GAP_MS)
                                  : pdMS_TO_TICKS((uint32_t)remain);
        int r = uart_read_bytes(m->uart_num, buf + n, 1, wait);
        if (r <= 0) {
            if (matched) {
                break;                  /* 一帧收完 */
            }
            continue;                   /* 还没等到应答头字节 */
        }
        n++;
        if (!matched && n >= 2 && buf[0] == m->addr &&
            (buf[1] == exp_func || buf[1] == 0x00)) {
            matched = true;
        }
    }

    if (!matched || n < 2) {
        return ZDT_ERR_TIMEOUT;
    }
    size_t take = (n < cap) ? n : cap;
    memcpy(out, buf, take);
    *len = take;
    return ZDT_OK;
}

esp_err_t zdt_x42_transact(zdt_x42_t *m, const uint8_t *tx, size_t tx_len,
                           uint8_t *rx, size_t rx_cap, size_t *rx_len)
{
    if (!m || !tx || !rx || tx_len == 0 || tx_len + 1 > ZDT_X42_RX_BUF_LEN) {
        return ZDT_ERR_NULL;
    }

    uint8_t frame[ZDT_X42_RX_BUF_LEN];
    memcpy(frame, tx, tx_len);
    frame[tx_len] = calc_checksum(m->checksum, frame, tx_len);
    bool broadcast = (frame[0] == ZDT_BROADCAST_ADDR);

    if (xSemaphoreTake(s_bus_mutex[m->uart_num],
                       pdMS_TO_TICKS(m->timeout_ms)) != pdTRUE) {
        return ZDT_ERR_TIMEOUT;
    }

    esp_err_t ret = ZDT_OK;
    int64_t t0_ms = esp_timer_get_time() / 1000;
    uart_flush_input(m->uart_num);
    if (uart_write_bytes(m->uart_num, frame, (int)(tx_len + 1)) != (int)(tx_len + 1)) {
        ret = ZDT_ERR_TX;
        goto out;
    }
    if (broadcast) {
        *rx_len = 0;    /* 广播不等应答（只有地址 1 会回，留在缓冲里下次清掉） */
        goto out;
    }
    ret = read_response(m, frame[1], rx, rx_cap, rx_len);

out:
    if (zdt_x42_dump_rx) {
        int elapsed = (int)(esp_timer_get_time() / 1000 - t0_ms);
        ESP_LOG_BUFFER_HEXDUMP(TAG, frame, tx_len + 1, ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEXDUMP(TAG, rx, *rx_len, ESP_LOG_INFO);
        ESP_LOGI(TAG, "addr=%d transact=%dms ret=%d", frame[0], elapsed, (int)ret);
    }
    xSemaphoreGive(s_bus_mutex[m->uart_num]);
    return ret;
}

/* ---------------- 应答解析 ---------------- */

/** 控制命令应答：[addr][func][status][chk]，错误帧 [addr][00][EE][chk] */
static esp_err_t parse_ack(const uint8_t *rx, size_t len)
{
    if (len >= 4 && rx[1] == 0x00 && rx[2] == 0xEE) {
        return ZDT_ERR_REJECT;
    }
    if (len >= 4 && rx[2] == 0x02) {
        return ZDT_OK;
    }
    if (len >= 4 && rx[2] == 0xE2) {
        return ZDT_ERR_COND;
    }
    return ZDT_ERR_FRAME;
}

/** 读取命令应答：[addr][func][数据...][chk]，错误帧同上 */
static esp_err_t parse_data_frame(const uint8_t *rx, size_t len,
                                  uint8_t exp_func, size_t min_len)
{
    if (len >= 4 && rx[1] == 0x00 && rx[2] == 0xEE) {
        return ZDT_ERR_REJECT;
    }
    if (len < min_len || rx[1] != exp_func) {
        return ZDT_ERR_FRAME;
    }
    return ZDT_OK;
}

/* ---------------- 控制命令 ---------------- */

esp_err_t zdt_x42_enable(zdt_x42_t *m, bool enable, bool sync)
{
    uint8_t tx[] = { m->addr, 0xF3, 0xAB, enable ? 0x01 : 0x00, sync ? 0x01 : 0x00 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_speed_mode(zdt_x42_t *m, bool cw, uint16_t speed_rpm,
                             uint8_t acc_gear, bool sync)
{
    uint8_t tx[] = {
        m->addr, 0xF6, cw ? 0x00 : 0x01,
        (uint8_t)(speed_rpm >> 8), (uint8_t)speed_rpm,
        acc_gear, sync ? 0x01 : 0x00,
    };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_pos_mode(zdt_x42_t *m, bool cw, uint16_t speed_rpm,
                           uint8_t acc_gear, uint32_t pulses, bool absolute,
                           bool sync)
{
    uint8_t tx[] = {
        m->addr, 0xFD, cw ? 0x00 : 0x01,
        (uint8_t)(speed_rpm >> 8), (uint8_t)speed_rpm,
        acc_gear,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses,
        absolute ? 0x01 : 0x00, sync ? 0x01 : 0x00,
    };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_stop(zdt_x42_t *m, bool sync)
{
    uint8_t tx[] = { m->addr, 0xFE, 0x98, sync ? 0x01 : 0x00 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_sync_start(zdt_x42_t *m)
{
    uint8_t tx[] = { ZDT_BROADCAST_ADDR, 0xFF, 0x66 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    return zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
}

esp_err_t zdt_x42_vib_step(zdt_x42_t *m2, zdt_x42_t *m3, bool cw, bool mirror,
                           uint16_t speed_rpm, uint8_t acc_gear, uint32_t pulses)
{
    /* 仅支持单机（m3==NULL）。双机龙门必须走 pos_mode+sync_start 事务路径：
     * 突发连续写会被 EMM5.0 电机解析器随机判错（回 00 EE）。 */
    if (!m2 || m3) {
        return ZDT_ERR_NULL;
    }
    uint8_t f2[] = {
        m2->addr, 0xFD, cw ? 0x00 : 0x01,
        (uint8_t)(speed_rpm >> 8), (uint8_t)speed_rpm,
        acc_gear,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses,
        0x00, 0x00,   /* REL + 立即执行 */
    };

    if (xSemaphoreTake(s_bus_mutex[m2->uart_num],
                       pdMS_TO_TICKS(m2->timeout_ms)) != pdTRUE) {
        return ZDT_ERR_TIMEOUT;
    }
    uart_flush_input(m2->uart_num);
    uint8_t buf2[sizeof(f2) + 1];
    memcpy(buf2, f2, sizeof(f2));
    buf2[sizeof(f2)] = calc_checksum(m2->checksum, f2, sizeof(f2));
    if (uart_write_bytes(m2->uart_num, buf2, sizeof(buf2)) != (int)sizeof(buf2)) {
        xSemaphoreGive(s_bus_mutex[m2->uart_num]);
        return ZDT_ERR_TX;
    }
    xSemaphoreGive(s_bus_mutex[m2->uart_num]);
    return ZDT_OK;
}

esp_err_t zdt_x42_home(zdt_x42_t *m, uint8_t home_mode, bool sync)
{
    uint8_t tx[] = { m->addr, 0x9A, home_mode, sync ? 0x01 : 0x00 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_set_home_params(zdt_x42_t *m, const zdt_x42_home_params_t *p, bool store)
{
    uint8_t tx[] = {
        m->addr, 0x4C, 0xAE, store ? 0x01 : 0x00,
        p->mode, p->dir_cw ? 0x00 : 0x01,
        (uint8_t)(p->speed_rpm >> 8), (uint8_t)p->speed_rpm,
        (uint8_t)(p->timeout_ms >> 24), (uint8_t)(p->timeout_ms >> 16),
        (uint8_t)(p->timeout_ms >> 8), (uint8_t)p->timeout_ms,
        (uint8_t)(p->clog_rpm >> 8), (uint8_t)p->clog_rpm,
        (uint8_t)(p->clog_ma >> 8), (uint8_t)p->clog_ma,
        (uint8_t)(p->clog_ms >> 8), (uint8_t)p->clog_ms,
        p->auto_home ? 0x01 : 0x00,
    };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_read_home_params(zdt_x42_t *m, zdt_x42_home_params_t *p)
{
    uint8_t tx[] = { m->addr, 0x22 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x22, 18);   /* addr 22 15B数据 chk */
    if (e != ZDT_OK) {
        return e;
    }
    p->mode       = rx[2];
    p->dir_cw     = (rx[3] == 0x00);
    p->speed_rpm  = (uint16_t)((uint16_t)rx[4] << 8 | rx[5]);
    p->timeout_ms = (uint32_t)rx[6] << 24 | (uint32_t)rx[7] << 16 |
                    (uint32_t)rx[8] << 8 | (uint32_t)rx[9];
    p->clog_rpm   = (uint16_t)((uint16_t)rx[10] << 8 | rx[11]);
    p->clog_ma    = (uint16_t)((uint16_t)rx[12] << 8 | rx[13]);
    p->clog_ms    = (uint16_t)((uint16_t)rx[14] << 8 | rx[15]);
    p->auto_home  = (rx[16] == 0x01);
    return ZDT_OK;
}

esp_err_t zdt_x42_read_home_status(zdt_x42_t *m, uint8_t *flags)
{
    uint8_t tx[] = { m->addr, 0x3B };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x3B, 4);
    if (e != ZDT_OK) {
        return e;
    }
    *flags = rx[2];
    return ZDT_OK;
}

esp_err_t zdt_x42_clear_position(zdt_x42_t *m)
{
    uint8_t tx[] = { m->addr, 0x0A, 0x6D };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_set_single_zero(zdt_x42_t *m, bool store)
{
    uint8_t tx[] = { m->addr, 0x93, 0x88, store ? 0x01 : 0x00 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_release_stall(zdt_x42_t *m)
{
    uint8_t tx[] = { m->addr, 0x0E, 0x52 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_trigger_calibration(zdt_x42_t *m)
{
    uint8_t tx[] = { m->addr, 0x06, 0x45 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_restore_factory(zdt_x42_t *m)
{
    uint8_t tx[] = { m->addr, 0x0F, 0x5F };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_set_subdivision(zdt_x42_t *m, uint8_t microstep, bool store)
{
    uint8_t tx[] = { m->addr, 0x84, 0x8A, store ? 0x01 : 0x00, microstep };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

esp_err_t zdt_x42_set_control_mode(zdt_x42_t *m, uint8_t mode, bool store)
{
    uint8_t tx[] = { m->addr, 0x46, 0x69, store ? 0x01 : 0x00, mode };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    return (e == ZDT_OK) ? parse_ack(rx, rl) : e;
}

/* ---------------- 读取命令 ---------------- */

esp_err_t zdt_x42_read_status(zdt_x42_t *m, zdt_x42_status_t *st)
{
    uint8_t tx[] = { m->addr, 0x3A };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x3A, 4);
    if (e != ZDT_OK) {
        return e;
    }
    uint8_t f = rx[2];
    st->enable = (f >> 0) & 0x01;
    st->inpos  = (f >> 1) & 0x01;
    st->stall  = (f >> 2) & 0x01;
    st->clog   = (f >> 3) & 0x01;
    return ZDT_OK;
}

esp_err_t zdt_x42_read_position(zdt_x42_t *m, int32_t *pos_raw)
{
    uint8_t tx[] = { m->addr, 0x36 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x36, 8);
    if (e != ZDT_OK) {
        return e;
    }
    int32_t v = (int32_t)((uint32_t)rx[3] << 24 | (uint32_t)rx[4] << 16 |
                          (uint32_t)rx[5] << 8 | (uint32_t)rx[6]);
    *pos_raw = (rx[2] == 0x01) ? -v : v;   /* 0x01 = 负，0~65535 表示一圈 */
    return ZDT_OK;
}

esp_err_t zdt_x42_read_speed(zdt_x42_t *m, int32_t *speed_rpm)
{
    uint8_t tx[] = { m->addr, 0x35 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x35, 6);
    if (e != ZDT_OK) {
        return e;
    }
    int32_t v = (int32_t)((uint16_t)rx[3] << 8 | rx[4]);   /* 单位 RPM */
    *speed_rpm = (rx[2] == 0x01) ? -v : v;
    return ZDT_OK;
}

esp_err_t zdt_x42_read_encoder(zdt_x42_t *m, uint16_t *raw)
{
    uint8_t tx[] = { m->addr, 0x31 };   /* EMM5.0：校准后编码器值 0~65535 */
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x31, 5);
    if (e != ZDT_OK) {
        return e;
    }
    *raw = (uint16_t)((uint16_t)rx[2] << 8 | rx[3]);
    return ZDT_OK;
}

esp_err_t zdt_x42_read_bus_voltage(zdt_x42_t *m, uint16_t *mv)
{
    uint8_t tx[] = { m->addr, 0x24 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x24, 5);
    if (e != ZDT_OK) {
        return e;
    }
    *mv = (uint16_t)((uint16_t)rx[2] << 8 | rx[3]);
    return ZDT_OK;
}

esp_err_t zdt_x42_read_phase_current(zdt_x42_t *m, uint16_t *ma)
{
    uint8_t tx[] = { m->addr, 0x27 };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    e = parse_data_frame(rx, rl, 0x27, 5);
    if (e != ZDT_OK) {
        return e;
    }
    *ma = (uint16_t)((uint16_t)rx[2] << 8 | rx[3]);
    return ZDT_OK;
}

esp_err_t zdt_x42_read_version(zdt_x42_t *m, uint16_t *fw, uint16_t *hw)
{
    uint8_t tx[] = { m->addr, 0x1F };
    uint8_t rx[ZDT_X42_RX_BUF_LEN];
    size_t rl = 0;
    esp_err_t e = zdt_x42_transact(m, tx, sizeof(tx), rx, sizeof(rx), &rl);
    if (e != ZDT_OK) {
        return e;
    }
    /* 应答：地址 1F 固件高 固件低 硬件高 硬件低 校验，共 7 字节 */
    e = parse_data_frame(rx, rl, 0x1F, 7);
    if (e != ZDT_OK) {
        return e;
    }
    *fw = (uint16_t)((uint16_t)rx[2] << 8 | rx[3]);
    *hw = (uint16_t)((uint16_t)rx[4] << 8 | rx[5]);
    return ZDT_OK;
}
