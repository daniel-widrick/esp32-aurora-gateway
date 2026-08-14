#include "aurora_modbus.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "aurora";

#define RX_BUF_SIZE     512
#define RESP_TIMEOUT_MS 250   /* generous; the ABC answers far faster */
#define INTERFRAME_MS   10    /* >> t3.5 (~2.0 ms at 19200 8E1) */

/* Bus-idle detection before transmit. FREERTOS_HZ is 100 (10 ms/tick), too
 * coarse to time a 2 ms gap, so this uses microsecond timing and non-blocking
 * reads instead. */
#define T3_5_US              2000    /* ~3.5 char times at 19200 8E1 */
#define BUS_IDLE_MAX_WAIT_US 40000   /* give up waiting for quiet after this */

#define FC_READ_HOLDING   0x03
#define FC_WRITE_SINGLE   0x06

static SemaphoreHandle_t s_lock;
static uint8_t s_last_exception;

/* Bus-health counters, surfaced via /api/health so contention on the
 * AWL-shared segment is visible. */
static uint32_t s_crc_errors;
static uint32_t s_addr_mismatch;
static uint32_t s_bus_busy;

void aurora_bus_faults(uint32_t *crc_errors, uint32_t *addr_mismatch, uint32_t *bus_busy)
{
    if (crc_errors)   *crc_errors = s_crc_errors;
    if (addr_mismatch) *addr_mismatch = s_addr_mismatch;
    if (bus_busy)     *bus_busy = s_bus_busy;
}

/*
 * Wait for the bus to be silent for >= t3.5 before transmitting, so we never
 * begin a frame on top of the AWL's - which, if it aligned on a frame boundary
 * with a matching length, would pass CRC and be cached as our data (silent
 * corruption that CRC cannot catch). Drains any RX while waiting. Best-effort:
 * if the AWL is saturating the bus and it never goes quiet within
 * BUS_IDLE_MAX_WAIT_US, count it and transmit anyway rather than stall a poll.
 */
static void wait_bus_idle(void)
{
    uint8_t sink[64];
    int64_t last_activity = esp_timer_get_time();
    int64_t give_up = last_activity + BUS_IDLE_MAX_WAIT_US;
    while (1) {
        int n = uart_read_bytes(AURORA_UART_PORT, sink, sizeof(sink), 0);  /* non-blocking */
        int64_t now = esp_timer_get_time();
        if (n > 0) {
            last_activity = now;
        } else if (now - last_activity >= T3_5_US) {
            return;   /* quiet long enough */
        }
        if (now >= give_up) {
            s_bus_busy++;
            return;
        }
        esp_rom_delay_us(150);
    }
}

static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* Append the CRC (low byte first) and return the total frame length. */
static size_t frame_finish(uint8_t *buf, size_t len)
{
    uint16_t crc = crc16(buf, len);
    buf[len]     = (uint8_t)(crc & 0xFF);
    buf[len + 1] = (uint8_t)(crc >> 8);
    return len + 2;
}

static bool crc_ok(const uint8_t *frame, size_t len)
{
    if (len < 4) {
        return false;
    }
    uint16_t want = (uint16_t)(frame[len - 2] | (frame[len - 1] << 8));
    return crc16(frame, len - 2) == want;
}

/* Read exactly `len` bytes or fail. uart_read_bytes can return short, so keep
 * pulling until we have the whole thing or the deadline passes. */
static esp_err_t read_exact(uint8_t *buf, size_t len, TickType_t deadline)
{
    size_t got = 0;
    while (got < len) {
        TickType_t now = xTaskGetTickCount();
        /* Signed difference so the comparison stays correct across the ~497-day
         * tick rollover (a plain now >= deadline mis-fires for one window). */
        if ((int32_t)(deadline - now) <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        int n = uart_read_bytes(AURORA_UART_PORT, buf + got, len - got, deadline - now);
        if (n < 0) {
            return ESP_FAIL;
        }
        got += (size_t)n;
    }
    return ESP_OK;
}

/*
 * One request/response exchange. `resp` receives the validated response frame
 * with the CRC still attached; `resp_len` is set to its length.
 */
static esp_err_t transact(const uint8_t *req, size_t req_len,
                          uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
    /* Stale exception from a prior transaction must not leak into this one's
     * error report (it made timeouts look like register-permission errors). */
    s_last_exception = 0;

    uart_flush_input(AURORA_UART_PORT);
    wait_bus_idle();   /* don't start transmitting into the AWL's frame */

    int written = uart_write_bytes(AURORA_UART_PORT, req, req_len);
    if (written != (int)req_len) {
        ESP_LOGE(TAG, "uart_write_bytes wrote %d of %u", written, (unsigned)req_len);
        return ESP_FAIL;
    }
    /* In RS485 half-duplex mode the driver holds DE asserted until the last
     * stop bit is out; this wait keeps us from reading our own echo. */
    esp_err_t err = uart_wait_tx_done(AURORA_UART_PORT, pdMS_TO_TICKS(RESP_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(RESP_TIMEOUT_MS);

    /* addr, function, then either a byte count (FC03) or the first data byte. */
    if (resp_cap < 5) {
        return ESP_ERR_INVALID_SIZE;
    }
    err = read_exact(resp, 3, deadline);
    if (err != ESP_OK) {
        return err;
    }

    size_t total;
    if (resp[1] & 0x80) {
        /* Exception response: addr, fc|0x80, code, crc16. */
        err = read_exact(resp + 3, 2, deadline);
        if (err != ESP_OK) {
            return err;
        }
        total = 5;
        if (!crc_ok(resp, total)) {
            s_crc_errors++;
            return ESP_ERR_INVALID_CRC;
        }
        s_last_exception = resp[2];
        ESP_LOGW(TAG, "modbus exception 0x%02x on function 0x%02x",
                 resp[2], (unsigned)(resp[1] & 0x7F));
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (resp[1] == FC_READ_HOLDING) {
        size_t nbytes = resp[2];
        total = 3 + nbytes + 2;
        if (total > resp_cap) {
            return ESP_ERR_INVALID_SIZE;
        }
        err = read_exact(resp + 3, nbytes + 2, deadline);
    } else {
        /* FC06 echoes the request: addr, fc, reg_hi, reg_lo, val_hi, val_lo, crc16. */
        total = 8;
        if (total > resp_cap) {
            return ESP_ERR_INVALID_SIZE;
        }
        err = read_exact(resp + 3, 5, deadline);
    }
    if (err != ESP_OK) {
        return err;
    }

    if (resp[0] != AURORA_SLAVE_ADDR) {
        s_addr_mismatch++;
        ESP_LOGW(TAG, "response from addr %u, expected %u", resp[0], AURORA_SLAVE_ADDR);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (!crc_ok(resp, total)) {
        s_crc_errors++;
        ESP_LOGW(TAG, "bad CRC on %u-byte response", (unsigned)total);
        return ESP_ERR_INVALID_CRC;
    }

    *resp_len = total;
    return ESP_OK;
}

esp_err_t aurora_modbus_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t cfg = {
        .baud_rate = AURORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        /* 8E1. NOT 8N1 - assuming no parity gets you silence or garbage.
         * Confirmed twice over: U5MODE=0x8801 in the AWL firmware, and
         * ccutrer's "19200 baud, EVEN". */
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(AURORA_UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(AURORA_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(AURORA_UART_PORT, AURORA_PIN_TXD, AURORA_PIN_RXD,
                                 AURORA_PIN_DE, UART_PIN_NO_CHANGE));
    /* Hardware drives RTS (= the board's transceiver enable on GPIO21) around
     * each transmission, which is what we want at 19200 with tight turnaround. */
    ESP_ERROR_CHECK(uart_set_mode(AURORA_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "RS485 up: UART%d tx=%d rx=%d de=%d, %d 8E1, slave %d",
             AURORA_UART_PORT, AURORA_PIN_TXD, AURORA_PIN_RXD, AURORA_PIN_DE,
             AURORA_BAUD, AURORA_SLAVE_ADDR);
    return ESP_OK;
}

esp_err_t aurora_read_holding(uint16_t start, uint16_t count, uint16_t *out)
{
    if (count < 1 || count > 125 || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t req[8];
    req[0] = AURORA_SLAVE_ADDR;
    req[1] = FC_READ_HOLDING;
    req[2] = (uint8_t)(start >> 8);
    req[3] = (uint8_t)(start & 0xFF);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count & 0xFF);
    size_t req_len = frame_finish(req, 6);

    uint8_t resp[5 + 250];
    size_t resp_len = 0;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = transact(req, req_len, resp, sizeof(resp), &resp_len);
    vTaskDelay(pdMS_TO_TICKS(INTERFRAME_MS));
    xSemaphoreGive(s_lock);

    if (err != ESP_OK) {
        return err;
    }
    if (resp[2] != count * 2) {
        ESP_LOGW(TAG, "byte count %u, expected %u", resp[2], (unsigned)(count * 2));
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (uint16_t i = 0; i < count; i++) {
        out[i] = (uint16_t)((resp[3 + i * 2] << 8) | resp[4 + i * 2]);
    }
    return ESP_OK;
}

esp_err_t aurora_write_single(uint16_t reg, uint16_t value)
{
    uint8_t req[8];
    req[0] = AURORA_SLAVE_ADDR;
    req[1] = FC_WRITE_SINGLE;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = (uint8_t)(value >> 8);
    req[5] = (uint8_t)(value & 0xFF);
    size_t req_len = frame_finish(req, 6);

    uint8_t resp[8];
    size_t resp_len = 0;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = transact(req, req_len, resp, sizeof(resp), &resp_len);
    vTaskDelay(pdMS_TO_TICKS(INTERFRAME_MS));
    xSemaphoreGive(s_lock);

    if (err != ESP_OK) {
        return err;
    }
    /* A conforming slave echoes the request verbatim. */
    if (memcmp(req, resp, 6) != 0) {
        ESP_LOGW(TAG, "write echo mismatch for reg %u", reg);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

uint8_t aurora_last_exception(void)
{
    return s_last_exception;
}
