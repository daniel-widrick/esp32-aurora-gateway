#include "logbuf.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static char s_buf[LOGBUF_SIZE];
static size_t s_head;          /* next write position */
static bool s_wrapped;
static uint32_t s_total;       /* bytes ever written */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev;

static void append(const char *data, size_t len)
{
    if (len > LOGBUF_SIZE) {
        /* Only the tail can survive anyway; skip the doomed prefix. */
        data += len - LOGBUF_SIZE;
        len = LOGBUF_SIZE;
    }

    /* A spinlock rather than a mutex: this runs inside the logging path, which
     * must never block or take a lock that logging itself might already hold.
     * Kept to two bounded memcpys - at most LOGBUF_SIZE bytes, in practice one
     * short log line. */
    portENTER_CRITICAL(&s_mux);
    size_t first = LOGBUF_SIZE - s_head;
    if (first > len) {
        first = len;
    }
    memcpy(s_buf + s_head, data, first);
    if (len > first) {
        memcpy(s_buf, data + first, len - first);
        s_wrapped = true;
    }
    s_head += len;
    if (s_head >= LOGBUF_SIZE) {
        s_head -= LOGBUF_SIZE;
        s_wrapped = true;
    }
    s_total += len;
    portEXIT_CRITICAL(&s_mux);
}

static int logbuf_vprintf(const char *fmt, va_list args)
{
    char line[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (n > 0) {
        size_t len = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1;
        append(line, len);
    }

    /* Keep feeding the real console while one is still attached. */
    return s_prev ? s_prev(fmt, args) : n;
}

void logbuf_init(void)
{
    s_head = 0;
    s_wrapped = false;
    s_total = 0;
    memset(s_buf, 0, sizeof(s_buf));
    s_prev = esp_log_set_vprintf(logbuf_vprintf);
}

size_t logbuf_len(void)
{
    portENTER_CRITICAL(&s_mux);
    size_t len = s_wrapped ? LOGBUF_SIZE : s_head;
    portEXIT_CRITICAL(&s_mux);
    return len;
}

uint32_t logbuf_total_bytes(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t t = s_total;
    portEXIT_CRITICAL(&s_mux);
    return t;
}

size_t logbuf_dump(char *out, size_t cap)
{
    if (out == NULL || cap == 0) {
        return 0;
    }

    /*
     * Only the indices are read under the lock. The copy itself happens
     * outside it - a 6 KB memcpy with interrupts disabled is far too long a
     * critical section on a device running WiFi and TLS.
     *
     * The tradeoff: a task logging during the copy can overwrite bytes we are
     * still reading, garbling the oldest part of the output. That is
     * acceptable for a diagnostic log and cannot overrun anything, since every
     * index is bounded and `out` is written strictly within `cap`.
     */
    portENTER_CRITICAL(&s_mux);
    size_t head = s_head;
    bool wrapped = s_wrapped;
    portEXIT_CRITICAL(&s_mux);

    size_t avail = wrapped ? LOGBUF_SIZE : head;
    size_t room = cap - 1;
    size_t len = 0;

    if (wrapped) {
        /* Oldest data starts at head and runs to the end of the buffer. */
        size_t tail = LOGBUF_SIZE - head;
        size_t n = tail < room ? tail : room;
        memcpy(out, s_buf + head, n);
        len = n;
    }
    if (len < room) {
        size_t n2 = head < (room - len) ? head : (room - len);
        memcpy(out + len, s_buf, n2);
        len += n2;
    }

    (void)avail;
    out[len] = '\0';
    return len;
}
