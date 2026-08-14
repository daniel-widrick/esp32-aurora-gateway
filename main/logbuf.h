/*
 * In-memory ring buffer of the log output.
 *
 * Once the board is at the furnace there is no USB console, so without this
 * there is no way to see why anything misbehaved. Everything ESP_LOG writes is
 * captured here and served over the API, while still going to the console for
 * as long as one is attached.
 *
 * Memory is fixed and pre-allocated: one buffer of LOGBUF_SIZE in .bss, no
 * heap, no flash. It can never grow, and old bytes are overwritten by new ones.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#define LOGBUF_SIZE 6144

void logbuf_init(void);

/* Copy the buffered log into `out` (NUL-terminated, oldest first). Safe for
 * any `cap`; the result is truncated to fit. Returns bytes written. */
size_t logbuf_dump(char *out, size_t cap);

/* Bytes currently buffered. */
size_t logbuf_len(void);

/* Total log bytes ever written, including those already overwritten - lets you
 * see that logging is alive even when the visible window looks unchanged. */
uint32_t logbuf_total_bytes(void);
